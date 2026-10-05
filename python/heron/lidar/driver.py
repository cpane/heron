"""Command sequencing and the scan stream.

This is where the sensor's *stateful* behaviour lives -- the parts that are
not pure decoding and that a C++ implementation has to get right for reasons
that are not obvious from the protocol document:

  * STOP leaves bytes in flight; you must settle and flush before the next
    command, or the next descriptor gets parsed out of stale scan nodes.
  * RESET replies with an ASCII firmware banner, not a descriptor. A read loop
    that expects ``A5 5A`` wedges on it.
  * The motor is the DTR line, and it keeps spinning if the process dies.
  * The scan stream has no frame sync word, so byte alignment can only be
    recovered from redundancy inside the nodes.
"""

from __future__ import annotations

import signal
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Iterator

from . import express as express_mod
from . import protocol
from .protocol import (
    Command,
    ConfType,
    Descriptor,
    DeviceInfo,
    Health,
    ProtocolError,
    Sample,
    SampleRate,
)
from .transport import Transport

#: STOP is not acknowledged, so the wait is the only way to know the device
#: has finished. The datasheet asks for at least 1 ms; 20 ms costs nothing at
#: bring-up and removes a whole class of intermittent failure.
STOP_SETTLE_S = 0.020

#: Upper bound on how long to wait for RESET's banner before giving up.
#: Measured on this unit: the banner's 64 bytes arrive in a single burst
#: 0.671-0.672 s after the request, and GET_INFO is accepted 4 ms later --
#: the device is ready as soon as the banner is out. So the correct rule is
#: "wait for the banner", not "sleep a fixed time"; this is only the timeout.
RESET_TIMEOUT_S = 2.000

#: How long the line must stay quiet before the banner is considered complete.
RESET_QUIET_S = 0.100

#: The motor takes a couple of revolutions to reach a stable speed.
MOTOR_SPINUP_S = 2.0

#: Read size. Large enough that one syscall usually drains the kernel buffer,
#: small enough that chunk timestamps stay meaningful as arrival times.
READ_CHUNK_BYTES = 512


class LidarError(Exception):
    """The device did not behave as the protocol requires."""


class LidarTimeout(LidarError):
    """The device went quiet when a response was expected."""


@dataclass
class ScanSample:
    """A decoded sample plus where and when it came from."""

    sample: Sample
    t_mono_ns: int  # Arrival time of the chunk containing it.
    raw_offset: int  # Byte offset in the .rpraw capture, or -1 if untapped.


@dataclass
class ResyncEvent:
    t_mono_ns: int
    bytes_discarded: int
    raw_offset: int
    detail: str


@dataclass
class StreamCounters:
    samples: int = 0
    chunks: int = 0
    bytes_read: int = 0
    bytes_discarded: int = 0
    resyncs: int = 0
    timeouts: int = 0
    chunk_arrivals: deque[int] = field(default_factory=lambda: deque(maxlen=4096))
    max_in_waiting: int = 0


class _OffsetBuffer:
    """A byte buffer that remembers each byte's offset in the capture file.

    Needed because a node can straddle two reads, so the file offset of a
    sample is not simply "bytes consumed so far" -- chunk headers sit between
    the payloads in the raw file. Marks are kept in absolute-consumed
    coordinates and pruned as they fall behind, so lookup stays O(1) amortised.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self._marks: deque[tuple[int, int]] = deque()
        self._start = 0

    def append(self, file_offset: int, data: bytes) -> None:
        self._marks.append((self._start + len(self._buf), file_offset))
        self._buf += data

    def __len__(self) -> int:
        return len(self._buf)

    def __getitem__(self, index: int) -> int:
        return self._buf[index]

    def peek(self, n: int) -> bytes:
        return bytes(self._buf[:n])

    def _prune(self) -> None:
        """Drop marks the read position has already passed.

        This has to run before reading marks[0], not only after consuming.
        A chunk is appended only once the previous one is fully consumed, so
        at the moment of the append the read position already sits exactly on
        the new mark. Reading marks[0] first would extrapolate from the old
        chunk and land 20 bytes early -- one chunk header -- for every node
        that happens to begin on a chunk boundary.
        """
        while len(self._marks) > 1 and self._marks[1][0] <= self._start:
            self._marks.popleft()

    def head_offset(self) -> int:
        """Capture-file offset of the first buffered byte, or -1."""
        self._prune()
        if not self._marks:
            return -1
        mark_pos, file_offset = self._marks[0]
        return file_offset + (self._start - mark_pos)

    def consume(self, n: int) -> tuple[bytes, int]:
        offset = self.head_offset()
        data = bytes(self._buf[:n])
        del self._buf[:n]
        self._start += n
        self._prune()
        return data, offset


class Lidar:
    """Driver for one RPLIDAR over a :class:`Transport`."""

    def __init__(
        self,
        transport: Transport,
        on_command: Callable[[str, bytes, int], None] | None = None,
        on_descriptor: Callable[[Descriptor, int], None] | None = None,
        on_resync: Callable[[ResyncEvent], None] | None = None,
    ) -> None:
        self.transport = transport
        self.counters = StreamCounters()
        self._on_command = on_command
        self._on_descriptor = on_descriptor
        self._on_resync = on_resync
        self._scanning = False

    # -- low level ---------------------------------------------------------

    def _send(self, command: Command, payload: bytes | None = None) -> None:
        frame = protocol.build_request(command, payload)
        self.transport.write(frame)
        if self._on_command:
            # A request is a TX chunk, so it must be located by the TX offset;
            # reporting the last RX offset here points at unrelated bytes.
            self._on_command(
                command.name,
                frame,
                int(getattr(self.transport, "last_tx_offset", -1)),
            )

    def _tap_offset(self) -> int:
        return int(getattr(self.transport, "last_rx_offset", -1))

    def _read_exact(self, count: int, timeout: float) -> bytes:
        """Read exactly ``count`` bytes or raise :class:`LidarTimeout`."""
        deadline = time.monotonic() + timeout
        out = bytearray()

        while len(out) < count:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise LidarTimeout(
                    f"wanted {count} bytes, got {len(out)} in {timeout:.2f}s"
                    + (f": {bytes(out).hex()}" if out else "")
                )
            _, data = self.transport.read_chunk(count - len(out), remaining)
            out += data

        return bytes(out)

    def _read_descriptor(self, timeout: float = 2.0) -> Descriptor:
        raw = self._read_exact(protocol.DESCRIPTOR_LEN, timeout)
        try:
            descriptor = protocol.parse_descriptor(raw)
        except ProtocolError as exc:
            raise LidarError(f"bad response descriptor: {exc}") from exc

        if self._on_descriptor:
            self._on_descriptor(descriptor, self._tap_offset())
        return descriptor

    def _single_response(
        self,
        command: Command,
        expected_type: int,
        timeout: float = 2.0,
    ) -> tuple[Descriptor, bytes]:
        self._send(command)
        descriptor = self._read_descriptor(timeout)

        if descriptor.data_type != expected_type:
            raise LidarError(
                f"{command.name}: expected data type 0x{expected_type:02x}, "
                f"got 0x{descriptor.data_type:02x}"
            )
        if descriptor.is_stream:
            raise LidarError(f"{command.name}: expected a single response")

        return descriptor, self._read_exact(descriptor.length, timeout)

    # -- commands ----------------------------------------------------------

    def stop(self) -> None:
        """Stop scanning.

        STOP is not acknowledged. The settle-then-flush is mandatory: without
        it, scan nodes still in the pipe get parsed as the *next* command's
        descriptor, which looks like a protocol bug much later.
        """
        self._send(Command.STOP)
        self._scanning = False
        time.sleep(STOP_SETTLE_S)
        flush = getattr(self.transport, "flush_input", None)
        if flush:
            flush()

    def reset(self) -> bytes:
        """Reboot the device and return the ASCII banner it emits.

        The reply is **not** descriptor-framed -- it is plain CRLF text::

            RP LIDAR System.
            Firmware Ver 1.29 - rtm, HW Ver 7
            Model: 18

        A read loop expecting ``A5 5A`` will wedge on it, which is why this
        drains it explicitly and hands it back for probe 1 to display.

        Readiness is detected by waiting for the banner to finish rather than
        by sleeping a fixed interval. Measured on this unit, the device
        accepts GET_INFO 4 ms after the banner's last byte, so a blind sleep
        is both less reliable (a slower board could still be booting) and
        slower than necessary.
        """
        self._send(Command.RESET)

        deadline = time.monotonic() + RESET_TIMEOUT_S
        banner = bytearray()
        last_data = None

        while time.monotonic() < deadline:
            _, data = self.transport.read_chunk(256, 0.02)
            if data:
                banner += data
                last_data = time.monotonic()
            elif last_data is not None:
                if time.monotonic() - last_data >= RESET_QUIET_S:
                    break

        flush = getattr(self.transport, "flush_input", None)
        if flush:
            flush()
        return bytes(banner)

    def get_info(self) -> tuple[Descriptor, bytes, DeviceInfo]:
        descriptor, payload = self._single_response(
            Command.GET_INFO, protocol.DATA_TYPE_DEVICE_INFO
        )
        return descriptor, payload, protocol.decode_device_info(payload)

    def get_health(self) -> tuple[Descriptor, bytes, Health]:
        descriptor, payload = self._single_response(
            Command.GET_HEALTH, protocol.DATA_TYPE_HEALTH
        )
        return descriptor, payload, protocol.decode_health(payload)

    def get_samplerate(self) -> tuple[Descriptor, bytes, SampleRate]:
        descriptor, payload = self._single_response(
            Command.GET_SAMPLERATE, protocol.DATA_TYPE_SAMPLERATE
        )
        return descriptor, payload, protocol.decode_samplerate(payload)

    # -- device configuration ---------------------------------------------

    def get_lidar_conf(
        self,
        conf_type: int,
        mode: int | None = None,
        timeout: float = 2.0,
    ) -> bytes:
        """Query one GET_LIDAR_CONF item. Returns the answer bytes.

        Added in firmware 1.24, so an older unit will simply not answer.
        """
        payload = protocol.build_conf_payload(conf_type, mode)
        self._send(Command.GET_LIDAR_CONF, payload)
        descriptor = self._read_descriptor(timeout)

        if descriptor.data_type != protocol.DATA_TYPE_LIDAR_CONF:
            raise LidarError(
                f"GET_LIDAR_CONF: expected data type 0x20, "
                f"got 0x{descriptor.data_type:02x}"
            )

        raw = self._read_exact(descriptor.length, timeout)
        return protocol.decode_conf_response(raw, conf_type)

    def scan_mode_count(self) -> int:
        data = self.get_lidar_conf(ConfType.SCAN_MODE_COUNT)
        return int.from_bytes(data[:2], "little")

    def typical_scan_mode(self) -> int:
        data = self.get_lidar_conf(ConfType.SCAN_MODE_TYPICAL)
        return int.from_bytes(data[:2], "little")

    def scan_mode_name(self, mode: int) -> str:
        data = self.get_lidar_conf(ConfType.SCAN_MODE_NAME, mode)
        return data.split(b"\x00")[0].decode("ascii", "replace")

    def scan_mode_us_per_sample(self, mode: int) -> float:
        """Microseconds per sample for this mode. Q8 fixed point on the wire."""
        data = self.get_lidar_conf(ConfType.SCAN_MODE_US_PER_SAMPLE, mode)
        return int.from_bytes(data[:4], "little") / 256.0

    def scan_mode_max_distance_m(self, mode: int) -> float:
        """Maximum range in metres. Also Q8, but the SDK keeps only the
        integer part (``>> 8``) rather than dividing."""
        data = self.get_lidar_conf(ConfType.SCAN_MODE_MAX_DISTANCE, mode)
        return int.from_bytes(data[:4], "little") / 256.0

    def scan_mode_ans_type(self, mode: int) -> int:
        """The scan data type this mode streams (0x81 legacy, 0x82 express)."""
        data = self.get_lidar_conf(ConfType.SCAN_MODE_ANS_TYPE, mode)
        return data[0]

    def enumerate_scan_modes(self) -> list[dict[str, object]]:
        """Everything the device will say about each of its scan modes."""
        modes = []
        for mode in range(self.scan_mode_count()):
            modes.append(
                {
                    "id": mode,
                    "name": self.scan_mode_name(mode),
                    "us_per_sample": self.scan_mode_us_per_sample(mode),
                    "max_distance_m": self.scan_mode_max_distance_m(mode),
                    "ans_type": self.scan_mode_ans_type(mode),
                }
            )
        return modes

    # -- motor -------------------------------------------------------------

    def set_motor(self, running: bool) -> None:
        self.transport.set_motor(running)  # type: ignore[attr-defined]

    def spin_up(self, settle: float = MOTOR_SPINUP_S) -> None:
        """Start the motor and wait for it to reach a stable speed."""
        self.set_motor(True)
        time.sleep(settle)

    # -- scanning ----------------------------------------------------------

    def start_scan(self, timeout: float = 2.0) -> Descriptor:
        """Issue SCAN and consume the descriptor. Returns it for display."""
        self._send(Command.SCAN)
        descriptor = self._read_descriptor(timeout)

        if descriptor.data_type != protocol.DATA_TYPE_LEGACY_SCAN:
            raise LidarError(
                f"SCAN: expected data type 0x81, got 0x{descriptor.data_type:02x}"
            )
        if not descriptor.is_stream:
            raise LidarError("SCAN: expected a multi-response descriptor")
        if descriptor.length != protocol.LEGACY_NODE_LEN:
            raise LidarError(
                f"SCAN: expected {protocol.LEGACY_NODE_LEN}-byte nodes, "
                f"got {descriptor.length}"
            )

        self._scanning = True
        return descriptor

    def start_force_scan(self, timeout: float = 2.0) -> Descriptor:
        """Issue FORCE_SCAN, which scans regardless of motor rotation.

        Useful as a bring-up check: it produces a data stream even with the
        motor stopped, which separates "the serial link and the ranging core
        work" from "the motor is turning". The reply is the same legacy node
        stream as SCAN, so :meth:`samples` consumes it unchanged.
        """
        self._send(Command.FORCE_SCAN)
        descriptor = self._read_descriptor(timeout)

        if descriptor.data_type != protocol.DATA_TYPE_LEGACY_SCAN:
            raise LidarError(
                f"FORCE_SCAN: expected data type 0x81, "
                f"got 0x{descriptor.data_type:02x}"
            )
        if descriptor.length != protocol.LEGACY_NODE_LEN:
            raise LidarError(
                f"FORCE_SCAN: expected {protocol.LEGACY_NODE_LEN}-byte nodes, "
                f"got {descriptor.length}"
            )

        self._scanning = True
        return descriptor

    def samples(
        self,
        duration: float | None = None,
        read_timeout: float = 1.0,
        skip_bytes: int = 0,
        stop: Callable[[], bool] | None = None,
    ) -> Iterator[ScanSample]:
        """Yield samples until ``duration`` elapses or the stream ends.

        ``skip_bytes`` deliberately discards bytes right after the descriptor
        so probe 2 can demonstrate resync recovering from a lost alignment.

        ``stop`` is polled on every read timeout, and ends the stream when it
        returns True. See :meth:`raw_chunks` for why it exists.
        """
        buffer = _OffsetBuffer()
        counters = self.counters
        deadline = None if duration is None else time.monotonic() + duration
        to_skip = skip_bytes
        last_t = 0

        while deadline is None or time.monotonic() < deadline:
            t_mono_ns, data = self.transport.read_chunk(
                READ_CHUNK_BYTES, read_timeout
            )

            if not data:
                counters.timeouts += 1
                if getattr(self.transport, "exhausted", False):
                    return
                if stop is not None and stop():
                    return
                continue

            last_t = t_mono_ns
            counters.chunks += 1
            counters.bytes_read += len(data)
            counters.chunk_arrivals.append(t_mono_ns)
            in_waiting = getattr(self.transport, "in_waiting", 0)
            counters.max_in_waiting = max(counters.max_in_waiting, in_waiting)

            if to_skip:
                drop = min(to_skip, len(data))
                data = data[drop:]
                to_skip -= drop
                if not data:
                    continue

            buffer.append(self._tap_offset(), data)

            while len(buffer) >= protocol.LEGACY_NODE_LEN:
                if not protocol.looks_like_node_start(buffer[0], buffer[1]):
                    if not self._resync(buffer, last_t):
                        break
                    continue

                node, offset = buffer.consume(protocol.LEGACY_NODE_LEN)
                counters.samples += 1
                yield ScanSample(
                    sample=protocol.decode_legacy_node(node),
                    t_mono_ns=last_t,
                    raw_offset=offset,
                )

    # -- express scanning --------------------------------------------------

    def start_express_scan(self, timeout: float = 2.0) -> Descriptor:
        """Issue EXPRESS_SCAN and consume the descriptor.

        Unlike SCAN, this request carries a payload, so it is checksummed.
        Getting that wrong is silent: the device simply never replies.
        """
        self._send(Command.EXPRESS_SCAN, express_mod.EXPRESS_SCAN_PAYLOAD)
        descriptor = self._read_descriptor(timeout)

        if descriptor.data_type != protocol.DATA_TYPE_EXPRESS_SCAN:
            raise LidarError(
                f"EXPRESS_SCAN: expected data type 0x82, "
                f"got 0x{descriptor.data_type:02x}"
            )
        if not descriptor.is_stream:
            raise LidarError("EXPRESS_SCAN: expected a multi-response descriptor")
        if descriptor.length != express_mod.CAPSULE_LEN:
            raise LidarError(
                f"EXPRESS_SCAN: expected {express_mod.CAPSULE_LEN}-byte "
                f"capsules, got {descriptor.length}"
            )

        self._scanning = True
        return descriptor

    def start_raw_express(self, working_mode: int, timeout: float = 2.0) -> Descriptor:
        """EXPRESS_SCAN with an explicit working_mode, format not assumed.

        :meth:`start_express_scan` insists on an 84-byte 0x82 stream, which is
        right for the express format and wrong for everything else. This
        variant returns whatever descriptor the device answers with and lets
        the caller dispatch on ``data_type``.

        That is the only safe shape for a mode-experiment tool. The device
        reports five scan modes, three of which stream ultra capsules (0x84,
        132 bytes) that this Python library does not decode (the C++ library
        does, through the vendor SDK), and the mapping from scan-mode
        id to working_mode is not verified on this unit. Asking the device and
        believing its answer beats predicting it.
        """
        self._send(
            Command.EXPRESS_SCAN, express_mod.express_scan_payload(working_mode)
        )
        descriptor = self._read_descriptor(timeout)

        if not descriptor.is_stream:
            raise LidarError(
                f"EXPRESS_SCAN(working_mode={working_mode}): expected a "
                "multi-response descriptor"
            )

        self._scanning = True
        return descriptor

    def raw_chunks(
        self,
        duration: float | None = None,
        read_timeout: float = 1.0,
        stop: Callable[[], bool] | None = None,
    ) -> Iterator[tuple[int, bytes]]:
        """Yield ``(arrival_ns, bytes)`` straight off the transport, undecoded.

        For streams this repository cannot interpret. Counting bytes and
        frames still answers useful questions -- is it alive, at what rate,
        does the framing hold -- without inventing geometry that cannot be
        computed.

        ``stop`` is polled on every read timeout, and ends the stream when it
        returns True. Without it a device that goes silent blocks the caller
        forever: a timeout yields nothing, so a consumer that checks for
        "should I stop?" between items never gets the chance. Measured
        2026-10-01 -- mode 4 sent no bytes at all in one run of four, and the
        server's worker could not be stopped until the process was signalled.
        """
        deadline = None if duration is None else time.monotonic() + duration
        while deadline is None or time.monotonic() < deadline:
            t_mono_ns, data = self.transport.read_chunk(READ_CHUNK_BYTES, read_timeout)
            if not data:
                self.counters.timeouts += 1
                if getattr(self.transport, "exhausted", False):
                    return
                if stop is not None and stop():
                    return
                continue
            self.counters.chunks += 1
            self.counters.bytes_read += len(data)
            yield t_mono_ns, data

    def express_samples(
        self,
        duration: float | None = None,
        read_timeout: float = 1.0,
        stop: Callable[[], bool] | None = None,
    ) -> Iterator[ScanSample]:
        """Yield samples decoded from express capsules.

        Note the inherent delay: a capsule's samples are only emitted once the
        following capsule has arrived, because the angles are interpolated
        between the two start angles.

        ``stop`` is polled on every read timeout, and ends the stream when it
        returns True. See :meth:`raw_chunks` for why it exists.
        """
        buffer = _OffsetBuffer()
        decoder = express_mod.ExpressDecoder()
        counters = self.counters
        deadline = None if duration is None else time.monotonic() + duration
        last_t = 0
        self.express_decoder = decoder

        while deadline is None or time.monotonic() < deadline:
            t_mono_ns, data = self.transport.read_chunk(
                READ_CHUNK_BYTES, read_timeout
            )

            if not data:
                counters.timeouts += 1
                if getattr(self.transport, "exhausted", False):
                    return
                if stop is not None and stop():
                    return
                continue

            last_t = t_mono_ns
            counters.chunks += 1
            counters.bytes_read += len(data)
            counters.chunk_arrivals.append(t_mono_ns)
            in_waiting = getattr(self.transport, "in_waiting", 0)
            counters.max_in_waiting = max(counters.max_in_waiting, in_waiting)

            buffer.append(self._tap_offset(), data)

            while len(buffer) >= express_mod.CAPSULE_LEN:
                if not express_mod.looks_like_capsule(buffer[0], buffer[1]):
                    if not self._resync_express(buffer, last_t):
                        break
                    continue

                frame, offset = buffer.consume(express_mod.CAPSULE_LEN)
                try:
                    capsule = express_mod.parse_capsule(frame)
                except ProtocolError:
                    counters.resyncs += 1
                    continue

                for sample in decoder.push(capsule):
                    counters.samples += 1
                    yield ScanSample(
                        sample=sample, t_mono_ns=last_t, raw_offset=offset
                    )

    def _resync_express(self, buffer: _OffsetBuffer, t_mono_ns: int) -> bool:
        """Re-establish capsule alignment. Returns False if more data needed."""
        confirm = 3
        window = confirm * express_mod.CAPSULE_LEN
        offset = express_mod.find_capsule_alignment(buffer.peek(window + 8))

        if offset is None:
            if len(buffer) < window + express_mod.CAPSULE_LEN:
                return False
            offset = 1

        if offset == 0:
            return True

        raw_offset = buffer.head_offset()
        buffer.consume(offset)
        self.counters.bytes_discarded += offset
        self.counters.resyncs += 1

        if self._on_resync:
            self._on_resync(
                ResyncEvent(
                    t_mono_ns=t_mono_ns,
                    bytes_discarded=offset,
                    raw_offset=raw_offset,
                    detail="express sync nibbles not found",
                )
            )
        return True

    def _resync(self, buffer: _OffsetBuffer, t_mono_ns: int) -> bool:
        """Re-establish byte alignment. Returns False if more data is needed."""
        window = protocol.RESYNC_CONFIRM_NODES * protocol.LEGACY_NODE_LEN
        offset = protocol.find_node_alignment(buffer.peek(window + 8))

        if offset is None:
            # Not enough buffered to confirm any alignment yet. Only start
            # throwing bytes away once there is genuinely enough to decide.
            if len(buffer) < window + protocol.LEGACY_NODE_LEN:
                return False
            offset = 1

        if offset == 0:
            return True

        raw_offset = buffer.head_offset()
        buffer.consume(offset)
        self.counters.bytes_discarded += offset
        self.counters.resyncs += 1

        if self._on_resync:
            self._on_resync(
                ResyncEvent(
                    t_mono_ns=t_mono_ns,
                    bytes_discarded=offset,
                    raw_offset=raw_offset,
                    detail="node redundancy check failed",
                )
            )
        return True

    # -- lifecycle ---------------------------------------------------------

    def shutdown(self) -> None:
        """Stop scanning and the motor. Safe to call more than once."""
        try:
            if self._scanning:
                self.stop()
        except Exception:
            pass  # Shutdown must not mask the original failure.
        try:
            self.set_motor(False)
        except Exception:
            pass

    def __enter__(self) -> Lidar:
        return self

    def __exit__(self, *exc: object) -> None:
        self.shutdown()


class MotorGuard:
    """Install signal handlers that stop the motor before the process dies.

    Without this, Ctrl-C or a SIGTERM leaves the motor spinning -- the DTR
    line stays low until something closes the port. The production C++ driver
    needs the equivalent in both its destructor and its signal handling.
    """

    def __init__(self, lidar: Lidar) -> None:
        self._lidar = lidar
        self._previous: dict[int, object] = {}

    def __enter__(self) -> MotorGuard:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            try:
                self._previous[sig] = signal.getsignal(sig)
                signal.signal(sig, self._handle)
            except (ValueError, OSError):
                pass  # Not on the main thread, or the signal does not exist.
        return self

    def _handle(self, signum: int, frame: object) -> None:
        self._lidar.shutdown()
        previous = self._previous.get(signum)
        if callable(previous):
            previous(signum, frame)
        elif signum == signal.SIGINT:
            raise KeyboardInterrupt
        else:
            raise SystemExit(128 + signum)

    def __exit__(self, *exc: object) -> None:
        for sig, handler in self._previous.items():
            try:
                signal.signal(sig, handler)  # type: ignore[arg-type]
            except (ValueError, OSError):
                pass
        self._lidar.shutdown()
