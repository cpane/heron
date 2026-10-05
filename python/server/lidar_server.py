"""LiDAR server -- runs on the Pi, owns the sensor, serves one GUI client.

    scripts/deploy_pi.sh cpane@<pi> server/lidar_server.py

Design notes, in the order they will bite you if ignored:

* **One client at a time.** There is a single physical sensor and a single
  serial port; two clients would mean two owners of one motor. A second
  connection is accepted only far enough to be told why it is being closed,
  which beats leaving it to hang unexplained in the listen backlog.

* **The device is touched by exactly one thread.** All serial traffic happens
  on the worker thread. The socket thread only ever posts intentions onto a
  queue. Sharing a serial port across threads produces interleaved requests
  and responses that look exactly like protocol corruption.

* **The motor is turned off on every exit path.** Destructor, signal handler,
  client disconnect, and an unhandled exception in the worker. An orphaned
  process leaves the head spinning, which on this hardware used to mean an
  under-voltage board. The supply is fixed, but a spinning motor nobody owns
  is still a bug.

* **Undecodable modes still run.** Three of this unit's five scan modes
  stream ultra capsules (0x84), which the Python library cannot decode. They
  start, they stream, and the server reports byte and frame statistics for
  them. It does not fabricate points. See docs/rplidar-a1m8-findings.md.
"""

from __future__ import annotations

import argparse
import errno
import os
import queue
import signal
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from heron.lidar import platform_info, protocol  # noqa: E402
from heron.lidar.driver import Lidar, LidarError, ScanSample  # noqa: E402
from heron.lidar.scan import RevolutionAssembler  # noqa: E402
from heron.lidar.transport import (  # noqa: E402
    DEFAULT_BAUD,
    DEFAULT_PORT,
    SerialTransport,
)
from heron.link import wire  # noqa: E402

#: Data types this server can turn into points.
DECODABLE_ANS_TYPES = {
    protocol.DATA_TYPE_LEGACY_SCAN: "legacy",
    protocol.DATA_TYPE_EXPRESS_SCAN: "express",
}

#: How often the raw-stream statistics message is emitted, in seconds. Slow
#: enough not to flood the link, fast enough to look live.
RAW_REPORT_INTERVAL_S = 0.5


class Service:
    """Owns the sensor. Every method here runs on the worker thread."""

    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.lidar: Lidar | None = None
        self.motor_on = False
        self.scanning = False
        self.mode: int | None = None
        self._modes: list[dict[str, object]] = []
        self._typical = 0
        #: Mode of a scan that a motor-off interrupted, so motor-on can put it
        #: back. Without this the toggle is asymmetric -- off stops the
        #: stream, on does nothing -- and the viewer sits on a dead plot.
        self.resume_mode: int | None = None
        #: The mode the user last asked to scan in, which deliberately
        #: OUTLIVES stop_stream(). It has to: a queued command interrupts the
        #: stream, whose own teardown clears `scanning` and `mode` before the
        #: command is applied, so a handler that consulted those would always
        #: see an idle device and conclude nothing was running.
        #: Cleared only by an explicit stop or a reset.
        self.active_mode: int | None = None

    # -- lifecycle ---------------------------------------------------------

    def open(self) -> None:
        transport = SerialTransport(port=self.port, baud=self.baud, motor=False)
        self.lidar = Lidar(transport)
        # A device left mid-stream by a previous run answers GET_INFO with
        # scan data. Stopping first makes the first query deterministic.
        self.lidar.stop()
        time.sleep(0.05)

    def close(self) -> None:
        """Best effort, and deliberately noisy about failure.

        Called from the signal handler and from normal shutdown, so it must
        tolerate being called twice and being called half-initialised.
        """
        if self.lidar is None:
            return
        try:
            self.lidar.stop()
        except Exception as exc:  # noqa: BLE001 - shutdown must not raise
            print(f"  WARNING: stop failed during shutdown: {exc}")
        try:
            self.lidar.set_motor(False)
        except Exception as exc:  # noqa: BLE001
            print(f"  WARNING: motor off failed during shutdown: {exc}")
        try:
            self.lidar.shutdown()
        except Exception as exc:  # noqa: BLE001
            print(f"  WARNING: transport close failed: {exc}")
        self.lidar = None
        self.motor_on = False
        self.scanning = False

    # -- queries -----------------------------------------------------------

    def describe(self) -> dict[str, object]:
        """The hello payload: what this device is and what it can do."""
        assert self.lidar is not None
        _, _, info = self.lidar.get_info()
        _, _, health = self.lidar.get_health()

        self._modes = self.lidar.enumerate_scan_modes()
        self._typical = self.lidar.typical_scan_mode()

        modes = []
        decodable = []
        for entry in self._modes:
            ans = int(entry["ans_type"])  # type: ignore[arg-type]
            kind = DECODABLE_ANS_TYPES.get(ans)
            if kind is not None:
                decodable.append(int(entry["id"]))  # type: ignore[arg-type]
            modes.append(
                {
                    "id": entry["id"],
                    "name": entry["name"],
                    "us_per_sample": entry["us_per_sample"],
                    "max_distance_m": entry["max_distance_m"],
                    "ans_type": ans,
                    "kind": kind or f"undecodable (0x{ans:02x})",
                }
            )

        return wire.hello(
            device={
                "model": info.model,
                "firmware": info.firmware,
                "hardware": info.hardware,
                "serial": info.serial_text,
            },
            health={"status": health.status, "state": health.state_name},
            modes=modes,
            decodable=decodable,
            typical_mode=self._typical,
            port=self.port,
        )

    def status(self, detail: str = "") -> dict[str, object]:
        return wire.status(
            motor=self.motor_on,
            scanning=self.scanning,
            mode=self.mode,
            detail=detail,
        )

    # -- control -----------------------------------------------------------

    def set_motor(self, on: bool) -> None:
        assert self.lidar is not None
        self.lidar.set_motor(on)
        self.motor_on = on

    def spin_up(self) -> None:
        """Motor on, then wait for the head to reach a stable speed.

        Always go through here rather than set_motor(True) before starting a
        stream. Reading during the spin-up transient yields a revolution's
        worth of nonsense -- it is why the probes discard the first few.
        """
        assert self.lidar is not None
        self.lidar.spin_up()
        self.motor_on = True

    def reset(self) -> None:
        assert self.lidar is not None
        self.lidar.reset()
        self.motor_on = False
        self.scanning = False
        self.mode = None
        self.active_mode = None
        self.resume_mode = None

    def ans_type_for(self, mode: int) -> int:
        for entry in self._modes:
            if int(entry["id"]) == mode:  # type: ignore[arg-type]
                return int(entry["ans_type"])  # type: ignore[arg-type]
        raise LidarError(f"unknown scan mode {mode}")

    def start_stream(self, mode: int) -> tuple[str, int]:
        """Begin scanning. Returns ``(kind, frame_len)``.

        ``kind`` is 'legacy', 'express' or 'raw'. It comes from the descriptor
        the device actually returned, not from the mode table, because the
        two are not known to agree on this unit -- see
        ``express_scan_payload``.
        """
        assert self.lidar is not None

        if not self.motor_on:
            self.spin_up()

        advertised = self.ans_type_for(mode)

        if advertised == protocol.DATA_TYPE_LEGACY_SCAN:
            descriptor = self.lidar.start_scan()
        else:
            descriptor = self.lidar.start_raw_express(mode)

        actual = descriptor.data_type
        kind = DECODABLE_ANS_TYPES.get(actual)
        if kind is None:
            kind = "raw"

        if actual != advertised:
            print(
                f"  NOTE: mode {mode} advertised 0x{advertised:02x} but the "
                f"descriptor says 0x{actual:02x}; trusting the descriptor."
            )

        self.scanning = True
        self.mode = mode
        self.active_mode = mode
        return kind, descriptor.length

    def stop_stream(self) -> None:
        assert self.lidar is not None
        self.lidar.stop()
        self.scanning = False
        self.mode = None


class Worker(threading.Thread):
    """Serialises every sensor access, and streams results to the client."""

    def __init__(self, service: Service, send, commands: queue.Queue) -> None:
        super().__init__(name="lidar-worker", daemon=True)
        self.service = service
        self.send = send
        self.commands = commands
        self.stop_event = threading.Event()

    # -- command handling --------------------------------------------------

    def _drain_commands(self, blocking: bool) -> None:
        """Apply queued commands. Never lets one bad command kill the thread."""
        while True:
            try:
                cmd = self.commands.get(timeout=0.25) if blocking else self.commands.get_nowait()
            except queue.Empty:
                return
            try:
                self._apply(cmd)
            except LidarError as exc:
                self.send(wire.error(str(exc)))
            except Exception as exc:  # noqa: BLE001
                self.send(wire.error(f"{type(exc).__name__}: {exc}"))
            blocking = False

    def _apply(self, cmd: dict) -> None:
        name = cmd.get("cmd")

        if name == wire.CMD_HELLO or name == wire.CMD_REFRESH:
            self.send(self.service.describe())
            self.send(self.service.status())

        elif name == wire.CMD_MOTOR:
            on = bool(cmd.get("on"))

            if not on:
                # Use active_mode, not scanning/mode: this command interrupted
                # the stream, and the stream's own teardown has already run and
                # cleared those. active_mode is what survives.
                if self.service.active_mode is not None:
                    self.service.resume_mode = self.service.active_mode
                if self.service.scanning:
                    self.service.stop_stream()
                self.service.set_motor(False)
                detail = "motor off"
                if self.service.resume_mode is not None:
                    detail += f"; scan mode {self.service.resume_mode} will resume"
                self.send(self.service.status(detail))
                return

            resume = self.service.resume_mode
            self.service.resume_mode = None
            self.service.spin_up()

            if resume is None:
                self.send(self.service.status("motor on"))
                return

            kind, frame_len = self.service.start_stream(resume)
            self.send(
                self.service.status(
                    f"motor on; resumed mode {resume} ({kind}, {frame_len}B frames)"
                )
            )
            self._stream(kind, frame_len)

        elif name == wire.CMD_SCAN_START:
            mode = int(cmd.get("mode", 0))
            if self.service.scanning:
                self.service.stop_stream()
            kind, frame_len = self.service.start_stream(mode)
            self.send(
                self.service.status(f"scanning mode {mode} ({kind}, {frame_len}B frames)")
            )
            self._stream(kind, frame_len)

        elif name == wire.CMD_SCAN_STOP:
            # An explicit stop is a decision, so drop any pending resume.
            # Otherwise a later motor-on would restart a scan nobody asked for.
            self.service.resume_mode = None
            self.service.active_mode = None
            if self.service.scanning:
                self.service.stop_stream()
            self.send(self.service.status("idle"))

        elif name == wire.CMD_RESET:
            if self.service.scanning:
                self.service.stop_stream()
            self.service.resume_mode = None
            self.service.active_mode = None
            self.service.reset()
            self.send(self.service.status("device reset"))
            self.send(self.service.describe())

        else:
            self.send(wire.error(f"unknown command {name!r}"))

    # -- streaming ---------------------------------------------------------

    def _stream(self, kind: str, frame_len: int) -> None:
        """Pump the sensor until told to stop.

        Commands are polled between revolutions, so a Stop lands within about
        one revolution -- 137 ms -- rather than whenever the generator decides
        to yield.
        """
        try:
            if kind == "raw":
                self._stream_raw(frame_len)
            else:
                self._stream_points(kind)
        except LidarError as exc:
            self.send(wire.error(f"stream ended: {exc}"))
        finally:
            if self.service.scanning:
                try:
                    self.service.stop_stream()
                except Exception:  # noqa: BLE001
                    pass
            self.send(self.service.status("idle"))

    def _should_break(self) -> bool:
        """True when a command is waiting or the server is shutting down."""
        return self.stop_event.is_set() or not self.commands.empty()

    def _stream_points(self, kind: str) -> None:
        assert self.service.lidar is not None
        lidar = self.service.lidar
        mode = self.service.mode or 0
        assembler = RevolutionAssembler()
        # stop= matters when the device goes silent: the checks in the loop
        # below only run when something is yielded, and silence yields nothing.
        if kind == "express":
            source = lidar.express_samples(stop=self._should_break)
        else:
            source = lidar.samples(stop=self._should_break)
        seq = 0

        for scan_sample in source:
            revolution = assembler.push(scan_sample)
            if revolution is None or revolution.index < 0:
                if self._should_break():
                    return
                continue

            ordered = revolution.sorted_samples()
            points: list[tuple[int, int, int]] = []
            for item in ordered:
                s = item.sample
                if s.valid:
                    points.append((s.angle_q6, s.dist_q2, s.quality))

            # Over every sample, not just the points sent: a no-return still
            # proves the beam looked there. Measured on valid points this
            # read 30-115 degrees on perfectly healthy revolutions.
            gap_q6 = revolution.sampling_gap_q6()

            self.send(
                wire.scan(
                    seq=seq,
                    mode=mode,
                    t_start_ns=revolution.t_start_ns,
                    t_end_ns=revolution.t_end_ns,
                    points=points,
                    total_samples=revolution.count,
                    max_gap_q6=gap_q6,
                )
            )
            seq += 1

            if self._should_break():
                return

    def _stream_raw(self, frame_len: int) -> None:
        """Byte and frame statistics for a format this repo cannot decode.

        Frame count is ``bytes / frame_len`` -- an estimate, not a parse. The
        sync tally counts positions where the 0xA / 0x5 high nibbles that
        both capsule formats use actually appear, which is evidence the
        framing is holding without claiming to decode the payload.
        """
        assert self.service.lidar is not None
        mode = self.service.mode or 0
        ans = self.service.ans_type_for(mode)
        started = time.monotonic()
        last_report = started
        total_bytes = 0
        sync_hits = 0
        pending = bytearray()

        note = (
            f"0x{ans:02x} frames are not decoded by this build; "
            "showing stream statistics only"
        )

        # See _stream_points: without stop=, a silent stream cannot be stopped.
        for _t_ns, data in self.service.lidar.raw_chunks(stop=self._should_break):
            total_bytes += len(data)
            pending.extend(data)

            while len(pending) >= frame_len:
                frame = pending[:frame_len]
                del pending[:frame_len]
                if (frame[0] >> 4) == 0xA and (frame[1] >> 4) == 0x5:
                    sync_hits += 1

            now = time.monotonic()
            if now - last_report >= RAW_REPORT_INTERVAL_S:
                self.send(
                    wire.raw(
                        mode=mode,
                        ans_type=ans,
                        frames=sync_hits,
                        frame_bytes=frame_len,
                        bytes_read=total_bytes,
                        elapsed_s=now - started,
                        note=note,
                    )
                )
                last_report = now

            if self._should_break():
                return

    # -- thread body -------------------------------------------------------

    def run(self) -> None:
        while not self.stop_event.is_set():
            self._drain_commands(blocking=True)


class Server:
    def __init__(self, host: str, tcp_port: int, service: Service) -> None:
        self.host = host
        self.tcp_port = tcp_port
        self.service = service
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._shutdown = threading.Event()
        #: Held for the lifetime of a connected client. Non-blocking acquire,
        #: so a second GUI gets told why rather than hanging in the backlog.
        self._busy = threading.Lock()

    def serve_forever(self) -> None:
        self.sock.bind((self.host, self.tcp_port))
        self.sock.listen(2)
        print(f"  listening on {self.host}:{self.tcp_port}")
        print("  Ctrl-C to stop\n")

        while not self._shutdown.is_set():
            try:
                conn, addr = self.sock.accept()
            except OSError as exc:
                if self._shutdown.is_set() or exc.errno == errno.EBADF:
                    return
                raise

            if not self._busy.acquire(blocking=False):
                # One sensor, one motor, one owner. Say so and hang up rather
                # than leaving the second client waiting on a silent socket.
                try:
                    conn.sendall(
                        wire.encode(
                            wire.error(
                                "another client already owns this sensor",
                                fatal=True,
                            )
                        )
                    )
                    conn.close()
                except OSError:
                    pass
                print(f"  refused {addr[0]}:{addr[1]} -- already serving a client")
                continue

            threading.Thread(
                target=self._client_thread, args=(conn, addr), daemon=True
            ).start()

    def _client_thread(self, conn: socket.socket, addr) -> None:
        try:
            self._handle_client(conn, addr)
        finally:
            self._busy.release()

    def stop(self) -> None:
        self._shutdown.set()
        try:
            self.sock.close()
        except OSError:
            pass

    def _handle_client(self, conn: socket.socket, addr) -> None:
        print(f"  client connected from {addr[0]}:{addr[1]}")
        conn.settimeout(None)
        send_lock = threading.Lock()
        commands: queue.Queue = queue.Queue()

        def send(message: dict) -> None:
            """Serialised because the worker and the socket thread both send."""
            with send_lock:
                try:
                    conn.sendall(wire.encode(message))
                except OSError:
                    pass  # The reader loop below notices and tears down.

        worker = Worker(self.service, send, commands)
        worker.start()
        reader = wire.LineReader()

        try:
            send(self.service.describe())
            send(self.service.status("connected"))

            while True:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                for message in reader.feed(chunk):
                    name = message.get("cmd")
                    if name not in wire.CLIENT_COMMANDS:
                        send(wire.error(f"unknown command {name!r}"))
                        continue
                    # A running stream polls commands.empty() between
                    # revolutions, so simply enqueueing is enough to interrupt
                    # it within about one revolution (~137 ms).
                    commands.put(message)
        except (OSError, wire.WireError) as exc:
            print(f"  client error: {exc}")
        finally:
            print("  client disconnected")
            worker.stop_event.set()
            commands.put({"cmd": wire.CMD_SCAN_STOP})
            worker.join(timeout=5.0)
            try:
                conn.close()
            except OSError:
                pass

            if worker.is_alive():
                # The worker still owns the serial port. Idling the sensor
                # from this thread would mean two threads issuing requests on
                # one port, which reads as protocol corruption rather than as
                # the race it is. Leave it alone and say so.
                print(
                    "  WARNING: worker did not exit; leaving the sensor alone "
                    "rather than racing it on the serial port. The motor may "
                    "still be spinning -- restart the server to clear it."
                )
            else:
                # Leaving the motor spinning for the next client would be a
                # bug, not a convenience.
                try:
                    if self.service.scanning:
                        self.service.stop_stream()
                    self.service.set_motor(False)
                except Exception as exc:  # noqa: BLE001
                    print(f"  WARNING: could not idle the sensor: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial device")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--listen", default="0.0.0.0", help="bind address")
    parser.add_argument("--tcp-port", type=int, default=wire.DEFAULT_TCP_PORT)
    args = parser.parse_args()

    print("=" * 40)
    print(" Heron LiDAR server")
    print("=" * 40)
    for line in platform_info.summary_lines():
        print(line)
    print()

    serial_port, note = platform_info.resolve_port(args.port)
    if note:
        print(f"  NOTE: {note}")

    service = Service(serial_port, args.baud)
    try:
        service.open()
    except ImportError:
        print(
            "ERROR: pyserial is not installed.\n"
            "  sudo apt-get install python3-serial\n"
            "  or re-run scripts/provision_pi.sh <user@host>"
        )
        return 1
    except OSError as exc:
        print(
            f"ERROR: cannot open {serial_port}: {exc}\n"
            "  Check the device is attached, that /dev/rplidar exists, and\n"
            "  that you are in the 'dialout' group."
        )
        return 1

    server = Server(args.listen, args.tcp_port, service)

    def shutdown(signum, _frame) -> None:
        print(f"\n  signal {signum}: shutting down")
        server.stop()
        service.close()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        server.serve_forever()
    finally:
        server.stop()
        service.close()
        print("  sensor idled, motor off")
    return 0


if __name__ == "__main__":
    sys.exit(main())
