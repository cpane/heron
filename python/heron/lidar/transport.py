"""Byte transports: the real serial port, and replay from a capture file.

Every probe takes its bytes through this interface, so ``--replay`` gives all
of them offline operation against a recorded ``.rpraw`` with no hardware and
no spinning motor.

pyserial is imported lazily inside :class:`SerialTransport` so that the replay
path and the host-side analysis tools work on a machine where pyserial is not
installed.
"""

from __future__ import annotations

import time
from typing import Protocol

from .recording import RawReader, RawWriter

#: The RPLIDAR A1 talks 115200 8N1. (The A2/A3 use 256000.)
DEFAULT_BAUD = 115200

#: Preferred device path. Created by target/udev/99-heron-rplidar.rules.
#: /dev/ttyUSB0 is assigned in enumeration order and is not stable once a
#: second USB serial device is attached.
DEFAULT_PORT = "/dev/rplidar"

#: Fallback used when the udev rule has not been installed yet.
FALLBACK_PORT = "/dev/ttyUSB0"


class Transport(Protocol):
    def write(self, data: bytes) -> None: ...

    def read_chunk(self, max_bytes: int, timeout: float) -> tuple[int, bytes]: ...

    def close(self) -> None: ...


class SerialTransport:
    """The physical port.

    Motor control is the thing to understand here. On the A1's USB adapter the
    motor is driven by the serial **DTR line**, not by a command:

        DTR asserted     -> motor stopped
        DTR de-asserted  -> motor running

    Two consequences that bite in production:

      * Opening the port asserts DTR, so the motor is stopped on open and must
        be explicitly released to spin.
      * The motor spins whenever DTR is low regardless of whether a scan is
        running, so a crashed process leaves it spinning. Hence the context
        manager, and the signal handling in :mod:`heron.lidar.driver`.

    There is no PWM speed control on the A1; ``SET_MOTOR_PWM`` (0xF0) is an
    A2/A3 feature. Rotation rate is whatever the supply and the motor give you.
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        motor: bool = False,
    ) -> None:
        import serial  # Imported here so replay works without pyserial.

        self.port = port
        self._serial = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0,  # Non-blocking; read_chunk implements its own timeout.
            dsrdtr=False,
            rtscts=False,
            xonxoff=False,
        )
        self.set_motor(motor)

    @property
    def dtr(self) -> bool:
        return bool(self._serial.dtr)

    @property
    def motor_running(self) -> bool:
        return not self.dtr

    def set_motor(self, running: bool) -> None:
        """Start or stop the motor. Inverted, because DTR high stops it."""
        self._serial.dtr = not running

    @property
    def in_waiting(self) -> int:
        """Bytes buffered by the kernel.

        Worth watching: the tty buffer is only a few KB and at 7.5 KB/s that
        is well under a second of slack. Overrun is silent -- no error, just
        missing bytes and a discontinuous angle.
        """
        return self._serial.in_waiting

    def flush_input(self) -> None:
        self._serial.reset_input_buffer()

    def write(self, data: bytes) -> None:
        self._serial.write(data)
        self._serial.flush()

    def read_chunk(self, max_bytes: int, timeout: float) -> tuple[int, bytes]:
        """Read whatever is available, up to ``max_bytes``.

        Returns ``(t_mono_ns, data)`` with the timestamp taken immediately
        after ``read()`` returns, so it is the arrival time of the *chunk*.
        Returns empty bytes on timeout rather than raising; a scan that goes
        quiet is a finding, not an exception.
        """
        deadline = time.monotonic() + timeout

        while True:
            data = self._serial.read(max_bytes)
            now = time.monotonic_ns()
            if data:
                return now, data
            if time.monotonic() >= deadline:
                return now, b""
            time.sleep(0.0005)

    def close(self) -> None:
        if self._serial.is_open:
            # Stop the motor before dropping the port. The tty layer also
            # drops DTR on close (HUPCL), which would start it spinning.
            self.set_motor(False)
            self._serial.close()

    def __enter__(self) -> SerialTransport:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class ReplayTransport:
    """Replay the sensor->host side of a ``.rpraw`` capture.

    Writes are accepted and discarded: the recorded responses are already
    fixed, so commands cannot change them. That is a feature -- it makes the
    replay deterministic and it is how a parser gets developed on a laptop.
    """

    def __init__(self, path: str, speed: float = 0.0) -> None:
        """``speed`` of 0 replays as fast as possible; 1.0 is real time."""
        self._reader = RawReader(path)
        self.header = self._reader.header
        self._chunks = (c for c in self._reader.chunks() if c.direction == "rx")
        self._speed = speed
        self._pending = b""
        self._last_t: int | None = None
        self.tx_log: list[bytes] = []
        self.exhausted = False

    def write(self, data: bytes) -> None:
        self.tx_log.append(data)

    def read_chunk(self, max_bytes: int, timeout: float) -> tuple[int, bytes]:
        if not self._pending:
            try:
                chunk = next(self._chunks)
            except StopIteration:
                self.exhausted = True
                return time.monotonic_ns(), b""

            if self._speed > 0 and self._last_t is not None:
                delay = (chunk.t_mono_ns - self._last_t) / 1e9 / self._speed
                if delay > 0:
                    time.sleep(min(delay, timeout))
            self._last_t = chunk.t_mono_ns
            self._pending = chunk.data

        data, self._pending = self._pending[:max_bytes], self._pending[max_bytes:]
        return time.monotonic_ns(), data

    # The motor and buffer interface exist so probes need no special-casing.
    @property
    def in_waiting(self) -> int:
        return len(self._pending)

    @property
    def motor_running(self) -> bool:
        return False

    def set_motor(self, running: bool) -> None:
        pass

    def flush_input(self) -> None:
        self._pending = b""

    def close(self) -> None:
        self._reader.close()

    def __enter__(self) -> ReplayTransport:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


class TappedTransport:
    """Wrap a transport and tee every chunk, both directions, into a file.

    Delegates everything it does not intercept, so a probe cannot tell the
    difference between a tapped and an untapped transport.
    """

    def __init__(self, inner: Transport, writer: RawWriter) -> None:
        self._inner = inner
        self._writer = writer
        self.last_rx_offset = -1
        self.last_tx_offset = -1

    def write(self, data: bytes) -> None:
        self.last_tx_offset = self._writer.write_chunk(
            "tx", time.monotonic_ns(), data
        )
        self._inner.write(data)

    def read_chunk(self, max_bytes: int, timeout: float) -> tuple[int, bytes]:
        t_mono_ns, data = self._inner.read_chunk(max_bytes, timeout)
        if data:
            self.last_rx_offset = self._writer.write_chunk("rx", t_mono_ns, data)
        return t_mono_ns, data

    def __getattr__(self, name: str) -> object:
        return getattr(self._inner, name)

    def close(self) -> None:
        self._inner.close()
        self._writer.close()

    def __enter__(self) -> TappedTransport:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
