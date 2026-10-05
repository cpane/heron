"""The GUI <-> server wire format: one JSON object per line, over TCP.

Why newline-delimited JSON rather than something binary or a framework:

* It adds no dependency on either end. The Pi's interpreter is
  EXTERNALLY-MANAGED (PEP 668), so anything needing pip is friction, and the
  host side is meant to stay runnable from a bare checkout.
* It is inspectable. ``nc <pi> 5555`` and typing ``{"cmd":"hello"}``
  is a complete diagnostic client, which matters for a tool whose entire
  purpose is learning what the sensor does.
* Volume is not a problem at this scale. A worst-case express revolution is
  ~530 points; at ~7.3 Hz that is well under 100 KB/s on a LAN.

This module is deliberately I/O-free apart from :class:`LineReader`, which
only ever consumes bytes handed to it. Sockets live in the server and the GUI.

Units on the wire
-----------------
Angles and distances travel in the sensor's own fixed point, not in degrees
and millimetres:

* ``angle_q6``  -- 1/64 degree, exactly as the device sends it
* ``dist_q2``   -- 1/4 millimetre, likewise

That is lossless, keeps the JSON compact, and means the numbers on the wire
are the numbers in the protocol spec. Consumers divide. ``deg()`` and
``mm()`` below are the only conversions anybody should need.

Angle convention
----------------
Bearings are passed through untouched, in the sensor's own frame: **0 degrees
points out the front of the housing and the angle increases clockwise viewed
from above** (measured 2026-09-26, see docs/rplidar-a1m8-findings.md). No
robot-frame transform is applied here. A consumer that wants the REP 103
convention must negate, because the sensor frame is left-handed.

Robot-frame extension (C++ server)
----------------------------------
cpp/tools/LidarGuiServer.cpp serves the C++ adapter, whose scans are already
in the robot frame. Its scan messages add optional fields; a client that does
not know them ignores them, and the Python server never sends them, so
PROTOCOL_VERSION stays 1:

* ``frame``        ``"robot"``
* ``rpts``         ``[[bearing_cdeg, range_mm], ...]``: bearing in hundredths
  of a degree, REP 103 (0 forward, **positive to the left**), in
  [-18000, 18000); range in millimetres. Sorted by bearing. ``pts`` is sent
  empty.
* ``coverage_ok``  ``Scan::isCoverageOk()``: the whole revolution was observed
* ``unobserved``   ``[[start_cdeg, width_cdeg], ...]``: arcs with no samples,
  each extending anticlockwise from its start
* ``dropped``      estimated samples lost to overrun

Its hello adds ``"server": "cpp-adapter"``, and its ``typical_mode`` is the
adapter's default (Express), not the device's own (mode 3).
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Iterator

#: Bumped when a message changes shape in a way an old peer would misread.
#: The server refuses a client that does not match, because the failure mode
#: otherwise is a plot that looks plausible and is wrong.
PROTOCOL_VERSION = 1

DEFAULT_TCP_PORT = 5555

# -- client -> server ------------------------------------------------------

CMD_HELLO = "hello"
CMD_MOTOR = "motor"
CMD_SCAN_START = "scan_start"
CMD_SCAN_STOP = "scan_stop"
CMD_REFRESH = "refresh"
CMD_RESET = "reset"

CLIENT_COMMANDS = frozenset(
    {CMD_HELLO, CMD_MOTOR, CMD_SCAN_START, CMD_SCAN_STOP, CMD_REFRESH, CMD_RESET}
)

# -- server -> client ------------------------------------------------------

MSG_HELLO = "hello"
MSG_STATUS = "status"
MSG_SCAN = "scan"
MSG_RAW = "raw"
MSG_ERROR = "error"


class WireError(Exception):
    """A message was malformed, truncated, or of an unknown kind."""


def deg(angle_q6: int) -> float:
    """1/64 degree -> degrees."""
    return angle_q6 / 64.0


def mm(dist_q2: int) -> float:
    """1/4 millimetre -> millimetres."""
    return dist_q2 / 4.0


def encode(message: dict[str, Any]) -> bytes:
    """One message, one line.

    ``separators`` strips the whitespace json.dumps adds by default; on a
    530-point revolution at 7 Hz that padding is not free.
    """
    return json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n"


def decode(line: bytes) -> dict[str, Any]:
    try:
        obj = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise WireError(f"not valid JSON: {exc}") from exc
    if not isinstance(obj, dict):
        raise WireError(f"expected a JSON object, got {type(obj).__name__}")
    return obj


@dataclass
class LineReader:
    """Reassembles newline-delimited messages from arbitrary chunk boundaries.

    TCP gives no message framing, so a read can land mid-object or carry three
    objects at once. Feed it whatever ``recv`` returned and iterate what comes
    out.
    """

    #: Refuse to buffer more than this without seeing a newline. A peer that
    #: never terminates a line would otherwise grow this without limit.
    max_line: int = 8 * 1024 * 1024
    _buffer: bytearray = field(default_factory=bytearray, repr=False)

    def feed(self, chunk: bytes) -> Iterator[dict[str, Any]]:
        self._buffer.extend(chunk)
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) > self.max_line:
                    raise WireError(
                        f"no newline in {len(self._buffer)} bytes; "
                        "peer is not speaking this protocol"
                    )
                return
            line = bytes(self._buffer[:newline])
            del self._buffer[: newline + 1]
            if line.strip():
                yield decode(line)


# -- message constructors --------------------------------------------------
#
# These exist so the two ends cannot drift: a key renamed here is renamed for
# both, and a typo is a NameError at import rather than a field that silently
# reads as None on the far side.


def hello(
    *,
    device: dict[str, Any],
    health: dict[str, Any],
    modes: list[dict[str, Any]],
    decodable: list[int],
    typical_mode: int,
    port: str,
) -> dict[str, Any]:
    """Everything the GUI needs to populate its controls, sent once on connect.

    ``decodable`` is the subset of mode ids this server can turn into points.
    The others stream ultra capsules (0x84), which the Python library cannot
    decode -- see docs/rplidar-a1m8-findings.md. They are still startable,
    and still reported, just as raw stream statistics rather than geometry.
    """
    return {
        "type": MSG_HELLO,
        "version": PROTOCOL_VERSION,
        "device": device,
        "health": health,
        "modes": modes,
        "decodable": decodable,
        "typical_mode": typical_mode,
        "port": port,
    }


def status(
    *,
    motor: bool,
    scanning: bool,
    mode: int | None,
    detail: str = "",
) -> dict[str, Any]:
    return {
        "type": MSG_STATUS,
        "motor": motor,
        "scanning": scanning,
        "mode": mode,
        "detail": detail,
    }


def scan(
    *,
    seq: int,
    mode: int,
    t_start_ns: int,
    t_end_ns: int,
    points: list[tuple[int, int, int]],
    total_samples: int,
    max_gap_q6: int,
) -> dict[str, Any]:
    """One revolution.

    ``points`` carries only *valid* returns, as ``[angle_q6, dist_q2,
    quality]`` triples sorted by angle. Invalid returns are dropped because
    they are 61-84% of samples and are noise to a consumer -- but
    ``total_samples`` keeps the count, because "nothing at 90 degrees" and
    "did not look at 90 degrees" are different facts and the GUI should be
    able to tell them apart.

    ``max_gap_q6`` is the widest angular gap between consecutive *samples* --
    all of them, no-returns included, closing the 360 seam. It is the cheap
    drop detector: buffer overrun on this device is completely silent, so a
    sudden wide gap is the only evidence that samples vanished. It is NOT the
    gap between the points in ``pts``; that is routinely 30-115 degrees on a
    healthy revolution and says nothing about loss. Healthy is ~8-9 degrees.
    """
    return {
        "type": MSG_SCAN,
        "seq": seq,
        "mode": mode,
        "t_start_ns": t_start_ns,
        "t_end_ns": t_end_ns,
        "pts": points,
        "total": total_samples,
        "max_gap_q6": max_gap_q6,
    }


def raw(
    *,
    mode: int,
    ans_type: int,
    frames: int,
    frame_bytes: int,
    bytes_read: int,
    elapsed_s: float,
    note: str,
) -> dict[str, Any]:
    """Statistics for a mode whose payload this server cannot decode.

    Sent instead of :func:`scan` for the ultra-capsule modes, so the GUI can
    still show that the stream is alive and healthy without inventing points
    it cannot compute.
    """
    return {
        "type": MSG_RAW,
        "mode": mode,
        "ans_type": ans_type,
        "frames": frames,
        "frame_bytes": frame_bytes,
        "bytes_read": bytes_read,
        "elapsed_s": round(elapsed_s, 3),
        "note": note,
    }


def error(message: str, *, fatal: bool = False) -> dict[str, Any]:
    return {"type": MSG_ERROR, "message": message, "fatal": fatal}
