"""RPLIDAR A1M8 wire protocol: pure decoding, no I/O.

This module is the part of the evaluation program that is expected to be
re-implemented in C++ for production. It is written to be transliterated:

  * No I/O, no logging, no threads, no global state.
  * Only ``dataclasses``, ``enum`` and ``struct`` are imported.
  * Every bit-field extraction is an explicit shift/mask, commented with the
    field name used by the SLAMTEC interface protocol document.
  * Fixed-point values are kept as integers (``angle_q6``, ``dist_q2``).
    Conversion to floating point happens only in the convenience properties,
    which the C++ version is free not to have.

Verified against the physical device (firmware 1.29, hardware 7) during
bring-up; see the values quoted in the probe output.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# --------------------------------------------------------------------------
# Framing constants
# --------------------------------------------------------------------------

#: First byte of every host->sensor request.
SYNC_REQUEST = 0xA5

#: Second byte of every sensor->host response *descriptor*. A descriptor is
#: always the pair (0xA5, 0x5A) followed by five more bytes.
SYNC_RESPONSE = 0x5A

#: A response descriptor is exactly seven bytes.
DESCRIPTOR_LEN = 7

#: A legacy scan node is exactly five bytes.
LEGACY_NODE_LEN = 5

#: Bit 7 of the command byte means "this request carries a payload", and
#: therefore also a checksum. This is a property of the opcode, not of the
#: caller: SL_LIDAR_CMDFLAG_HAS_PAYLOAD in the vendor SDK's sl_lidar_protocol.h.
#: Every command >= 0x80 (EXPRESS_SCAN, GET/SET_LIDAR_CONF, SET_MOTOR_PWM...)
#: takes one; every command below it does not.
CMDFLAG_HAS_PAYLOAD = 0x80


class Command(IntEnum):
    """Second byte of a request frame."""

    STOP = 0x25
    RESET = 0x40
    SCAN = 0x20
    FORCE_SCAN = 0x21
    EXPRESS_SCAN = 0x82
    GET_INFO = 0x50
    GET_HEALTH = 0x52
    GET_SAMPLERATE = 0x59
    GET_LIDAR_CONF = 0x84


class ResponseMode(IntEnum):
    """Top two bits of the descriptor's length word."""

    SINGLE = 0  # One payload of the stated length, then the device goes quiet.
    MULTI = 1  # The stated length repeats forever until STOP.
    RESERVED_2 = 2
    RESERVED_3 = 3


class ConfType(IntEnum):
    """Query selectors for GET_LIDAR_CONF (firmware 1.24 and later).

    The request payload is this value as a little-endian uint32, optionally
    followed by a uint16 scan-mode id for the per-mode queries. The response
    echoes the uint32 back, followed by the answer.
    """

    SCAN_MODE_COUNT = 0x70  # -> uint16
    SCAN_MODE_US_PER_SAMPLE = 0x71  # per mode -> uint32, Q8 microseconds
    SCAN_MODE_MAX_DISTANCE = 0x74  # per mode -> uint32, Q8 metres
    SCAN_MODE_ANS_TYPE = 0x75  # per mode -> uint8, the scan data type
    SCAN_MODE_TYPICAL = 0x7C  # -> uint16, the recommended mode id
    SCAN_MODE_NAME = 0x7F  # per mode -> ASCII string


#: Conf queries that take a scan-mode id appended to the request.
PER_MODE_CONF = frozenset(
    {
        ConfType.SCAN_MODE_US_PER_SAMPLE,
        ConfType.SCAN_MODE_MAX_DISTANCE,
        ConfType.SCAN_MODE_ANS_TYPE,
        ConfType.SCAN_MODE_NAME,
    }
)


class HealthState(IntEnum):
    GOOD = 0
    WARNING = 1
    ERROR = 2  # Requires RESET before the device will scan again.


#: Data-type byte of the descriptor, for the responses this program uses.
#: These identify the payload that follows; they are not error codes.
DATA_TYPE_DEVICE_INFO = 0x04
DATA_TYPE_HEALTH = 0x06
DATA_TYPE_SAMPLERATE = 0x15
DATA_TYPE_LEGACY_SCAN = 0x81
DATA_TYPE_EXPRESS_SCAN = 0x82
DATA_TYPE_LIDAR_CONF = 0x20


class ProtocolError(Exception):
    """A byte sequence did not match what the protocol specifies."""


class DescriptorError(ProtocolError):
    """A response descriptor was malformed or unexpected."""


# --------------------------------------------------------------------------
# Requests
# --------------------------------------------------------------------------


def build_request(command: int, payload: bytes | None = None) -> bytes:
    """Build a host->sensor request frame.

    Two shapes exist, and conflating them is a classic bring-up bug:

        no payload:   A5 <cmd>
        with payload: A5 <cmd> <payload_len> <payload...> <checksum>

    The checksum is the XOR of *every preceding byte in the frame*, including
    the sync byte, the command and the length. It is present **only** when the
    command's bit 7 is set (:data:`CMDFLAG_HAS_PAYLOAD`), which is how the
    vendor SDK decides too. ``A5 20`` (SCAN) carries no checksum;
    ``A5 82 05 00 00 00 00 00 <xor>`` (EXPRESS_SCAN) does. Appending a
    checksum to a payload-less command makes the device ignore the request;
    omitting it from EXPRESS_SCAN makes express mode silently never start.
    """
    expects_payload = bool(command & CMDFLAG_HAS_PAYLOAD)

    if expects_payload != (payload is not None):
        raise ProtocolError(
            f"command 0x{command:02x} "
            + ("requires" if expects_payload else "takes no")
            + f" payload (bit 7 = {int(expects_payload)}), "
            + f"but payload={'None' if payload is None else len(payload)}"
        )

    if payload is None:
        return bytes((SYNC_REQUEST, command))

    if len(payload) > 0xFF:
        raise ProtocolError(f"payload too long: {len(payload)} bytes")

    frame = bytearray((SYNC_REQUEST, command, len(payload)))
    frame += payload

    checksum = 0
    for byte in frame:
        checksum ^= byte
    frame.append(checksum)

    return bytes(frame)


def build_conf_payload(conf_type: int, mode: int | None = None) -> bytes:
    """Build the GET_LIDAR_CONF request payload.

    Little-endian uint32 selector, then an optional uint16 scan-mode id. The
    mode id is required for the per-mode queries in :data:`PER_MODE_CONF` and
    must be absent otherwise.
    """
    if mode is None:
        return struct.pack("<I", conf_type)
    return struct.pack("<IH", conf_type, mode)


def decode_conf_response(payload: bytes, expected_type: int) -> bytes:
    """Strip and validate the echoed selector, returning the answer bytes."""
    if len(payload) < 4:
        raise ProtocolError(
            f"GET_LIDAR_CONF reply too short: {len(payload)} bytes"
        )

    (echoed,) = struct.unpack_from("<I", payload, 0)
    if echoed != expected_type:
        raise ProtocolError(
            f"GET_LIDAR_CONF echoed type 0x{echoed:02x}, "
            f"expected 0x{expected_type:02x}"
        )
    return bytes(payload[4:])


# --------------------------------------------------------------------------
# Response descriptor
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Descriptor:
    """The seven-byte header that precedes every sensor->host response."""

    length: int  # Payload length in bytes (per response, for MULTI mode).
    mode: ResponseMode
    data_type: int
    raw: bytes

    @property
    def is_stream(self) -> bool:
        return self.mode is ResponseMode.MULTI


def parse_descriptor(data: bytes) -> Descriptor:
    """Decode a seven-byte response descriptor.

    Layout::

        byte 0    0xA5
        byte 1    0x5A
        bytes 2-5 little-endian uint32: low 30 bits = length, top 2 = mode
        byte 6    data type

    The 30/2 split is the trap. Read bytes 2-5 as a plain uint32 length and
    SCAN's ``length=5, mode=1`` silently becomes ``0x40000005`` -- a value
    large enough to look like a plausible buffer size, so the mistake surfaces
    much later as a hang rather than as a parse error.
    """
    if len(data) != DESCRIPTOR_LEN:
        raise DescriptorError(
            f"descriptor must be {DESCRIPTOR_LEN} bytes, got {len(data)}"
        )
    if data[0] != SYNC_REQUEST or data[1] != SYNC_RESPONSE:
        raise DescriptorError(
            f"bad descriptor sync: expected a55a, got {data[:2].hex()}"
        )

    (word,) = struct.unpack_from("<I", data, 2)

    length = word & 0x3FFFFFFF  # bits 0-29
    mode = (word >> 30) & 0x03  # bits 30-31

    return Descriptor(
        length=length,
        mode=ResponseMode(mode),
        data_type=data[6],
        raw=bytes(data),
    )


# --------------------------------------------------------------------------
# Single-response payloads
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class DeviceInfo:
    model: int
    firmware_minor: int
    firmware_major: int
    hardware: int
    serial: bytes  # 16 raw bytes, in the order they arrived on the wire.

    @property
    def firmware(self) -> str:
        return f"{self.firmware_major}.{self.firmware_minor:02d}"

    @property
    def serial_text(self) -> str:
        """Serial as printed by SLAMTEC's own tools: reversed, uppercase hex.

        The bytes arrive least-significant first. Both orders are printed by
        probe 1 so the one matching the device's label can be confirmed by eye
        rather than assumed.
        """
        return self.serial[::-1].hex().upper()


def decode_device_info(payload: bytes) -> DeviceInfo:
    """Decode the 20-byte GET_INFO payload."""
    if len(payload) != 20:
        raise ProtocolError(f"GET_INFO payload must be 20 bytes, got {len(payload)}")

    return DeviceInfo(
        model=payload[0],
        firmware_minor=payload[1],
        firmware_major=payload[2],
        hardware=payload[3],
        serial=bytes(payload[4:20]),
    )


@dataclass(frozen=True)
class Health:
    status: int
    error_code: int

    @property
    def state(self) -> HealthState | None:
        try:
            return HealthState(self.status)
        except ValueError:
            return None

    @property
    def state_name(self) -> str:
        state = self.state
        return state.name.title() if state is not None else f"Unknown({self.status})"

    @property
    def needs_reset(self) -> bool:
        return self.status == HealthState.ERROR


def decode_health(payload: bytes) -> Health:
    """Decode the 3-byte GET_HEALTH payload."""
    if len(payload) != 3:
        raise ProtocolError(f"GET_HEALTH payload must be 3 bytes, got {len(payload)}")

    return Health(
        status=payload[0],
        error_code=payload[1] | (payload[2] << 8),  # little-endian uint16
    )


@dataclass(frozen=True)
class SampleRate:
    """Microseconds per sample, as *claimed* by the device.

    On the unit under evaluation this reports 508 us standard (about 1970
    samples/s), while the measured sustained rate is nearer 1509 samples/s.
    Explaining that gap is one of the objectives of this program, so the
    claimed value is recorded rather than trusted.
    """

    t_standard_us: int
    t_express_us: int

    @property
    def standard_hz(self) -> float:
        return 1e6 / self.t_standard_us if self.t_standard_us else 0.0

    @property
    def express_hz(self) -> float:
        return 1e6 / self.t_express_us if self.t_express_us else 0.0


def decode_samplerate(payload: bytes) -> SampleRate:
    """Decode the 4-byte GET_SAMPLERATE payload."""
    if len(payload) != 4:
        raise ProtocolError(
            f"GET_SAMPLERATE payload must be 4 bytes, got {len(payload)}"
        )

    t_standard, t_express = struct.unpack("<HH", payload)
    return SampleRate(t_standard_us=t_standard, t_express_us=t_express)


# --------------------------------------------------------------------------
# Legacy scan nodes
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Sample:
    """One measurement.

    Fixed-point fields are the wire values; the float properties exist for
    display and analysis only. Production C++ should carry the integers and
    convert at the API boundary, if at all.
    """

    # 0-63, the raw 6-bit field. NOTE: the vendor SDK rescales this to 0-255
    # (`(sync_quality >> 2) << 2`) when building its HQ node, so SDK quality
    # values are 4x these. Kept raw here because the histogram structure is
    # what the analysis looks at; do not compare the two scales directly.
    quality: int
    angle_q6: int  # Degrees in Q6 (1/64 degree units).
    dist_q2: int  # Millimetres in Q2 (1/4 mm units). 0 means NO RETURN.
    start: bool  # The S flag: this sample begins a new revolution.
    interpolated: bool = False  # True for express samples with derived angles.

    @property
    def angle_deg(self) -> float:
        return self.angle_q6 / 64.0

    @property
    def dist_mm(self) -> float:
        return self.dist_q2 / 4.0

    @property
    def valid(self) -> bool:
        """``dist_q2 == 0`` means the beam got no return, not zero distance.

        On this unit roughly 68% of samples in an open room are invalid, so
        this is the common case. Any API that returns a bare float distance
        leaks the sentinel into every consumer.
        """
        return self.dist_q2 != 0


def looks_like_node_start(byte0: int, byte1: int) -> bool:
    """Cheap plausibility test for the first two bytes of a legacy node.

    Two redundant bits are checked:

      * ``byte0`` bit 0 is the start flag S, and bit 1 is its inverse !S.
        They must differ.
      * ``byte1`` bit 0 is a constant check bit, always 1.

    That is only three constrained bits, so a random byte pair passes about
    one time in four. This function is therefore not sufficient on its own --
    see :func:`find_node_alignment`.
    """
    s = byte0 & 0x01
    s_inverted = (byte0 >> 1) & 0x01
    check = byte1 & 0x01

    return s != s_inverted and check == 1


def decode_legacy_node(node: bytes) -> Sample:
    """Decode one five-byte legacy scan node.

    Layout::

        byte 0   bit 0     S, start of a new revolution
                 bit 1     !S, the inverse of bit 0
                 bits 2-7  quality (6 bits)
        byte 1   bit 0     C, check bit, always 1
                 bits 1-7  angle_q6 bits 0-6
        byte 2   bits 0-7  angle_q6 bits 7-14
        byte 3   distance_q2 low byte   (little-endian uint16 with byte 4)
        byte 4   distance_q2 high byte

    Note the asymmetry that makes this easy to fumble: the angle is split
    7 bits / 8 bits across a byte boundary because bit 0 of byte 1 is stolen
    for the check bit, while the distance is a plain little-endian uint16.
    """
    if len(node) != LEGACY_NODE_LEN:
        raise ProtocolError(
            f"legacy node must be {LEGACY_NODE_LEN} bytes, got {len(node)}"
        )

    byte0, byte1, byte2, byte3, byte4 = node

    if not looks_like_node_start(byte0, byte1):
        raise ProtocolError(f"node failed redundancy check: {node.hex()}")

    start = bool(byte0 & 0x01)
    quality = byte0 >> 2

    angle_q6 = (byte1 >> 1) | (byte2 << 7)
    dist_q2 = byte3 | (byte4 << 8)

    return Sample(quality=quality, angle_q6=angle_q6, dist_q2=dist_q2, start=start)


#: How many consecutive plausible nodes are required before byte alignment is
#: believed. With a per-node false-positive rate near 1-in-4, ten in a row
#: leaves a chance around 1e-6 of locking onto a wrong offset.
RESYNC_CONFIRM_NODES = 10


def find_node_alignment(
    buffer: bytes,
    confirm: int = RESYNC_CONFIRM_NODES,
) -> int | None:
    """Return the offset of the first byte of a plausible run of nodes.

    The scan stream has **no frame sync word**. Once byte alignment is lost
    there is nothing to search for except the redundancy inside the nodes
    themselves, so recovery means finding an offset at which ``confirm``
    consecutive nodes all pass :func:`looks_like_node_start`.

    Returns ``None`` when the buffer is too short to confirm any offset; the
    caller should read more bytes and try again rather than discarding data.
    """
    needed = confirm * LEGACY_NODE_LEN

    for offset in range(LEGACY_NODE_LEN):
        if offset + needed > len(buffer):
            break

        if all(
            looks_like_node_start(buffer[pos], buffer[pos + 1])
            for pos in range(offset, offset + needed, LEGACY_NODE_LEN)
        ):
            return offset

    return None
