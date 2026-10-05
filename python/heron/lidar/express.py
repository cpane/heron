"""Express scan: the 84-byte capsule frame.

Express mode roughly doubles the sample rate (254 us/sample against 508 on
this unit) by packing 32 measurements into one 84-byte frame and sending only
a *start angle* plus a small per-sample offset, instead of a full angle per
sample.

The structural consequence, and the reason this module exists separately from
:mod:`heron.lidar.protocol`:

    A capsule cannot be decoded until the NEXT capsule has arrived.

The per-sample angles are interpolated between this capsule's start angle and
the next one's, so the decoder is permanently one frame -- about 21 ms at this
sample rate -- behind the wire. That is a latency property baked into the
format, not an implementation choice, and it dictates a different buffering
design from legacy mode. Find this out here rather than in production.

Frame layout (84 bytes, little-endian)::

    byte 0      high nibble = 0xA sync, low nibble = checksum bits 0-3
    byte 1      high nibble = 0x5 sync, low nibble = checksum bits 4-7
    bytes 2-3   uint16: bit 15 = start flag S, bits 0-14 = start angle in Q6
    bytes 4-83  16 cabins of 5 bytes, 2 measurements each

Cabin (5 bytes)::

    bytes 0-1   uint16 distance_angle_1: bits 2-15 = distance Q2,
                                         bits 0-1  = angle offset bits 4-5
    bytes 2-3   uint16 distance_angle_2: same, for the second measurement
    byte 4      low nibble  = angle offset 1, bits 0-3
                high nibble = angle offset 2, bits 0-3

The checksum is the XOR of all 82 bytes after the first two.

The nibble order above was confirmed against the physical device: frame
headers arrive as ``a9 5d``, ``ac 5b``, ``a5 57`` -- the high nibbles are
invariably 0xA and 0x5, and the low nibbles reproduce the computed XOR
exactly. It is worth checking rather than assuming, because getting it
backwards still passes a naive "did I find two sync nibbles" test often
enough to look like intermittent corruption instead of a decode bug.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from .protocol import ProtocolError, Sample

#: Every express frame is exactly 84 bytes.
CAPSULE_LEN = 84

#: 16 cabins, two measurements each.
SAMPLES_PER_CAPSULE = 32

SYNC1 = 0xA
SYNC2 = 0x5

#: Payload for EXPRESS_SCAN, matching sl_lidar_payload_express_scan_t:
#: working_mode (u8) = 0 for the legacy capsule format, working_flags (u16),
#: param (u16). Five bytes, all zero.
EXPRESS_SCAN_PAYLOAD = bytes(5)


def express_scan_payload(
    working_mode: int = 0, working_flags: int = 0, param: int = 0
) -> bytes:
    """Build an EXPRESS_SCAN payload with an explicit working_mode.

    Five bytes: working_mode (u8), working_flags (u16), param (u16), little
    endian. The request carries a payload, so it is checksummed, and getting
    that wrong is silent -- the device simply never replies.

    A caution on ``working_mode``. It is *not* reliably the same numbering as
    the GET_LIDAR_CONF scan-mode table: mode 0 in that table is Standard,
    which streams legacy 0x81 nodes, yet ``working_mode`` 0 here yields 0x82
    express capsules. Modes 2-4 were confirmed to yield 0x84 ultra capsules.
    The mapping for the rest is unverified on this unit, so callers should
    dispatch on the data type in the descriptor the device actually returns
    rather than on what they asked for.
    """
    return struct.pack("<BHH", working_mode & 0xFF, working_flags, param)

#: Quality the vendor SDK synthesises for a valid express return (0x2F). There
#: is no quality field in the capsule format; this is a constant, not a
#: measurement.
SYNTHETIC_QUALITY = 0x2F


@dataclass(frozen=True)
class Capsule:
    """One parsed 84-byte frame, before angle interpolation."""

    start_angle_q6: int
    start_flag: bool
    #: 32 entries of (dist_q2, angle_offset_q3), in transmission order.
    measurements: tuple[tuple[int, int], ...]
    checksum_ok: bool
    raw: bytes


def looks_like_capsule(byte0: int, byte1: int) -> bool:
    """Sync-nibble test, used to find frame alignment in the stream."""
    return (byte0 >> 4) == SYNC1 and (byte1 >> 4) == SYNC2


def parse_capsule(data: bytes) -> Capsule:
    """Parse one 84-byte capsule. Does not interpolate angles."""
    if len(data) != CAPSULE_LEN:
        raise ProtocolError(
            f"express capsule must be {CAPSULE_LEN} bytes, got {len(data)}"
        )
    if not looks_like_capsule(data[0], data[1]):
        raise ProtocolError(
            f"bad express sync nibbles: {data[0]:02x} {data[1]:02x}"
        )

    expected = (data[0] & 0x0F) | ((data[1] & 0x0F) << 4)
    actual = 0
    for byte in data[2:]:
        actual ^= byte

    (start_word,) = struct.unpack_from("<H", data, 2)
    start_flag = bool(start_word & 0x8000)
    start_angle_q6 = start_word & 0x7FFF

    measurements = []
    for cabin in range(16):
        base = 4 + cabin * 5
        d1, d2 = struct.unpack_from("<HH", data, base)
        offsets = data[base + 4]

        # The low two bits of each distance word are not distance: they are
        # the high bits of that measurement's angle offset.
        dist1_q2 = d1 & 0xFFFC
        dist2_q2 = d2 & 0xFFFC
        offset1_q3 = (offsets & 0x0F) | ((d1 & 0x03) << 4)
        offset2_q3 = (offsets >> 4) | ((d2 & 0x03) << 4)

        measurements.append((dist1_q2, offset1_q3))
        measurements.append((dist2_q2, offset2_q3))

    return Capsule(
        start_angle_q6=start_angle_q6,
        start_flag=start_flag,
        measurements=tuple(measurements),
        checksum_ok=(expected == actual),
        raw=bytes(data),
    )


class ExpressDecoder:
    """Turn a sequence of capsules into samples.

    Feed capsules in arrival order with :meth:`push`. Each call returns the
    samples belonging to the *previous* capsule, because their angles are
    interpolated across the gap between the two start angles. The first call
    therefore returns nothing.

    The arithmetic mirrors ``_capsuleToNormal`` in SLAMTEC's C++ SDK, kept in
    fixed point so the C++ version can be a direct transliteration.
    """

    def __init__(self) -> None:
        self._previous: Capsule | None = None
        self.capsules = 0
        self.checksum_failures = 0

    def reset(self) -> None:
        self._previous = None

    def push(self, capsule: Capsule) -> list[Sample]:
        self.capsules += 1
        if not capsule.checksum_ok:
            self.checksum_failures += 1

        previous, self._previous = self._previous, capsule
        if previous is None:
            return []

        # Angles advance from the previous capsule's start angle to this
        # one's, spread evenly over the 32 measurements in between.
        current_start_q8 = capsule.start_angle_q6 << 2
        previous_start_q8 = previous.start_angle_q6 << 2

        diff_q8 = current_start_q8 - previous_start_q8
        if previous_start_q8 > current_start_q8:
            diff_q8 += 360 << 8  # The capsule straddles the 0/360 seam.

        # q8 -> q16 is <<8; dividing by 32 samples is >>5; net <<3.
        increment_q16 = diff_q8 << 3
        angle_raw_q16 = previous_start_q8 << 8

        full_turn_q16 = 360 << 16
        samples = []
        for dist_q2, offset_q3 in previous.measurements:
            # offset_q3 << 13 promotes Q3 to Q16; >> 10 demotes Q16 to Q6.
            angle_q6 = (angle_raw_q16 - (offset_q3 << 13)) >> 10
            angle_q6 %= 360 << 6

            # The per-sample start flag is DERIVED, not transmitted: it marks
            # the sample whose interpolated angle crosses zero. The capsule
            # header's bit 15 flags the capsule, not a sample, and using it
            # directly yields at most one start per capsule in the wrong place
            # -- which shows up as zero revolutions downstream.
            start = (
                (angle_raw_q16 + increment_q16) % full_turn_q16
            ) < increment_q16

            samples.append(
                Sample(
                    # Express frames carry no per-sample quality field. The
                    # vendor SDK synthesises 0x2F for a valid return and 0 for
                    # no return; the same constant is used here so a C++ port
                    # against this code produces identical numbers. It is NOT
                    # a measured quality and must not be compared against
                    # legacy quality values.
                    quality=0 if dist_q2 == 0 else SYNTHETIC_QUALITY,
                    angle_q6=angle_q6,
                    dist_q2=dist_q2,
                    start=start,
                    interpolated=True,
                )
            )
            angle_raw_q16 += increment_q16

        return samples


def find_capsule_alignment(buffer: bytes, confirm: int = 3) -> int | None:
    """Find the offset of a run of plausible capsules.

    Only eight bits are constrained per frame (two sync nibbles), so a single
    frame's sync test is weak; several consecutive frames are required, the
    same way legacy node alignment works.
    """
    for offset in range(CAPSULE_LEN):
        if offset + confirm * CAPSULE_LEN > len(buffer):
            break
        if all(
            looks_like_capsule(buffer[pos], buffer[pos + 1])
            for pos in range(offset, offset + confirm * CAPSULE_LEN, CAPSULE_LEN)
        ):
            return offset
    return None
