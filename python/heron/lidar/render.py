"""Human-facing rendering of protocol bytes.

Deliberately quarantined from :mod:`heron.lidar.protocol`, which stays free
of presentation concerns so it can be transliterated to C++. Nothing in here
is expected to survive into production.
"""

from __future__ import annotations

import math

from . import protocol
from .protocol import Descriptor, DeviceInfo, Health, SampleRate, Sample


def hexdump(data: bytes, width: int = 16, base_offset: int = 0) -> str:
    """Classic offset / hex / ASCII dump."""
    lines = []
    for offset in range(0, len(data), width):
        chunk = data[offset : offset + width]
        hex_part = " ".join(f"{b:02x}" for b in chunk).ljust(width * 3 - 1)
        text = "".join(chr(b) if 0x20 <= b < 0x7F else "." for b in chunk)
        lines.append(f"  {base_offset + offset:08x}  {hex_part}  |{text}|")
    return "\n".join(lines)


def annotate_descriptor(descriptor: Descriptor) -> str:
    """Show the seven descriptor bytes with the 30/2 bit split spelled out."""
    raw = descriptor.raw
    word = int.from_bytes(raw[2:6], "little")

    return "\n".join(
        [
            f"  descriptor  {raw.hex(' ')}",
            f"    byte 0-1   {raw[:2].hex(' ')}  sync (must be a5 5a)",
            f"    byte 2-5   {raw[2:6].hex(' ')}  little-endian uint32 "
            f"= 0x{word:08x}",
            f"      bits 0-29  length = {descriptor.length}",
            f"      bits 30-31 mode   = {descriptor.mode.value} "
            f"({descriptor.mode.name})",
            f"    byte 6     {raw[6]:02x}           data type",
        ]
    )


def annotate_node(node: bytes, index: int = 0, offset: int = 0) -> str:
    """Show a five-byte legacy node with every bit field and its arithmetic.

    This is the core teaching output: the numbers below can be checked by hand
    against the hex, which is how you gain confidence that a C++ decoder using
    the same shifts is correct.
    """
    byte0, byte1, byte2, byte3, byte4 = node

    s = byte0 & 0x01
    s_inverted = (byte0 >> 1) & 0x01
    quality = byte0 >> 2
    check = byte1 & 0x01
    angle_lo7 = byte1 >> 1
    angle_hi8 = byte2
    angle_q6 = angle_lo7 | (angle_hi8 << 7)
    dist_q2 = byte3 | (byte4 << 8)

    s_ok = "ok" if s != s_inverted else "BAD"
    check_ok = "ok" if check == 1 else "BAD"

    return "\n".join(
        [
            f"  node[{index:04d}] @0x{offset:06x}  {node.hex(' ').upper()}",
            f"    b0 = 0b{byte0:08b}  S={s}  !S={s_inverted} ({s_ok})  "
            f"quality=0b{quality:06b}={quality}",
            f"    b1 = 0b{byte1:08b}  C={check} ({check_ok})  "
            f"angle_lo7=0b{angle_lo7:07b}={angle_lo7}",
            f"    b2 = 0b{byte2:08b}  angle_hi8={angle_hi8}",
            f"      angle_q6 = {angle_lo7} | ({angle_hi8} << 7) = {angle_q6}"
            f"  ->  {angle_q6}/64 = {angle_q6 / 64.0:.3f} deg",
            f"    b3b4 = 0x{dist_q2:04x} (LE) = {dist_q2}"
            f"  ->  {dist_q2}/4 = {dist_q2 / 4.0:.1f} mm"
            + ("" if dist_q2 else "   <- NO RETURN"),
        ]
    )


def device_info_table(info: DeviceInfo) -> str:
    return "\n".join(
        [
            # The device's own RESET banner prints this byte as its hex
            # digits -- 0x18 is reported as "Model: 18" -- so showing the
            # decimal 24 alongside it invites confusion. Hex is what matches
            # SLAMTEC's own labelling.
            f"    model           0x{info.model:02x}"
            f"  (the RESET banner calls this \"Model: {info.model:02x}\")",
            f"    firmware        {info.firmware}",
            f"    hardware        {info.hardware}",
            f"    serial (wire)   {info.serial.hex().upper()}",
            f"    serial (label)  {info.serial_text}",
        ]
    )


def health_table(health: Health) -> str:
    lines = [
        f"    status          {health.status} ({health.state_name})",
        f"    error code      {health.error_code}",
    ]
    if health.needs_reset:
        lines.append("    NOTE: status is Error; the device needs RESET to scan.")
    return "\n".join(lines)


def samplerate_table(rate: SampleRate) -> str:
    return "\n".join(
        [
            f"    standard        {rate.t_standard_us} us/sample "
            f"({rate.standard_hz:.0f} samples/s claimed)",
            f"    express         {rate.t_express_us} us/sample "
            f"({rate.express_hz:.0f} samples/s claimed)",
        ]
    )


def ascii_polar(
    samples: list[Sample],
    width: int = 73,
    height: int = 31,
    max_range_mm: float | None = None,
) -> str:
    """Top-down plot of one revolution, for use over SSH with no display.

    Angle 0 is drawn pointing up and angle increases clockwise. Whether that
    matches the physical sensor is exactly what probe 3's ``--bearing-test``
    exists to determine, so the plot is a navigation aid, not a ground truth.
    """
    valid = [s for s in samples if s.valid]
    if not valid:
        return "  (no valid returns in this revolution)"

    if max_range_mm is None:
        distances = sorted(s.dist_mm for s in valid)
        # 95th percentile, so one far outlier does not collapse the scale.
        max_range_mm = distances[int(len(distances) * 0.95)] or distances[-1]

    grid = [[" "] * width for _ in range(height)]
    cx, cy = width // 2, height // 2

    for sample in valid:
        radius = min(sample.dist_mm / max_range_mm, 1.0)
        theta = math.radians(sample.angle_deg)

        # Screen rows are half the angular resolution of columns for a typical
        # terminal cell aspect ratio, so the y scale is halved to keep circles
        # looking round.
        x = cx + int(round(radius * math.sin(theta) * (width // 2)))
        y = cy - int(round(radius * math.cos(theta) * (height // 2)))

        if 0 <= x < width and 0 <= y < height:
            grid[y][x] = "*"

    grid[cy][cx] = "+"

    border = "  +" + "-" * width + "+"
    body = "\n".join("  |" + "".join(row) + "|" for row in grid)

    return "\n".join(
        [
            border,
            body,
            border,
            f"  scale: edge = {max_range_mm / 1000.0:.2f} m   "
            f"'+' = sensor   {len(valid)}/{len(samples)} samples valid",
            "  0 deg is drawn up (sensor front), angle increases "
            "clockwise -- measured 2026-09-26, probe 3 --bearing-test",
        ]
    )


def banner(title: str) -> str:
    """Match the banner style used by the provisioning shell scripts."""
    rule = "=" * 40
    return f"{rule}\n {title}\n{rule}"


def section(title: str) -> str:
    return f"\n{title}\n{'-' * len(title)}"


__all__ = [
    "hexdump",
    "annotate_descriptor",
    "annotate_node",
    "device_info_table",
    "health_table",
    "samplerate_table",
    "ascii_polar",
    "banner",
    "section",
]
