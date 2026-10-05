"""Host/target environment facts worth recording alongside every capture.

The RPLIDAR motor is a meaningful load on a Pi 3, and the board used for this
evaluation already reports ``throttled=0x50000`` -- under-voltage and
throttling have both occurred since boot. Recording that in every capture
header and footer means that when a result looks strange weeks later, the
evidence is already in the file rather than needing to be reconstructed.

Everything here degrades gracefully: on macOS the Pi-specific probes simply
return None, so the host-side tools can import this module.
"""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys

#: Bit meanings of vcgencmd get_throttled.
THROTTLE_BITS = {
    0: "under-voltage detected (now)",
    1: "ARM frequency capped (now)",
    2: "currently throttled",
    3: "soft temperature limit active (now)",
    16: "under-voltage has occurred",
    17: "ARM frequency capping has occurred",
    18: "throttling has occurred",
    19: "soft temperature limit has occurred",
}


def _read_text(path: str) -> str | None:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return handle.read().strip("\x00\n ")
    except OSError:
        return None


def pi_model() -> str | None:
    return _read_text("/proc/device-tree/model")


def throttled() -> str | None:
    """Raw ``vcgencmd get_throttled`` output, e.g. ``throttled=0x50000``."""
    if not shutil.which("vcgencmd"):
        return None
    try:
        out = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None


def throttled_flags(value: str | None) -> list[str]:
    """Decode a ``throttled=0x...`` string into human-readable flags."""
    if not value or "=" not in value:
        return []
    try:
        bits = int(value.split("=", 1)[1], 0)
    except ValueError:
        return []
    return [text for bit, text in THROTTLE_BITS.items() if bits & (1 << bit)]


def cpu_temp_c() -> float | None:
    raw = _read_text("/sys/class/thermal/thermal_zone0/temp")
    if raw is None:
        return None
    try:
        return int(raw) / 1000.0
    except ValueError:
        return None


def resolve_port(port: str) -> tuple[str, str | None]:
    """Resolve a device path, following the udev symlink if there is one.

    Returns ``(path, note)`` where the note explains any fallback taken, so a
    probe can say out loud that it is using an unstable name.
    """
    if os.path.exists(port):
        try:
            real = os.path.realpath(port)
        except OSError:
            real = port
        note = f"symlink -> {real}" if real != port else None
        return port, note

    from .transport import FALLBACK_PORT

    if port != FALLBACK_PORT and os.path.exists(FALLBACK_PORT):
        return FALLBACK_PORT, (
            f"{port} not present; fell back to {FALLBACK_PORT}. Run "
            "scripts/provision_pi.sh to install the udev rule -- ttyUSB "
            "numbering is not stable once a second USB serial device exists."
        )

    return port, f"{port} does not exist"


def snapshot() -> dict[str, object]:
    """Everything worth putting in a capture header."""
    value = throttled()
    return {
        "hostname": platform.node(),
        "model": pi_model(),
        "system": platform.system(),
        "machine": platform.machine(),
        "kernel": platform.release(),
        "python": sys.version.split()[0],
        "throttled": value,
        "throttled_flags": throttled_flags(value),
        "cpu_temp_c": cpu_temp_c(),
    }


def summary_lines() -> list[str]:
    """Same information, formatted for a probe banner."""
    info = snapshot()
    lines = [
        f"    host            {info['hostname']}",
        f"    model           {info['model'] or info['machine']}",
        f"    kernel          {info['kernel']}",
        f"    python          {info['python']}",
    ]

    value = info["throttled"]
    if value:
        lines.append(f"    throttled       {value}")
        for flag in info["throttled_flags"] or []:
            lines.append(f"                      - {flag}")
    else:
        lines.append("    throttled       (vcgencmd unavailable)")

    temp = info["cpu_temp_c"]
    if temp is not None:
        lines.append(f"    cpu temp        {temp:.1f} C")

    return lines
