"""Argument parsing and transport setup shared by every probe."""

from __future__ import annotations

import argparse
import datetime as _dt
import os
import sys

from . import platform_info, render
from .transport import DEFAULT_BAUD, DEFAULT_PORT, ReplayTransport, SerialTransport


def capture_dir(explicit: str | None = None) -> str:
    """Where captures are written.

    ``scripts/deploy_pi.sh`` exports HERON_CAPTURE_DIR so that recordings
    land outside the rsync mirror, which is deleted and rewritten on every
    deploy.
    """
    path = explicit or os.environ.get("HERON_CAPTURE_DIR") or "captures"
    os.makedirs(path, exist_ok=True)
    return path


def capture_basename(tag: str) -> str:
    stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in tag)
    return f"{safe}_{stamp}"


def base_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"serial device (default: {DEFAULT_PORT})",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--replay",
        metavar="FILE.rpraw",
        help="replay a recorded capture instead of opening the serial port",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.0,
        help="replay speed; 0 = as fast as possible, 1.0 = real time",
    )
    return parser


def open_transport(args: argparse.Namespace):
    """Return a transport, and print what it actually opened.

    Every probe goes through here, so every probe gets ``--replay`` for free.
    """
    if args.replay:
        print(f"  transport       replay of {args.replay}")
        return ReplayTransport(args.replay, speed=args.speed)

    port, note = platform_info.resolve_port(args.port)
    print(f"  transport       serial {port} @ {args.baud} 8N1")
    if note:
        print(f"                  NOTE: {note}")

    try:
        return SerialTransport(port=port, baud=args.baud, motor=False)
    except ImportError:
        sys.exit(
            "ERROR: pyserial is not installed.\n"
            "  On the Pi:   sudo apt-get install python3-serial\n"
            "  Or re-run:   scripts/provision_pi.sh <user@host>\n"
            "  On the host: use --replay to work from a recorded capture."
        )
    except OSError as exc:
        sys.exit(
            f"ERROR: cannot open {port}: {exc}\n"
            "  Check the device is attached, that /dev/rplidar exists, and\n"
            "  that you are in the 'dialout' group (log out and back in\n"
            "  after provisioning for group changes to take effect)."
        )


def print_header(title: str) -> None:
    """Banner plus the environment facts that explain odd results later."""
    print(render.banner(title))
    print()
    for line in platform_info.summary_lines():
        print(line)
