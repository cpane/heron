#!/usr/bin/env python3
"""Motion table points from a heron_robot log: what each held command did.

A floor's motion table (profiles/surfaces/<drive>/<name>.conf, p<n> lines;
navigation design §19, docs/rig-calibration.md §8.8) is measured by driving
in Manual from rest, one command held 3 to 4 s at a time, with
heron_robot's status lines in a log:

    ssh -t <user>@<pi> '~/heron/bin/heron_robot --lines 10 2>&1 | tee ~/heron/captures/<name>.log'
    scripts/fetch_captures.sh <user>@<pi>
    python3 python/tools/motion_points.py captures/<name>.log

This finds each stretch where the wheels were told one translation (the
status line's "out"), armed, for at least --min-seconds, and prints the
command and what the robot made of it, as the bench rig's tables were
worked out by hand (2026-11-16 and -18):

  direction  the command's, to the nearest 5 degrees (right is -90)
  size       its length, fractions of full drive
  vx, vy     m/s, robot frame (heron_robot's "speed"), averaged from
             --settle seconds in, once the robot is up to speed
  wz         rad/s, from the heading's change over the same span, not the
             OTOS's turn rate
  stall      under 0.02 m/s: written 0 0 0, as the table keeps stalls

then the p<n> lines to paste over the surface file's, numbered from
--first. The robot must start each stretch from rest, and let go between:
a command changed while held (by more than 15 degrees or 0.05) starts a
new stretch. Sliding the robot back between runs, the stick centred, does
not show.

Standard library only; reads the log, writes nothing.
"""

import argparse
import math
import re
import sys

STATUS = re.compile(
    r"\s(ARMED)\s.*?throttle (?P<throttle>[\d.]+)"
    r".*?\sout (?P<ox>\S+) (?P<oy>\S+) \S+"
    r".*?\sspeed (?P<vx>\S+) (?P<vy>\S+) m/s"
    r".*?\sheading\s+(?P<heading>\S+) deg"
)

STALL_MPS = 0.02  # MotionTable's kStallMps


def stretches(lines, min_lines):
    """Runs of status lines with one held translation, at least min_lines long."""
    found = []
    current = []
    first_line = 0

    def close():
        if len(current) >= min_lines:
            found.append((first_line, current[:]))

    for number, text in enumerate(lines, 1):
        m = STATUS.search(text)
        if not m:
            continue
        ox, oy = float(m["ox"]), float(m["oy"])
        size = math.hypot(ox, oy)
        row = dict(
            line=number,
            throttle=float(m["throttle"]),
            ox=ox,
            oy=oy,
            vx=float(m["vx"]),
            vy=float(m["vy"]),
            heading=float(m["heading"]),
        )
        if size < 0.05:
            close()
            current = []
            continue
        if current:
            a = current[0]
            turned = abs(math.remainder(math.atan2(oy, ox) - math.atan2(a["oy"], a["ox"]), 2 * math.pi))
            if turned > math.radians(15) or abs(size - math.hypot(a["ox"], a["oy"])) > 0.05:
                close()
                current = []
        if not current:
            first_line = number
        current.append(row)
    close()
    return found


def point(rows, per_second, settle_s):
    """The command and its motion, from settle_s seconds in."""
    late = rows[int(round(settle_s * per_second)):]
    ox = sum(r["ox"] for r in rows) / len(rows)
    oy = sum(r["oy"] for r in rows) / len(rows)
    vx = sum(r["vx"] for r in late) / len(late)
    vy = sum(r["vy"] for r in late) / len(late)
    # The heading's change, unwrapped through +-180.
    turned = 0.0
    for before, after in zip(late, late[1:]):
        turned += math.remainder(after["heading"] - before["heading"], 360.0)
    span = (len(late) - 1) / per_second
    wz = math.radians(turned) / span if span > 0 else 0.0
    direction = 5 * round(math.degrees(math.atan2(oy, ox)) / 5)
    if direction == -180:
        direction = 180
    stall = math.hypot(vx, vy) < STALL_MPS
    return dict(
        direction=direction,
        size=math.hypot(ox, oy),
        throttle=sum(r["throttle"] for r in rows) / len(rows),
        vx=0.0 if stall else vx,
        vy=0.0 if stall else vy,
        wz=0.0 if stall else wz,
        stall=stall,
        seconds=len(rows) / per_second,
        span=span,
    )


def number(value):
    """As the surface files write them: 2 decimals, no trailing zeros, no -0."""
    text = f"{value:.2f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("log", help="heron_robot's output, run with --lines N")
    parser.add_argument("--lines", type=int, default=10, help="status lines a second the log was run with (default 10)")
    parser.add_argument("--settle", type=float, default=1.0, help="seconds from the start of each run left out (default 1)")
    parser.add_argument("--min-seconds", type=float, default=2.0, help="shorter runs are left out (default 2)")
    parser.add_argument("--first", type=int, default=1, help="number the p<n> lines from this (default 1)")
    parser.add_argument("--from-line", type=int, default=1, help="read the log from this line")
    parser.add_argument("--to-line", type=int, default=0, help="and to this one (default the end)")
    args = parser.parse_args()

    with open(args.log, encoding="utf-8", errors="replace") as f:
        lines = f.read().split("\n")
    end = args.to_line if args.to_line > 0 else len(lines)
    window = [""] * (args.from_line - 1) + lines[args.from_line - 1:end]

    min_lines = max(2, int(round(args.min_seconds * args.lines)))
    runs = stretches(window, min_lines)
    if not runs:
        print("No held command of", args.min_seconds, "s or more found: was it run with --lines and armed?")
        return 1
    points = []
    print(f"{len(runs)} held command(s), speeds from {args.settle:g} s in:\n")
    print(" log line  throttle  dir  size   held   vx m/s  vy m/s  wz rad/s")
    for first, rows in runs:
        p = point(rows, args.lines, args.settle)
        points.append(p)
        print(
            f"{first:9d}  {p['throttle']:8.2f}  {p['direction']:+4d}  {p['size']:.2f}  {p['seconds']:4.1f} s"
            f"  {p['vx']:+.3f}  {p['vy']:+.3f}  {p['wz']:+.3f}{'  STALL' if p['stall'] else ''}"
        )
    print("\nTo paste into the surface file (check each against the table above):\n")
    for n, p in enumerate(points, args.first):
        print(
            f"p{n:02d} = {p['direction']} {number(p['size'])} {number(p['vx'])} {number(p['vy'])} {number(p['wz'])}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
