#!/usr/bin/env python3
"""Check every point the Lidar adapter published against the Python decoder.

Reads <stem>.adapter.dump (written by cpp/tests/LidarReplayCheck) and decodes
<stem>.rpraw with this repository's Python decoder. Each Python revolution is
transformed to the robot frame here, independently of the C++ (negate, wrap
into [-pi, +pi), sort; metres = dist_q2 / 4000), and every published scan must
match one revolution point for point. Legacy and express only.

A published scan with no matching revolution is expected only next to an
overrun discontinuity, where the SDK and the Python decoder are already known
to differ (findings, trap 10).

Usage
  python3 python/tools/lidar_adaptercheck.py captures/<stem>.rpraw
"""

from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import lidar_rawcat  # noqa: E402
from heron.lidar import recording  # noqa: E402

BEARING_TOL = 2e-5   # radians; float32 rounding of the C++ arithmetic
RANGE_TOL = 1e-6     # metres


def robot_bearing(angle_q6: int) -> float:
    """Sensor Q6 angle to robot bearing, the way the SDK and the adapter define it."""
    q14 = (angle_q6 << 8) // 90                      # the SDK's Q6 -> Q14
    rad = q14 * (2.0 * math.pi / 65536.0)
    b = math.fmod(-rad + math.pi, 2.0 * math.pi)     # negate, then wrap
    if b < 0:
        b += 2.0 * math.pi
    b -= math.pi
    return -math.pi if b >= math.pi else b


def python_revolutions(raw_path: str) -> tuple[str, list[list[tuple[float, float]]], list[int]]:
    with recording.RawReader(raw_path) as reader:
        data, _, _, mode = lidar_rawcat.scan_payload(reader)
    if mode not in ("legacy", "express"):
        raise SystemExit(f"  no Python decoder for {mode}")
    decode = lidar_rawcat.decode_all_express if mode == "express" else lidar_rawcat.decode_all
    samples, _, _ = decode(data)
    revs, counts, current = [], [], None
    for s in samples + [None]:
        if s is None or s.start:
            if current:
                pts = sorted((robot_bearing(x.angle_q6), x.dist_q2 / 4000.0)
                             for x in current if x.dist_q2 > 0)
                revs.append(pts)
                counts.append(len(current))
            current = [s] if s is not None else None
        elif current is not None:
            current.append(s)
    return mode, revs, counts


def read_adapter_dump(path: str) -> list[tuple[int, int, list[tuple[float, float]]]]:
    scans = []
    with open(path) as f:
        for line in f:
            parts = line.split()
            if parts[0] == "scan":
                scans.append((int(parts[1]), int(parts[2]), []))
            else:
                scans[-1][2].append((float(parts[0]), float(parts[1])))
    return scans


def same(a: list[tuple[float, float]], b: list[tuple[float, float]]) -> bool:
    if len(a) != len(b):
        return False
    for (ab, ar), (bb, br) in zip(a, b):
        # Bearings near the seam can sit at -pi on one side and just under +pi
        # on the other after float rounding; compare on the circle.
        d = abs(ab - bb)
        if min(d, 2.0 * math.pi - d) > BEARING_TOL or abs(ar - br) > RANGE_TOL:
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw")
    args = parser.parse_args()
    stem = args.raw[:-6] if args.raw.endswith(".rpraw") else args.raw

    mode, revs, counts = python_revolutions(args.raw)
    published = read_adapter_dump(stem + ".adapter.dump")
    by_count: dict[int, list[int]] = {}
    for i, c in enumerate(counts):
        by_count.setdefault(c, []).append(i)

    matched, unmatched = 0, []
    for seq, raw_count, pts in published:
        if any(same(pts, revs[i]) for i in by_count.get(raw_count, [])):
            matched += 1
        else:
            unmatched.append(seq)
    print(f"  vs Python decoder ({mode}): {matched} of {len(published)} published scans match "
          f"a revolution point for point" + (f"; unmatched seq {unmatched}" if unmatched else ""))
    return 0 if matched == len(published) else 1


if __name__ == "__main__":
    sys.exit(main())
