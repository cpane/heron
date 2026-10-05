#!/usr/bin/env python3
"""Look for data loss in an Express session recorded by lidar_record. Stdlib only.

Three independent views of the same capture:

  1. arrival   pauses between the recorded read() chunks. A stalled reader
               shows as a silence; the one at the very start is spin-up.
  2. capsules  jumps between consecutive capsule start angles, taken straight
               from the wire before anything interpolates. This is the ground
               truth for whether capsules were lost.
  3. SDK       what grabScanDataHq() delivered (<stem>.sdkdump): samples per
               scan and the widest gap between samples, which is what the
               adapter will see.

Why all three: measured 2026-10-04, an overrun can reach the SDK's output as
a wide gap, or as two revolutions merged into one over-full scan with no gap
at all. Only the capsule view sees every loss; only the SDK view shows which
symptom a consumer would get.

Usage
  python3 python/tools/lidar_overrun.py captures/<tag>-sdk1_<stamp>.rpraw
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import lidar_rawcat  # noqa: E402
from lidar_sdkcheck import read_dump  # noqa: E402
from heron.lidar import express, protocol, recording  # noqa: E402

#: A start-angle step this many times the median means capsules are missing.
#: Healthy steps are very regular: the largest in 9 healthy express captures is
#: 1.02x the median (measured 2026-10-04), and one lost capsule doubles the
#: step, so 1.5x sits well clear of both.
JUMP_FACTOR = 1.5
#: A scan whose sample count is off the median by more than this fraction is
#: flagged; a merged revolution reads ~1.6-1.8x.
COUNT_TOLERANCE = 0.10
#: The coverage threshold the Scan type uses (kUnobservedGapRad).
GAP_LIMIT_DEG = 10.0


def arrival(raw_path: str) -> None:
    with recording.RawReader(raw_path) as reader:
        rx = [c for c in reader.chunks() if c.direction == "rx"]
    t = [c.t_mono_ns for c in rx]
    span_s = (t[-1] - t[0]) / 1e9
    nbytes = sum(len(c.data) for c in rx)
    gaps = sorted((((b - a) / 1e6, (a - t[0]) / 1e9) for a, b in zip(t, t[1:])), reverse=True)
    print(f"arrival    {len(rx)} reads, {nbytes} bytes over {span_s:.1f} s = {nbytes / span_s:.0f} B/s")
    print("           longest silences: "
          + ", ".join(f"{g:.0f} ms at {at:.1f} s" for g, at in gaps[:5]))


def capsules(raw_path: str) -> int:
    with recording.RawReader(raw_path) as reader:
        data, _, _, mode = lidar_rawcat.scan_payload(reader)
    if mode != "express":
        raise SystemExit(f"ERROR: {raw_path} is a {mode} capture; this tool reads express only")

    starts: list[float] = []
    pos, size = 0, express.CAPSULE_LEN
    while pos + size <= len(data):
        if not express.looks_like_capsule(data[pos], data[pos + 1]):
            offset = express.find_capsule_alignment(data[pos:])
            pos += offset if offset else 1
            continue
        try:
            capsule = express.parse_capsule(data[pos : pos + size])
        except protocol.ProtocolError:
            pos += 1
            continue
        if capsule.checksum_ok:
            starts.append(capsule.start_angle_q6 / 64.0)
        pos += size

    steps = [(b - a) % 360.0 for a, b in zip(starts, starts[1:])]
    typical = statistics.median(steps)
    jumps = sorted((s for s in steps if s > JUMP_FACTOR * typical), reverse=True)
    lost = sum(round(s / typical) - 1 for s in jumps)
    print(f"capsules   {len(starts)} parsed; median start-angle step {typical:.1f} deg")
    print(f"           jumps over {JUMP_FACTOR}x median: {len(jumps)}"
          + (f" ({', '.join(f'{s:.1f}' for s in jumps[:6])} deg)" if jumps else ""))
    print(f"           capsules missing, estimated: {lost} (~{lost * 32} samples)")
    return lost


def sdk_view(label: str, path: str) -> tuple[int, int]:
    _, scans = read_dump(path)
    counts, widest = [], []
    for _, nodes in scans:
        angles = sorted(n[0] * 90.0 / 16384.0 for n in nodes)
        counts.append(len(angles))
        if len(angles) > 1:
            inner = [b - a for a, b in zip(angles, angles[1:])]
            widest.append(max(inner + [angles[0] + 360.0 - angles[-1]]))
        else:
            widest.append(360.0)
    median = statistics.median(counts)
    odd_count = [i for i, c in enumerate(counts) if abs(c - median) > COUNT_TOLERANCE * median]
    wide = [i for i, w in enumerate(widest) if w > GAP_LIMIT_DEG]
    print(f"{label:11s}{len(scans)} scans; samples per scan median {median:.0f}, "
          f"min {min(counts)}, max {max(counts)}")
    print(f"           widest gap per scan: median {statistics.median(widest):.1f}, "
          f"max {max(widest):.1f} deg")
    print(f"           caught by the {GAP_LIMIT_DEG:.0f} deg gap check: {len(wide)} scan(s); "
          f"by the sample-count check: {len(odd_count)} scan(s)")
    for i in sorted(set(odd_count) | set(wide))[:8]:
        print(f"             scan {i:3d}: {counts[i]} samples, widest gap {widest[i]:.1f} deg")
    return len(wide), len(odd_count)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw", help="an express .rpraw written by lidar_record")
    args = parser.parse_args()
    stem = args.raw[:-6] if args.raw.endswith(".rpraw") else args.raw

    print("=" * 40)
    print(" Overrun check")
    print("=" * 40)
    print(f"  raw     {args.raw}\n")
    arrival(args.raw)
    lost = capsules(args.raw)
    seen_by_gap = seen_by_count = 0
    for label, path in (("SDK live", stem + ".sdkdump"), ("SDK replay", stem + ".replay.sdkdump")):
        if os.path.exists(path):
            gap, count = sdk_view(label, path)
            if label == "SDK live":
                seen_by_gap, seen_by_count = gap, count

    print()
    if lost == 0:
        print("  No capsules lost.")
    elif seen_by_gap == 0 and seen_by_count == 0:
        print("  Capsules were lost, but no delivered scan shows it (the affected")
        print("  revolutions may have been overwritten before the consumer grabbed them).")
    else:
        print(f"  Capsules were lost. Delivered scans flagged: {seen_by_gap} by gap, "
              f"{seen_by_count} by sample count.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
