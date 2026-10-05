#!/usr/bin/env python3
"""Compare the C++ scan builder's gap detector with the Python reference.

For each SDK scan in <stem>.sdkdump, cpp/tests/ScanBuilderCheck reports the
widest sampling gap buildScan() found. This tool decodes the same .rpraw with
the Python decoder, pairs each SDK scan with its Python revolution by content
(as lidar_sdkcheck does), and compares against Revolution.sampling_gap_q6(),
the reference the C++ must match. Legacy and express only; for ultra it
reports the C++ results alone.

The two can differ by a hair: the SDK converts Q6 angles to Q14 with an
integer division, so a match within 0.02 degrees is exact up to that rounding.

Usage
  python3 python/tools/lidar_gapcheck.py captures/<stem>.rpraw [--bin build-host/bin/scan_builder_check]
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import lidar_rawcat  # noqa: E402
from lidar_sdkcheck import read_dump, to_sdk_units  # noqa: E402
from heron.lidar import recording  # noqa: E402
from heron.lidar.scan import Revolution  # noqa: E402

#: Microseconds per sample for this unit's five scan modes, as GET_LIDAR_CONF
#: reported them (lidar_info, 2026-10-03).
US_PER_SAMPLE = {0: 508.0, 1: 254.0, 2: 127.0, 3: 127.0, 4: 201.0}
TOLERANCE_DEG = 0.02


def scan_mode(dump_path: str) -> int:
    with open(dump_path) as f:
        for line in f:
            if line.startswith("# mode"):
                return int(line.split()[2])
    raise SystemExit(f"ERROR: {dump_path} has no '# mode' line")


def cpp_results(binary: str, dump_path: str, us: float) -> list[tuple[int, int, float, int, bool]]:
    out = subprocess.run([binary, dump_path, str(us)], check=True, capture_output=True, text=True)
    rows = []
    for line in out.stdout.splitlines():
        if line.startswith("#"):
            continue
        i, raw, valid, gap, dropped, merged = line.split()
        rows.append((int(raw), int(valid), float(gap), int(dropped), merged == "1"))
    return rows


def python_revolutions(raw_path: str, express: bool) -> dict[tuple, float]:
    """Map each Python revolution, in SDK units, to its reference gap in degrees."""
    with recording.RawReader(raw_path) as reader:
        data, _, _, _ = lidar_rawcat.scan_payload(reader)
    decode = lidar_rawcat.decode_all_express if express else lidar_rawcat.decode_all
    samples, _, _ = decode(data)

    revs: dict[tuple, float] = {}
    current: list | None = None
    for s in samples + [None]:
        if s is None or s.start:
            if current:
                key = tuple(to_sdk_units(x, express) for x in current)
                rev = Revolution(index=0, samples=[types.SimpleNamespace(sample=x) for x in current])
                revs[key] = rev.sampling_gap_q6() / 64.0
            current = [s] if s is not None else None
        elif current is not None:
            current.append(s)
    return revs


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw", help="a .rpraw written by lidar_record")
    parser.add_argument("--bin", default="build-host/bin/scan_builder_check")
    args = parser.parse_args()
    stem = args.raw[:-6] if args.raw.endswith(".rpraw") else args.raw
    dump_path = stem + ".sdkdump"

    mode = scan_mode(dump_path)
    ans_type, scans = read_dump(dump_path)
    cpp = cpp_results(args.bin, dump_path, US_PER_SAMPLE[mode])
    print(f"  {os.path.basename(stem)}: mode {mode}, {len(scans)} scans")

    merged = [i for i, r in enumerate(cpp) if r[4]]
    wide = [i for i, r in enumerate(cpp) if r[2] > 10.0]
    print(f"  C++: widest gap max {max(r[2] for r in cpp):.2f} deg; "
          f"scans over 10 deg {wide or 'none'}; flagged merged {merged or 'none'}")

    if ans_type not in (0x81, 0x82):
        print("  Python reference: none for this mode (ultra)")
        return 0

    reference = python_revolutions(args.raw, express=(ans_type == 0x82))
    compared, worst, unmatched = 0, 0.0, 0
    for (_, nodes), row in zip(scans, cpp):
        ref = reference.get(tuple(nodes))
        if ref is None:
            unmatched += 1
            continue
        compared += 1
        worst = max(worst, abs(row[2] - ref))
    ok = compared > 0 and worst <= TOLERANCE_DEG
    print(f"  vs Python sampling_gap_q6(): {compared} scans compared, worst difference "
          f"{worst:.4f} deg, {unmatched} without an identical Python revolution -> "
          f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
