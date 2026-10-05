#!/usr/bin/env python3
"""Check an SDK session recorded by cpp/probes/lidar_record. Stdlib only.

Two independent questions, both answered from files on disk:

  1. Replay fidelity -- did replaying the .rpraw through the SDK reproduce
     the scans the SDK produced live? Compares <stem>.sdkdump (live) with
     <stem>.replay.sdkdump (replay).

  2. Decoder agreement -- does the vendor SDK decode the recorded bytes to
     the same numbers as this repository's Python decoder? Decodes the
     .rpraw with heron.lidar (via lidar_rawcat) and requires every SDK
     scan to appear, node for node, among the Python revolutions. Legacy and
     express only: nothing in Python decodes ultra capsules.

Scans are matched by content, not by index. The SDK publishes latest-wins,
so a consumer that falls behind skips whole revolutions; a skipped scan is
reported as such, never as a mismatch.

Usage
  python3 python/tools/lidar_sdkcheck.py captures/<tag>-sdk1_<stamp>.rpraw
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

import lidar_rawcat  # noqa: E402
from heron.lidar import recording  # noqa: E402

ANS_LEGACY = 0x81
ANS_EXPRESS = 0x82

#: What the SDK puts in `quality` for a valid express return: 0x2F shifted
#: into the SDK's 0-255 scale (handler_capsules.cpp). The Python decoder
#: carries the unshifted 0x2F.
SDK_EXPRESS_QUALITY = 0x2F << 2

Node = tuple[int, int, int, int]  # angle_z_q14, dist_mm_q2, quality, flag


def read_dump(path: str) -> tuple[int, list[tuple[int, list[Node]]]]:
    """Returns (answer type, [(t_us, nodes), ...])."""
    ans_type = -1
    scans: list[tuple[int, list[Node]]] = []
    with open(path) as f:
        for line in f:
            if line.startswith("# mode"):
                ans_type = int(line.split("ans_type")[1], 16)
            elif line.startswith("scan "):
                _, _, _, t_us = line.split()
                scans.append((int(t_us), []))
            elif line.strip():
                a, d, q, fl = (int(x) for x in line.split())
                scans[-1][1].append((a, d, q, fl))
    return ans_type, scans


def to_sdk_units(sample, express: bool) -> Node:
    """A Python sample, in the units and conventions grabScanDataHq() uses.

    Each line mirrors the SDK's own conversion, cited so it can be checked:
      angle   (angle_q6 << 8) / 90                  both handlers
      flag    legacy: the sync bit                  handler_normalnode.cpp
              express: sync | (!sync << 1)          handler_capsules.cpp
      quality legacy: (byte >> 2) << 2, i.e. q * 4  handler_normalnode.cpp
              express: 0x2F << 2 if dist else 0     handler_capsules.cpp
    """
    angle_q14 = (sample.angle_q6 << 8) // 90
    if express:
        flag = 1 if sample.start else 2
        quality = SDK_EXPRESS_QUALITY if sample.dist_q2 else 0
    else:
        flag = 1 if sample.start else 0
        quality = sample.quality << 2
    return (angle_q14, sample.dist_q2, quality, flag)


def python_revolutions(raw_path: str, express: bool) -> list[list[Node]]:
    """Decode the capture in Python and split at start flags, as the SDK does."""
    with recording.RawReader(raw_path) as reader:
        data, _, _, _ = lidar_rawcat.scan_payload(reader)
    decode = lidar_rawcat.decode_all_express if express else lidar_rawcat.decode_all
    samples, _, _ = decode(data)

    revs: list[list[Node]] = []
    current: list[Node] | None = None  # nodes before the first start are dropped
    for s in samples:
        node = to_sdk_units(s, express)
        if s.start:
            if current:
                revs.append(current)
            current = [node]
        elif current is not None:
            current.append(node)
    return revs  # the last, unterminated revolution is never published


def first_difference(a: list[Node], b: list[Node]) -> str:
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return f"node {i}: sdk {x} vs python {y}"
    return f"length: sdk {len(a)} vs python {len(b)}"


def check_replay(live: list[tuple[int, list[Node]]],
                 replay: list[tuple[int, list[Node]]]) -> bool:
    print("\n-- 1. replay fidelity: live SDK scans vs replayed SDK scans")
    live_nodes = [n for _, n in live]
    replay_nodes = [n for _, n in replay]
    identical = live_nodes == replay_nodes
    replay_set = {tuple(n) for n in replay_nodes}
    found = sum(1 for n in live_nodes if tuple(n) in replay_set)
    print(f"  live scans        {len(live_nodes)}")
    print(f"  replay scans      {len(replay_nodes)}")
    print(f"  live found in replay, node for node   {found} of {len(live_nodes)}")
    print(f"  identical sequence                    {'yes' if identical else 'no'}")
    return found == len(live_nodes) and len(replay_nodes) == len(live_nodes)


def check_decoder(raw_path: str, ans_type: int, scans: list[tuple[int, list[Node]]],
                  label: str) -> bool:
    print(f"\n-- 2. decoder agreement: {label} SDK scans vs Python decode of the same bytes")
    if ans_type not in (ANS_LEGACY, ANS_EXPRESS):
        print(f"  skipped: answer type 0x{ans_type:02x} has no Python decoder")
        return True

    express = ans_type == ANS_EXPRESS
    revs = python_revolutions(raw_path, express)
    index = {tuple(r): i for i, r in enumerate(revs)}
    by_first = {}
    for r in revs:
        by_first.setdefault(r[0], r)

    matched, positions, bad = 0, [], []
    for k, (_, nodes) in enumerate(scans):
        i = index.get(tuple(nodes))
        if i is not None:
            matched += 1
            positions.append(i)
        else:
            near = by_first.get(nodes[0]) if nodes else None
            bad.append((k, first_difference(nodes, near) if near else "no Python revolution starts with this node"))

    skipped = (positions[-1] - positions[0] + 1 - len(positions)) if positions else 0
    print(f"  mode              {'express' if express else 'legacy'}")
    print(f"  python revolutions {len(revs)}")
    print(f"  sdk scans          {len(scans)}")
    print(f"  identical          {matched} of {len(scans)}")
    print(f"  in order           {'yes' if positions == sorted(positions) else 'NO'}")
    print(f"  revolutions the SDK consumer skipped between first and last match  {skipped}")
    for k, why in bad[:5]:
        print(f"    scan {k}: {why}")
    return matched == len(scans) and positions == sorted(positions)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw", help="a .rpraw written by lidar_record")
    args = parser.parse_args()

    stem = args.raw[:-6] if args.raw.endswith(".rpraw") else args.raw
    live_path, replay_path = stem + ".sdkdump", stem + ".replay.sdkdump"

    print("=" * 40)
    print(" SDK session check")
    print("=" * 40)
    print(f"  raw     {args.raw}")

    ok = True
    ans_type, live = read_dump(live_path)
    print(f"  live    {live_path} ({len(live)} scans, answer type 0x{ans_type:02x})")

    if os.path.exists(replay_path):
        _, replay = read_dump(replay_path)
        print(f"  replay  {replay_path} ({len(replay)} scans)")
        ok &= check_replay(live, replay)
    else:
        print(f"  replay  {replay_path} not found -- run lidar_record --replay first")
        ok = False

    ok &= check_decoder(args.raw, ans_type, live, "live")

    print()
    print("  PASS" if ok else "  FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
