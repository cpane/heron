#!/usr/bin/env python3
"""Inspect and verify a .rpraw capture. Stdlib only -- runs anywhere.

Two jobs:

  1. ``--dump`` walks the capture and prints the chunks and nodes with the
     same annotation probe 2 uses, so a recording can be read after the fact
     with no hardware attached.

  2. ``--verify`` re-decodes the raw bytes through heron.lidar.protocol and
     checks that the result matches the .jsonl written at capture time --
     both sequentially and by seeking to each sample's recorded ``raw_off``.

The second job is the point. It is the acceptance test the C++ is held to as
well: fed the same bytes, it must produce the same numbers (lidar_sdkcheck.py
and lidar_adaptercheck.py do that for the SDK and the C++ library). A parser
that agrees with this one on a real recording is a parser you can trust in the
robot.

Usage
  python3 python/tools/lidar_rawcat.py captures/desk-open_*.rpraw --verify
  python3 python/tools/lidar_rawcat.py captures/desk-open_*.rpraw --dump --nodes 8
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from heron.lidar import express, protocol, recording, render  # noqa: E402

NODE = protocol.LEGACY_NODE_LEN

#: Multi-response descriptors, by scan mode. Everything before one of these is
#: command traffic and single-response payloads; the data stream follows it.
SCAN_MARKERS = {
    "legacy": bytes.fromhex("a55a0500004081"),
    "express": bytes.fromhex("a55a54000040 82".replace(" ", "")),
}


def rx_stream(reader: recording.RawReader) -> tuple[bytes, list[int]]:
    """Concatenate the sensor->host bytes and map each to its file offset.

    The map is needed because the payloads are not contiguous in the file: a
    20-byte chunk header sits between them. A node that straddles a chunk
    boundary therefore occupies two separate stretches of the file, and
    reading five bytes at its offset would pick up a chunk header.
    """
    raw = bytearray()
    offsets: list[int] = []

    for chunk in reader.chunks():
        if chunk.direction != "rx":
            continue
        raw += chunk.data
        offsets.extend(range(chunk.offset, chunk.offset + len(chunk.data)))

    return bytes(raw), offsets


def detect_mode(header: dict, raw: bytes) -> str:
    """Which scan mode this capture holds.

    The header records it, but the raw bytes are the authority -- a capture
    can be inspected without its .jsonl, and decoding express capsules as
    legacy nodes silently produces plausible-looking nonsense (angles above
    360 degrees are the usual tell).
    """
    for mode, marker in SCAN_MARKERS.items():
        if raw.find(marker) >= 0:
            return mode
    return header.get("scan_mode") or "legacy"


def scan_payload(
    reader: recording.RawReader,
) -> tuple[bytes, int, list[int], str]:
    """Return the data bytes, the offset of the first one, the map and mode."""
    raw, offsets = rx_stream(reader)
    mode = detect_mode(reader.header, raw)

    index = raw.find(SCAN_MARKERS.get(mode, SCAN_MARKERS["legacy"]))
    if index < 0:
        return raw, (offsets[0] if offsets else -1), offsets, mode

    start = index + protocol.DESCRIPTOR_LEN
    first = offsets[start] if start < len(offsets) else -1
    return raw[start:], first, offsets[start:], mode


def decode_all_express(data: bytes) -> tuple[list[protocol.Sample], int, int]:
    """Decode the capsule stream, resyncing as needed.

    Mirrors the driver: a capsule's samples are emitted only once the NEXT
    capsule has arrived, because the angles are interpolated between the two
    start angles.
    """
    samples: list[protocol.Sample] = []
    decoder = express.ExpressDecoder()
    position = 0
    discarded = 0
    resyncs = 0
    size = express.CAPSULE_LEN

    while position + size <= len(data):
        if not express.looks_like_capsule(data[position], data[position + 1]):
            offset = express.find_capsule_alignment(data[position:])
            if offset is None or offset == 0:
                position += 1
                discarded += 1
                continue
            position += offset
            discarded += offset
            resyncs += 1
            continue

        try:
            capsule = express.parse_capsule(data[position : position + size])
        except protocol.ProtocolError:
            position += 1
            discarded += 1
            continue

        samples.extend(decoder.push(capsule))
        position += size

    return samples, discarded, resyncs


def matches(decoded: protocol.Sample, expected: dict) -> bool:
    """Compare a decoded sample against a recorded one.

    The .jsonl stores angles rounded to 3 decimals and distances to 2, so the
    comparison rounds the same way rather than using a tolerance. A tolerance
    of exactly half the last digit is ambiguous at the midpoint -- 110.5625
    is written as 110.562, and an ``abs(...) > 0.0005`` test trips on it.
    """
    return (
        round(decoded.angle_deg, 3) == expected["angle_deg"]
        and round(decoded.dist_mm, 2) == expected["dist_mm"]
        and decoded.quality == expected["quality"]
        and decoded.start == expected["start"]
        and decoded.valid == expected["valid"]
    )


def decode_all(data: bytes) -> tuple[list[protocol.Sample], int, int]:
    """Decode the node stream, resyncing as needed."""
    samples: list[protocol.Sample] = []
    position = 0
    discarded = 0
    resyncs = 0

    while position + NODE <= len(data):
        if not protocol.looks_like_node_start(data[position], data[position + 1]):
            offset = protocol.find_node_alignment(data[position:])
            if offset is None or offset == 0:
                position += 1
                discarded += 1
                continue
            position += offset
            discarded += offset
            resyncs += 1
            continue

        samples.append(protocol.decode_legacy_node(data[position : position + NODE]))
        position += NODE

    return samples, discarded, resyncs


def verify(raw_path: str, jsonl_path: str) -> int:
    print(render.banner("Verify: .rpraw re-decode vs .jsonl"))
    print()
    print(f"  raw    {raw_path}")
    print(f"  jsonl  {jsonl_path}")

    with recording.RawReader(raw_path) as reader:
        data, first_offset, offset_map, mode = scan_payload(reader)
        truncated = reader.truncated

    decoder_fn = decode_all_express if mode == "express" else decode_all
    samples, discarded, resyncs = decoder_fn(data)

    reader = recording.JsonlReader(jsonl_path)
    recorded = list(reader.of_type(recording.REC_SAMPLE))

    print(render.section("Sequential re-decode"))
    print(f"  scan mode              {mode}")
    print(f"  bytes of data stream   {len(data)}")
    print(f"  nodes decoded          {len(samples)}")
    print(f"  samples in jsonl       {len(recorded)}")
    print(f"  bytes discarded        {discarded} (resyncs: {resyncs})")
    if truncated:
        print("  NOTE: the raw capture ends mid-chunk (interrupted run).")

    compared = min(len(samples), len(recorded))
    mismatches = []
    for i in range(compared):
        decoded, expected = samples[i], recorded[i]
        if not matches(decoded, expected):
            mismatches.append((i, decoded, expected))

    print(f"  compared               {compared}")
    print(f"  mismatches             {len(mismatches)}")
    for i, decoded, expected in mismatches[:5]:
        print(f"    [{i}] decoded ang={decoded.angle_deg!r} "
              f"dist={decoded.dist_mm!r} q={decoded.quality} "
              f"start={decoded.start} valid={decoded.valid}")
        print(f"         jsonl   ang={expected['angle_deg']!r} "
              f"dist={expected['dist_mm']!r} q={expected['quality']} "
              f"start={expected['start']} valid={expected['valid']}")

    # Independent check: seek to each recorded raw_off and decode there.
    print(render.section("Offset spot-check"))

    if mode != "legacy":
        print("  Skipped: in express mode a sample's raw_off points at the")
        print("  84-byte capsule it came from, not at its own bytes, because")
        print("  32 samples share one capsule. The sequential check above is")
        print("  the meaningful one for this mode.")
        ok = not mismatches and compared > 0
        print()
        print("  PASS -- the raw capture reproduces the decoded output exactly."
              if ok else "  FAIL -- see the mismatches above.")
        return 0 if ok else 1

    # Index the offset map so a recorded raw_off can be turned back into a
    # position in the node stream. Reading the file directly at that offset
    # would be wrong for any node straddling a chunk boundary.
    print("  Each sample records the byte offset of its own node. Seeking to")
    print("  those offsets is an independent test of the whole pipeline.")

    position_of = {offset: i for i, offset in enumerate(offset_map)}

    checked = 0
    offset_bad = 0
    straddling = 0
    step = max(1, len(recorded) // 500)

    for expected in recorded[::step]:
        offset = expected.get("raw_off", -1)
        if offset is None or offset < 0:
            continue
        position = position_of.get(offset)
        if position is None or position + NODE > len(data):
            offset_bad += 1
            continue

        checked += 1
        node = data[position : position + NODE]
        if offset_map[position + NODE - 1] != offset + NODE - 1:
            straddling += 1
        try:
            decoded = protocol.decode_legacy_node(node)
        except protocol.ProtocolError:
            offset_bad += 1
            continue
        if not matches(decoded, expected):
            offset_bad += 1

    print(f"  offsets sampled        {checked} (every {step}th sample)")
    print(f"  offsets wrong          {offset_bad}")
    print(f"  of those, straddling   {straddling} nodes span a chunk boundary")

    ok = not mismatches and not offset_bad and compared > 0
    print()
    if ok:
        print("  PASS -- the raw capture reproduces the decoded output exactly.")
        print("  This file is now a valid regression fixture for a C++ decoder.")
    else:
        print("  FAIL -- see the mismatches above.")
    return 0 if ok else 1


def dump(raw_path: str, nodes: int) -> int:
    print(render.banner("Dump: .rpraw"))
    with recording.RawReader(raw_path) as reader:
        header = reader.header
        print()
        print(f"  capture   {header.get('capture_id')}")
        print(f"  probe     {header.get('probe')}")
        print(f"  device    {header.get('device')}")
        print(f"  host      {(header.get('host') or {}).get('model')}")
        print(f"  throttled {(header.get('host') or {}).get('throttled')}")
        print(f"  notes     {header.get('notes')}")

        print(render.section("First chunks"))
        for i, chunk in enumerate(reader.chunks()):
            if i >= 6:
                break
            print(f"  {chunk.direction} seq={chunk.seq} "
                  f"t={chunk.t_mono_ns} len={len(chunk.data)} @{chunk.offset}")
            print(render.hexdump(chunk.data[:32], base_offset=chunk.offset))

    with recording.RawReader(raw_path) as reader:
        data, first_offset, _, mode = scan_payload(reader)

    if mode != "legacy":
        print(render.section(f"Scan mode: {mode}"))
        print("  Node annotation below is legacy-only; use probe 5 to see")
        print("  annotated express capsules.")
        return 0

    print(render.section(f"First {nodes} nodes"))
    alignment = protocol.find_node_alignment(data) or 0
    for i in range(nodes):
        position = alignment + i * NODE
        if position + NODE > len(data):
            break
        print(render.annotate_node(
            data[position : position + NODE], i, first_offset + position
        ))
        print()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("raw", help="path to a .rpraw capture")
    parser.add_argument("--jsonl", help="matching .jsonl (default: same basename)")
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--dump", action="store_true")
    parser.add_argument("--nodes", type=int, default=6)
    args = parser.parse_args()

    if args.dump or not args.verify:
        dump(args.raw, args.nodes)
        if not args.verify:
            return 0
        print()

    jsonl = args.jsonl
    if not jsonl:
        base = args.raw[:-6] if args.raw.endswith(".rpraw") else args.raw
        for candidate in (base + ".jsonl", base + ".jsonl.gz"):
            if os.path.exists(candidate):
                jsonl = candidate
                break
    if not jsonl:
        print("ERROR: no matching .jsonl found; pass --jsonl explicitly.")
        return 1

    return verify(args.raw, jsonl)


if __name__ == "__main__":
    sys.exit(main())
