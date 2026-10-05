#!/usr/bin/env python3
"""Probe 2 -- the scan stream as raw bytes, annotated.

What this probe teaches
  * The SCAN descriptor, including the 30-bit length / 2-bit mode packing.
  * The five-byte legacy node at the bit level, with arithmetic you can check
    by hand against the hex immediately above it.
  * The two redundant bits (S / !S, and the check bit) that are the only thing
    standing between you and a permanently misaligned stream.
  * That resync works, and what it costs.

Questions answered
  * How is a measurement actually encoded on the wire?
  * How do you recover byte alignment when there is no frame sync word?

Usage
  deploy_pi.sh cpane@<pi> probes/lidar_02_wire.py
  deploy_pi.sh cpane@<pi> probes/lidar_02_wire.py --nodes 24
  deploy_pi.sh cpane@<pi> probes/lidar_02_wire.py --desync 1
"""

from __future__ import annotations

import sys
import time

from heron.lidar import cli, protocol, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard

NODE = protocol.LEGACY_NODE_LEN


def collect(transport, want: int, timeout: float = 10.0) -> bytes:
    """Read ``want`` raw bytes straight off the transport."""
    out = bytearray()
    deadline = time.monotonic() + timeout
    while len(out) < want and time.monotonic() < deadline:
        _, data = transport.read_chunk(want - len(out), 0.5)
        out += data
        if not data and getattr(transport, "exhausted", False):
            break
    return bytes(out)


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument(
        "--nodes", type=int, default=12, help="how many nodes to annotate"
    )
    parser.add_argument(
        "--desync",
        type=int,
        default=0,
        metavar="N",
        help="discard N bytes right after the descriptor to force a resync",
    )
    parser.add_argument(
        "--spinup",
        type=float,
        default=2.0,
        help="seconds to let the motor stabilise before scanning",
    )
    args = parser.parse_args()

    cli.print_header("Probe 2 -- Annotated Wire Format")
    print()
    transport = cli.open_transport(args)

    with transport, Lidar(transport) as lidar, MotorGuard(lidar):
        lidar.stop()

        print(render.section("Starting the motor"))
        print(f"  Spinning up and settling for {args.spinup:.1f} s.")
        lidar.spin_up(args.spinup)
        transport.flush_input()

        print(render.section("SCAN (0xA5 0x20)"))
        print("  tx              a5 20")
        print("  No checksum byte: SCAN carries no payload, and only")
        print("  payload-bearing requests are checksummed.")
        print()

        try:
            descriptor = lidar.start_scan()
        except LidarError as exc:
            print(f"  ERROR: {exc}")
            return 1

        print(render.annotate_descriptor(descriptor))
        print()
        print("  Read bytes 2-5 as a plain uint32 and you get 0x40000005,")
        print("  which looks like a plausible buffer size. That is the trap:")
        print("  the top two bits are the mode, not part of the length.")
        print(f"  mode {descriptor.mode.name} means the {descriptor.length}-byte")
        print("  payload repeats forever until STOP.")

        if args.desync:
            print(render.section(f"Forcing a desync of {args.desync} byte(s)"))
            print("  Discarding bytes immediately after the descriptor, so the")
            print("  buffer below starts mid-node. Watch the search recover.")
            collect(transport, args.desync, timeout=5.0)

        # One contiguous buffer, used for both the hexdump, the alignment
        # search and the decode, so what is annotated is exactly what was
        # dumped. Sized for at least a couple of revolutions (~280 nodes each
        # on this unit) so the summary statistics at the end mean something,
        # even when only a handful of nodes are annotated.
        want = max(
            (args.nodes + protocol.RESYNC_CONFIRM_NODES + 2) * NODE,
            600 * NODE,
        )
        raw = collect(transport, want)
        lidar.stop()

    if len(raw) < NODE:
        print("\n  ERROR: no scan data received.")
        return 1

    print(render.section(f"Raw stream, first {min(len(raw), 80)} bytes"))
    print(render.hexdump(raw[:80]))

    print(render.section("Finding byte alignment"))
    print("  The scan stream has NO frame sync word. The only way to find a")
    print("  node boundary is to look for an offset where several consecutive")
    print("  nodes all satisfy the in-node redundancy:")
    print("      byte0 bit0 (S) must differ from byte0 bit1 (!S)")
    print("      byte1 bit0 (check bit) must be 1")
    print("  That is three constrained bits, so a random byte pair passes")
    print(
        f"  about one time in four. {protocol.RESYNC_CONFIRM_NODES} consecutive "
        "nodes must all pass"
    )
    print("  before alignment is believed.")
    print()

    for offset in range(NODE):
        ok = (
            offset + 2 <= len(raw)
            and protocol.looks_like_node_start(raw[offset], raw[offset + 1])
        )
        print(f"    offset {offset}: single-node check {'passes' if ok else 'fails'}")

    alignment = protocol.find_node_alignment(raw)
    print()
    if alignment is None:
        print("  ERROR: no alignment confirmed; the stream looks corrupt.")
        return 1
    print(f"  confirmed alignment at offset {alignment}")
    if alignment:
        print(f"  -> {alignment} byte(s) discarded to resynchronise")
    else:
        print("  -> already aligned; the device starts nodes right after the")
        print("     descriptor, so this is the healthy case.")

    print(render.section(f"First {args.nodes} nodes, annotated"))
    print("  Every number below comes from the shifts in")
    print("  protocol.decode_legacy_node -- check them against the hex.")
    print()

    shown = 0
    position = alignment
    while shown < args.nodes and position + NODE <= len(raw):
        node = raw[position : position + NODE]
        try:
            sample = protocol.decode_legacy_node(node)
        except protocol.ProtocolError as exc:
            print(f"  node[{shown:04d}] @0x{position:06x}  {node.hex(' ')}  <- {exc}")
            position += 1
            continue

        print(render.annotate_node(node, shown, position))
        if sample.start:
            print("      ^ S flag set: this sample begins a new revolution")
        print()
        shown += 1
        position += NODE

    decoded = []
    position = alignment
    while position + NODE <= len(raw):
        try:
            decoded.append(protocol.decode_legacy_node(raw[position : position + NODE]))
        except protocol.ProtocolError:
            pass
        position += NODE

    valid = [s for s in decoded if s.valid]
    starts = sum(1 for s in decoded if s.start)

    print(render.section("Summary of this buffer"))
    print(f"  bytes captured    {len(raw)}")
    print(f"  nodes decoded     {len(decoded)}")
    print(f"  start flags set   {starts}")
    print(
        f"  valid returns     {len(valid)} "
        f"({100.0 * len(valid) / max(len(decoded), 1):.1f}%)"
    )
    if valid:
        print(
            f"  distance range    {min(s.dist_mm for s in valid):.1f} .. "
            f"{max(s.dist_mm for s in valid):.1f} mm"
        )
    print()
    print("  A high invalid fraction is normal: dist_q2 == 0 means the beam")
    print("  got no return, which in an open room is the common case.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
