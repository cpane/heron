#!/usr/bin/env python3
"""Probe 5 -- legacy versus express scan, back to back on the same scene.

What this probe teaches
  * That EXPRESS_SCAN is a completely different wire format, not just a faster
    mode: 84-byte capsules of 32 measurements, with interpolated angles.
  * That a capsule cannot be decoded until the NEXT capsule arrives, so the
    decoder is inherently one frame (~21 ms) behind. That is a latency
    property of the format and it changes how you buffer.
  * That EXPRESS_SCAN carries a payload and is therefore checksummed, unlike
    SCAN. Get the checksum wrong and the device silently never replies.
  * What you actually gain: roughly double the angular resolution.

Questions answered
  * Actual sustained sample rate, legacy and express.
  * Does express reduce or increase latency relative to legacy?
  * Is the resolution gain worth the added decoder complexity?

Usage
  deploy_pi.sh cpane@<pi> probes/lidar_05_express.py
  deploy_pi.sh cpane@<pi> probes/lidar_05_express.py --seconds 15
"""

from __future__ import annotations

import sys
import time

from heron.lidar import cli, express, protocol, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard
from heron.lidar.scan import RevolutionAssembler
from heron.lidar.stats import ScanStats


def run_mode(lidar: Lidar, mode: str, seconds: float) -> tuple[ScanStats, float]:
    """Run one scan mode and accumulate statistics. Returns (stats, cpu_s)."""
    lidar.stop()
    lidar.transport.flush_input()

    if mode == "legacy":
        lidar.start_scan()
        stream = lidar.samples(duration=seconds)
    else:
        lidar.start_express_scan()
        stream = lidar.express_samples(duration=seconds)

    stats = ScanStats()
    assembler = RevolutionAssembler()
    cpu_start = time.process_time()

    for scan in stream:
        stats.add_sample(scan.sample, scan.t_mono_ns)
        revolution = assembler.push(scan)
        if revolution is not None:
            stats.add_revolution(revolution)
    partial = assembler.flush()
    if partial is not None:
        stats.add_revolution(partial)

    cpu = time.process_time() - cpu_start
    lidar.stop()
    return stats, cpu


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument("--seconds", type=float, default=15.0,
                        help="duration of EACH mode")
    parser.add_argument("--spinup", type=float, default=2.0)
    parser.add_argument("--capsules", type=int, default=2,
                        help="how many raw capsules to annotate")
    args = parser.parse_args()

    cli.print_header("Probe 5 -- Legacy vs Express Scan")
    print()
    transport = cli.open_transport(args)

    with transport, Lidar(transport) as lidar, MotorGuard(lidar):
        lidar.stop()
        if not args.replay:
            print(f"\n  Spinning up and settling for {args.spinup:.1f} s.")
            lidar.spin_up(args.spinup)
            transport.flush_input()

        print(render.section("EXPRESS_SCAN request framing"))
        frame = protocol.build_request(
            protocol.Command.EXPRESS_SCAN, express.EXPRESS_SCAN_PAYLOAD
        )
        print(f"  tx              {frame.hex(' ')}")
        print("    a5            sync")
        print("    82            EXPRESS_SCAN")
        print("    05            payload length")
        print("    00 x5         working mode 0, then reserved")
        print(f"    {frame[-1]:02x}            checksum = XOR of every preceding byte")
        print()
        print("  SCAN sends just 'a5 20' with NO checksum. Only payload-bearing")
        print("  requests are checksummed, and getting that wrong here is")
        print("  silent: the device simply never answers.")

        print(render.section("Raw capsules"))
        try:
            descriptor = lidar.start_express_scan()
        except LidarError as exc:
            print(f"  ERROR: {exc}")
            return 1
        print(render.annotate_descriptor(descriptor))
        print()

        raw = bytearray()
        deadline = time.monotonic() + 5.0
        want = (args.capsules + 4) * express.CAPSULE_LEN
        while len(raw) < want and time.monotonic() < deadline:
            _, data = transport.read_chunk(want - len(raw), 0.5)
            raw += data

        alignment = express.find_capsule_alignment(bytes(raw)) or 0
        print(f"  capsule alignment found at offset {alignment}")
        print()

        decoder = express.ExpressDecoder()
        for i in range(args.capsules + 1):
            start = alignment + i * express.CAPSULE_LEN
            if start + express.CAPSULE_LEN > len(raw):
                break
            capsule = express.parse_capsule(
                bytes(raw[start : start + express.CAPSULE_LEN])
            )
            emitted = decoder.push(capsule)

            if i >= args.capsules:
                print(f"  capsule[{i}] supplied only to close capsule[{i - 1}] "
                      "-- this is the one-frame delay, visible.")
                print(f"    start angle {capsule.start_angle_q6 / 64.0:7.3f} deg")
                break

            print(f"  capsule[{i}] @0x{start:04x}")
            print(f"    sync/checksum  {raw[start]:02x} {raw[start + 1]:02x}"
                  f"  -> checksum {'OK' if capsule.checksum_ok else 'FAILED'}")
            print(f"    start angle    {capsule.start_angle_q6 / 64.0:7.3f} deg"
                  f"   S flag {capsule.start_flag}")
            print(f"    cabin 0        dist_q2={capsule.measurements[0][0]} "
                  f"({capsule.measurements[0][0] / 4.0:.1f} mm)  "
                  f"offset_q3={capsule.measurements[0][1]}")
            if emitted:
                first, last = emitted[0], emitted[-1]
                print(f"    -> emitted {len(emitted)} samples for the PREVIOUS "
                      f"capsule, {first.angle_deg:.3f} .. {last.angle_deg:.3f} deg")
            else:
                print("    -> emitted nothing: no previous capsule to close yet")
            print()

        print("  The angles above were never transmitted. They are interpolated")
        print("  between two capsules' start angles, which is why a capsule")
        print("  cannot be decoded until the next one lands.")

        print(render.section("A/B comparison"))
        print(f"  Running each mode for {args.seconds:.0f} s on the same scene.")
        print()

        print("  legacy...")
        legacy, legacy_cpu = run_mode(lidar, "legacy", args.seconds)
        print("  express...")
        express_stats, express_cpu = run_mode(lidar, "express", args.seconds)
        decoder_used = getattr(lidar, "express_decoder", None)

    def row(label: str, left: object, right: object) -> None:
        print(f"  {label:<22} {str(left):>14}  {str(right):>14}")

    print()
    row("", "legacy", "express")
    print("  " + "-" * 52)
    row("samples", legacy.samples, express_stats.samples)
    row("samples/s", f"{legacy.sample_rate_hz:.0f}",
        f"{express_stats.sample_rate_hz:.0f}")
    row("revolutions", legacy.revolutions, express_stats.revolutions)
    row("rotation Hz", f"{legacy.rev_rate_hz:.2f}",
        f"{express_stats.rev_rate_hz:.2f}")
    row("samples/rev", f"{legacy.samples_per_rev.mean:.1f}",
        f"{express_stats.samples_per_rev.mean:.1f}")
    row("median delta deg", f"{legacy.angle_delta.percentile(0.5):.2f}",
        f"{express_stats.angle_delta.percentile(0.5):.2f}")
    row("worst gap deg", f"{legacy.max_gap.maximum:.2f}",
        f"{express_stats.max_gap.maximum:.2f}")
    row("invalid %", f"{100 * legacy.invalid_fraction:.1f}",
        f"{100 * express_stats.invalid_fraction:.1f}")
    row("CPU s per 1k samples",
        f"{1000 * legacy_cpu / max(legacy.samples, 1):.4f}",
        f"{1000 * express_cpu / max(express_stats.samples, 1):.4f}")

    if decoder_used is not None:
        print()
        print(f"  express capsules    {decoder_used.capsules}")
        print(f"  checksum failures   {decoder_used.checksum_failures}")

    print(render.section("Reading this"))
    ratio = (express_stats.sample_rate_hz / legacy.sample_rate_hz
             if legacy.sample_rate_hz else 0.0)
    print(f"  Express delivered {ratio:.2f}x the sample rate of legacy.")
    print()
    print("  The cost is not CPU -- it is structure. Express samples are")
    print("  interpolated, not measured, angles; they carry no real quality")
    print("  field; and the decoder is always one capsule behind, which at")
    print("  32 samples per capsule is roughly 21 ms of added latency.")
    print()
    print("  For a robot, that trade is worth making consciously: more angular")
    print("  resolution, slightly staler data, and a harder decoder to port.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
