#!/usr/bin/env python3
"""Probe 3 -- assembling revolutions, and finding out where zero degrees is.

What this probe teaches
  * How a continuous sample stream becomes discrete 360-degree scans.
  * That samples are NOT uniformly spaced in angle, and how big the gaps get.
  * Whether the S start flag and the angle wrap agree about where a
    revolution begins.
  * Empirically, where the sensor's zero bearing points and which way the
    angle increases. Do not take this from the datasheet.

Questions answered
  * Samples per revolution, and does it vary?
  * Are angles uniformly spaced? What is the largest angular gap?
  * Where is 0 degrees, and does angle increase clockwise or anticlockwise?
  * Does the angle ever go backwards or repeat within a revolution?
  * Is the S flag reliable for segmentation?

Usage
  deploy_pi.sh cpane@<pi> probes/lidar_03_revolution.py
  deploy_pi.sh cpane@<pi> probes/lidar_03_revolution.py --revs 3 --table
  deploy_pi.sh cpane@<pi> probes/lidar_03_revolution.py --bearing-test
"""

from __future__ import annotations

import sys
import time

from heron.lidar import cli, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard
from heron.lidar.scan import RevolutionAssembler, revolutions


def bearing_test(lidar: Lidar, seconds: float) -> None:
    """Continuously report the nearest return, so a target can be placed.

    Put a box on the housing's symmetry axis -- this unit has no cable exit
    or other off-axis landmark -- read the bearing, then swing it 90 degrees
    and watch which way the number moves. That tells you the zero reference
    and the direction of rotation, both of which the production coordinate
    transform depends on.

    Note this reports the nearest *return*, i.e. the closest point on the
    target's surface, not the target's centre. Keep the target closer than
    anything else in the scene or it will not win.
    """
    print(render.section("Bearing test"))
    print("  Place an object near the sensor and move it around.")
    print("  The nearest valid return is reported once per revolution.\n")
    print("    time      bearing    distance   quality")

    assembler = RevolutionAssembler()
    deadline = time.monotonic() + seconds

    for scan in lidar.samples(duration=seconds):
        revolution = assembler.push(scan)
        if revolution is None or revolution.index < 0:
            continue

        valid = [s.sample for s in revolution.samples if s.sample.valid]
        if not valid:
            print("    "
                  f"{time.strftime('%H:%M:%S')}  (no valid returns this rev)")
            continue

        nearest = min(valid, key=lambda s: s.dist_mm)
        print(
            f"    {time.strftime('%H:%M:%S')}  "
            f"{nearest.angle_deg:7.2f} deg  "
            f"{nearest.dist_mm:8.1f} mm  {nearest.quality:3d}"
        )

        if time.monotonic() > deadline:
            break


def print_revolution(revolution, show_table: bool, polar: bool) -> None:
    label = "leading partial" if revolution.index < 0 else f"rev {revolution.index}"
    print(render.section(f"Revolution {revolution.index} ({label})"))

    raw_deltas = revolution.angle_deltas_deg()
    deltas = revolution.angle_deltas_deg(ascending=True)

    print(f"  samples            {revolution.count}")
    print(f"  valid              {revolution.valid_count} "
          f"({100.0 * revolution.valid_count / max(revolution.count, 1):.1f}%)")
    print(f"  duration           {revolution.duration_s * 1000:.1f} ms "
          f"({revolution.rate_hz:.2f} Hz)")

    out_of_order = revolution.out_of_order_count()
    print(f"  arrival order      {'ascending' if revolution.is_monotonic() else 'NOT ascending'}"
          f" -- {out_of_order} of {max(revolution.count - 1, 1)} steps go backwards")
    print(f"  S flag / wrap      "
          f"{'agree' if revolution.wrap_agrees else 'DISAGREE'}")

    if raw_deltas:
        print(f"  delta, raw order   min {min(raw_deltas):7.3f}  "
              f"max {max(raw_deltas):7.3f} deg")
    if deltas:
        ordered = sorted(deltas)
        median = ordered[len(ordered) // 2]
        print(f"  delta, sorted      min {min(deltas):7.3f}  "
              f"median {median:.3f}  max {max(deltas):.3f} deg")
        print(f"  largest true gap   {revolution.max_gap_deg():.2f} deg "
              "(angle-sorted: this is the real coverage)")

    if deltas:
        print("\n  angular delta histogram, angle-sorted (0.5 deg bins):")
        buckets: dict[int, int] = {}
        for delta in deltas:
            buckets[int(delta / 0.5)] = buckets.get(int(delta / 0.5), 0) + 1
        peak = max(buckets.values())
        for bucket in sorted(buckets):
            count = buckets[bucket]
            bar = "#" * max(1, int(40 * count / peak))
            print(f"    {bucket * 0.5:5.1f}-{(bucket + 1) * 0.5:4.1f} deg "
                  f"{count:5d}  {bar}")

    if show_table:
        print("\n  Samples in ARRIVAL order (note it is not angular order):")
        print("\n      i   angle_deg   delta   dist_mm   qual  valid")
        previous = None
        for i, scan in enumerate(revolution.samples):
            sample = scan.sample
            delta = "     -"
            if previous is not None:
                step = sample.angle_deg - previous
                if step < -180.0:
                    step += 360.0
                delta = f"{step:6.3f}"
            previous = sample.angle_deg
            print(
                f"    {i:3d}   {sample.angle_deg:9.3f}  {delta}  "
                f"{sample.dist_mm:8.1f}   {sample.quality:3d}   "
                f"{'yes' if sample.valid else ' - '}"
            )

    if polar:
        print()
        print(render.ascii_polar([s.sample for s in revolution.samples]))


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument("--revs", type=int, default=2,
                        help="complete revolutions to analyse")
    parser.add_argument("--skip", type=int, default=3,
                        help="revolutions to discard first (spin-up transient)")
    parser.add_argument("--table", action="store_true",
                        help="print every sample in the revolution")
    parser.add_argument("--no-polar", action="store_true",
                        help="skip the ASCII polar plot")
    parser.add_argument("--bearing-test", action="store_true",
                        help="continuously report the nearest return instead")
    parser.add_argument("--seconds", type=float, default=20.0,
                        help="bearing-test duration")
    parser.add_argument("--spinup", type=float, default=2.0)
    args = parser.parse_args()

    cli.print_header("Probe 3 -- Revolutions and Bearings")
    print()
    transport = cli.open_transport(args)

    with transport, Lidar(transport) as lidar, MotorGuard(lidar):
        lidar.stop()
        if not args.replay:
            print(f"\n  Spinning up and settling for {args.spinup:.1f} s.")
            lidar.spin_up(args.spinup)
            transport.flush_input()

        try:
            lidar.start_scan()
        except LidarError as exc:
            print(f"  ERROR: {exc}")
            return 1

        if args.bearing_test:
            bearing_test(lidar, args.seconds)
            print("\n  Measured 2026-09-26: 0 deg points out the front, on")
            print("  the housing's symmetry axis, and the bearing increases")
            print("  CLOCKWISE viewed from above. That makes the sensor frame")
            print("  left-handed, so the robot transform negates the angle:")
            print("      bearing_robot = -bearing_sensor + mounting_offset")
            return 0

        collected = []
        stream = lidar.samples(duration=args.seconds)
        for revolution in revolutions(stream, skip=args.skip,
                                      limit=args.revs + 1):
            collected.append(revolution)

        lidar.stop()

    if not collected:
        print("\n  ERROR: no revolutions assembled.")
        return 1

    for revolution in collected[: args.revs + 1]:
        print_revolution(revolution, args.table, not args.no_polar)

    complete = [r for r in collected if r.index >= 0]
    print(render.section("Across these revolutions"))
    if complete:
        counts = [r.count for r in complete]
        print(f"  revolutions        {len(complete)}")
        print(f"  samples/rev        min {min(counts)}  max {max(counts)}  "
              f"mean {sum(counts) / len(counts):.1f}")
        print(f"  segmentation       "
              f"{sum(1 for r in complete if not r.wrap_agrees)} of "
              f"{len(complete)} disagreed between the S flag and angle wrap")
        print(f"  ascending arrival  "
              f"{sum(1 for r in complete if r.is_monotonic())} of "
              f"{len(complete)} revolutions")
        total_ooo = sum(r.out_of_order_count() for r in complete)
        total_steps = sum(max(r.count - 1, 0) for r in complete)
        print(f"  out-of-order       {total_ooo} of {total_steps} steps "
              f"({100.0 * total_ooo / max(total_steps, 1):.1f}%)")
    print()
    print("  Two things to take away:")
    print()
    print("  1. Samples per revolution varies because the sample rate and the")
    print("     rotation rate are independent: the motor is free-running and")
    print("     nothing locks one to the other.")
    print()
    print("  2. The stream is NOT in angular order, and the S flag does not")
    print("     fire at 0 degrees. A revolution must be sorted by angle before")
    print("     it is treated as a 360-degree scan. SLAMTEC's own C++ SDK ships")
    print("     ascendScanData() for exactly this reason -- so the production")
    print("     driver needs the equivalent, not an assumption of ordering.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
