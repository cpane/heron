#!/usr/bin/env python3
"""Probe 6 -- what the device says about itself, and FORCE_SCAN.

Two commands that the first five probes declared but never exercised.

GET_LIDAR_CONF (0x84, firmware 1.24+)
  A generic key/value query. Ask it for the scan-mode table and the device
  will tell you, per mode, its name, its microseconds-per-sample, its maximum
  range, and which data type it streams. That is a better source than any
  datasheet, because it is what this unit at this firmware actually supports.

FORCE_SCAN (0x21)
  Scans regardless of whether the motor is turning. On this unit, with the
  head stationary, it streams legacy nodes at the full ~1473/s rate. What is
  IN those nodes varies:

    * Usually (10 of 12 observed runs) every node is null -- angle 0.00,
      quality 0, distance 0, one distinct angle across thousands of samples.
    * Occasionally (2 runs) it returns real measurements instead, 100% valid,
      clustered at a near-fixed bearing around 353 deg at roughly 770 mm.

  Settle time is NOT the variable: seven consecutive runs at 2 s and 10 s
  settle all produced nulls. The most likely explanation is simply what the
  parked head happens to be aimed at -- a target in range gives returns, open
  space gives nulls -- but that was not isolated, because it cannot be varied
  without physically moving the sensor. Try it: put an object 30 cm from the
  head and re-run.

  Either way the useful conclusion is the same: FORCE_SCAN confirms the link,
  the command handling and the stream, but a null result does NOT mean the
  optics are broken.

Questions answered
  * What scan modes does this unit support, and what are their real limits?
  * Do the advertised per-mode sample rates match GET_SAMPLERATE?
  * What does the sensor report when it is not spinning?

Usage
  deploy_pi.sh <user>@<pi> probes/lidar_06_capabilities.py
  deploy_pi.sh <user>@<pi> probes/lidar_06_capabilities.py --skip-force-scan
"""

from __future__ import annotations

import sys
import time
from collections import Counter

from heron.lidar import cli, protocol, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard

#: The head coasts for several seconds after a scan. Without this wait,
#: FORCE_SCAN's angle field still sweeps and looks like real rotation.
SETTLE_S = 3.0

ANS_TYPE_NAMES = {
    protocol.DATA_TYPE_LEGACY_SCAN: "legacy 5-byte nodes (0x81)",
    protocol.DATA_TYPE_EXPRESS_SCAN: "express 84-byte capsules (0x82)",
    0x83: "HQ nodes (0x83)",
    0x84: "ultra capsules (0x84)",
}


def show_scan_modes(lidar: Lidar) -> None:
    print(render.section("GET_LIDAR_CONF (0xA5 0x84) -- scan mode table"))

    payload = protocol.build_conf_payload(protocol.ConfType.SCAN_MODE_COUNT)
    frame = protocol.build_request(protocol.Command.GET_LIDAR_CONF, payload)
    print(f"  tx              {frame.hex(' ')}")
    print("    a5 84         sync, GET_LIDAR_CONF")
    print("    04            payload length")
    print("    70 00 00 00   selector SCAN_MODE_COUNT, little-endian uint32")
    print(f"    {frame[-1]:02x}            checksum (0x84 has bit 7 set, so this")
    print("                  command carries a payload and is checksummed)")
    print()

    try:
        count = lidar.scan_mode_count()
    except LidarError as exc:
        print(f"  ERROR: {exc}")
        print("  GET_LIDAR_CONF needs firmware 1.24 or later.")
        return

    try:
        typical = lidar.typical_scan_mode()
    except LidarError:
        typical = -1

    print(f"  scan modes      {count}")
    print(f"  typical mode    {typical}")
    print()
    print("    id  name                us/sample   max range   streams")
    print("    " + "-" * 62)

    for mode in range(count):
        try:
            name = lidar.scan_mode_name(mode)
            us = lidar.scan_mode_us_per_sample(mode)
            max_m = lidar.scan_mode_max_distance_m(mode)
            ans = lidar.scan_mode_ans_type(mode)
        except LidarError as exc:
            print(f"    {mode:2d}  <query failed: {exc}>")
            continue

        marker = " *" if mode == typical else "  "
        print(
            f"    {mode:2d}{marker}{name:<18}  {us:8.2f}    "
            f"{max_m:6.1f} m   {ANS_TYPE_NAMES.get(ans, f'0x{ans:02x}')}"
        )

    if typical >= 0:
        print("\n    * = the mode the device recommends")

    print()
    print("  These come from the device, not a datasheet, so they describe")
    print("  this unit at this firmware. Cross-check them against")
    print("  GET_SAMPLERATE above: the numbers should agree.")

    undecodable = []
    for mode in range(count):
        try:
            ans = lidar.scan_mode_ans_type(mode)
        except LidarError:
            continue
        if ans not in (protocol.DATA_TYPE_LEGACY_SCAN,
                       protocol.DATA_TYPE_EXPRESS_SCAN):
            undecodable.append((mode, ans))

    if undecodable:
        print()
        print("  IMPORTANT: this code implements only the legacy (0x81) and")
        print("  express (0x82) formats. These modes stream something else:")
        for mode, ans in undecodable:
            print(f"    mode {mode} -> 0x{ans:02x} "
                  f"({ANS_TYPE_NAMES.get(ans, 'unknown')})")
        print()
        print("  They were confirmed to stream -- issuing EXPRESS_SCAN with")
        print("  those working_mode values returns a 132-byte multi-response")
        print("  descriptor of type 0x84 -- but nothing here can decode them.")
        if typical in [m for m, _ in undecodable]:
            print()
            print(f"  Note that mode {typical}, which the device RECOMMENDS, is")
            print("  one of them. Express is not this sensor's best mode.")


def force_scan_test(lidar: Lidar, seconds: float, settle: float) -> None:
    print(render.section("FORCE_SCAN (0xA5 0x21) with the motor STOPPED"))
    print("  FORCE_SCAN ignores the motor state. On a stationary head this")
    print("  unit streams at the full rate, but the nodes are usually null")
    print("  (angle 0.00, quality 0, distance 0). Occasionally it returns")
    print("  real measurements from whatever the parked head faces.")
    print()
    print("  Try placing an object ~30 cm from the head and re-running.")
    print()
    print(f"  Letting the head coast to a stop first ({settle:.0f} s) -- without")
    print("  this the angle still sweeps from a previous run's momentum, which")
    print("  looks like rotation but is not.")
    print()

    lidar.stop()
    lidar.set_motor(False)
    time.sleep(settle)
    lidar.transport.flush_input()

    try:
        descriptor = lidar.start_force_scan()
    except LidarError as exc:
        print(f"  ERROR: {exc}")
        return

    print(render.annotate_descriptor(descriptor))
    print()

    samples = [s.sample for s in lidar.samples(duration=seconds)]
    lidar.stop()

    if not samples:
        print("  No data returned with the motor stopped.")
        return

    angles = [s.angle_deg for s in samples]
    valid = [s for s in samples if s.valid]
    spread = max(angles) - min(angles)

    print(f"  samples           {len(samples)} in {seconds:.0f} s "
          f"({len(samples) / seconds:.0f}/s)")
    print(f"  angle range       {min(angles):.2f} .. {max(angles):.2f} deg "
          f"(spread {spread:.2f})")
    print(f"  valid returns     {len(valid)} "
          f"({100.0 * len(valid) / len(samples):.1f}%)")
    if valid:
        distances = [s.dist_mm for s in valid]
        print(f"  distance          {min(distances):.1f} .. "
              f"{max(distances):.1f} mm (mean "
              f"{sum(distances) / len(distances):.1f})")
    print(f"  start flags       {sum(1 for s in samples if s.start)}")

    # Classify on how concentrated the angles are, not on their spread. A
    # stationary head can still emit one stray outlier, and a single sample
    # at 195 degrees makes the spread look like full rotation when 99.9% of
    # the data sits at a single bearing.
    counts = Counter(s.angle_q6 for s in samples)
    dominant_q6, dominant_n = counts.most_common(1)[0]
    share = dominant_n / len(samples)

    print(f"  distinct angles   {len(counts)}")
    print(f"  dominant angle    {dominant_q6 / 64.0:.2f} deg "
          f"({100.0 * share:.1f}% of samples)")
    print()

    if share > 0.95 and not valid:
        print("  All nodes are null and share one bearing. This is the usual")
        print("  result on a stationary head: it confirms the link, the")
        print("  command handling and the stream, but exercises no ranging.")
        print("  A null result here does NOT mean the optics are broken.")
    elif share > 0.95:
        print("  Nodes are concentrated at one bearing and carry real")
        print("  returns -- the parked head is facing something in range.")
        print("  This is the more informative outcome: the optics work too.")
    else:
        print("  The angles are spread out, so the head is still turning.")
        print("  It coasts for several seconds after a scan; increase")
        print("  --settle before drawing conclusions from this test.")


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument("--skip-force-scan", action="store_true")
    parser.add_argument("--force-seconds", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=SETTLE_S,
                        help="seconds to let the head coast to a stop")
    args = parser.parse_args()

    cli.print_header("Probe 6 -- Device Capabilities and FORCE_SCAN")
    print()
    transport = cli.open_transport(args)

    with transport, Lidar(transport) as lidar, MotorGuard(lidar):
        lidar.stop()

        try:
            _, _, rate = lidar.get_samplerate()
            print(render.section("GET_SAMPLERATE, for comparison"))
            print(render.samplerate_table(rate))
        except LidarError as exc:
            print(f"  WARNING: GET_SAMPLERATE failed: {exc}")

        show_scan_modes(lidar)

        if not args.skip_force_scan and not args.replay:
            force_scan_test(lidar, args.force_seconds, args.settle)

    print()
    print("  Motor stopped, port closed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
