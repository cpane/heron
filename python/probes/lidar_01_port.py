#!/usr/bin/env python3
"""Probe 1 -- port, motor, device identity and health.

What this probe teaches
  * The motor is the DTR line, not a command. You will hear this.
  * The request/response shape for single-response commands, with the seven
    byte descriptor shown field by field.
  * What this specific unit reports for identity, health and sample rate.
  * That RESET replies with an ASCII banner rather than a descriptor.

Questions answered
  * Does the advertised sample rate match reality? (claim only; probe 4 measures)
  * What exactly happens on port open and close, given DTR controls the motor?

Usage
  deploy_pi.sh cpane@<pi> probes/lidar_01_port.py
  deploy_pi.sh cpane@<pi> probes/lidar_01_port.py --reset
  deploy_pi.sh cpane@<pi> probes/lidar_01_port.py --no-motor-test
"""

from __future__ import annotations

import sys
import time

from heron.lidar import cli, render
from heron.lidar.driver import Lidar, LidarError, MotorGuard


def motor_test(lidar: Lidar, transport) -> None:
    print(render.section("Motor control (DTR)"))
    print(
        "  On the A1's USB adapter the motor is driven by the DTR line:\n"
        "      DTR asserted    -> motor stopped\n"
        "      DTR de-asserted -> motor running\n"
        "  Opening the port asserts DTR, so the motor starts out stopped.\n"
        "  There is no PWM speed control on the A1 (SET_MOTOR_PWM is A2/A3).\n"
    )
    print(f"  DTR after open  {transport.dtr}  "
          f"(motor {'running' if transport.motor_running else 'stopped'})")

    print("\n  Listen to the sensor.")
    for label, running, hold in (
        ("stopped", False, 2.0),
        ("RUNNING -- you should hear it spin up", True, 3.0),
        ("stopped", False, 1.5),
    ):
        lidar.set_motor(running)
        print(f"    {time.strftime('%H:%M:%S')}  motor {label}")
        time.sleep(hold)

    print(
        "\n  Note what this means for production: the motor spins whenever DTR\n"
        "  is low, even with no scan running, so a crashed process leaves it\n"
        "  spinning until something closes the port."
    )


def main() -> int:
    parser = cli.base_parser(__doc__)
    parser.add_argument(
        "--no-motor-test",
        action="store_true",
        help="skip the audible motor sequence",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="send RESET and dump the ASCII banner it replies with",
    )
    args = parser.parse_args()

    cli.print_header("Probe 1 -- Port, Motor, Identity, Health")
    print()
    transport = cli.open_transport(args)

    with transport, Lidar(transport) as lidar, MotorGuard(lidar):
        if not args.no_motor_test and hasattr(transport, "dtr"):
            motor_test(lidar, transport)

        print(render.section("STOP (clear any scan left running)"))
        print(f"  tx              {bytes((0xA5, 0x25)).hex()}")
        print("  STOP is not acknowledged, so the only way to know the device")
        print("  has finished is to wait, then flush the input buffer.")
        lidar.stop()
        print("  settled and flushed.")

        for title, call, renderer in (
            ("GET_INFO (0xA5 0x50)", lidar.get_info, render.device_info_table),
            ("GET_HEALTH (0xA5 0x52)", lidar.get_health, render.health_table),
            (
                "GET_SAMPLERATE (0xA5 0x59)",
                lidar.get_samplerate,
                render.samplerate_table,
            ),
        ):
            print(render.section(title))
            try:
                descriptor, payload, decoded = call()
            except LidarError as exc:
                print(f"  ERROR: {exc}")
                continue

            print(render.annotate_descriptor(descriptor))
            print(f"\n  payload     {payload.hex(' ')}")
            print()
            print(renderer(decoded))

        if args.reset:
            print(render.section("RESET (0xA5 0x40)"))
            print(
                "  RESET reboots the MCU. Its reply is an ASCII firmware\n"
                "  banner, NOT a descriptor-framed response -- a read loop\n"
                "  that expects a5 5a wedges here.\n"
                "\n"
                "  Measured on this unit: the 64-byte banner arrives in one\n"
                "  burst 0.671 s after the request, and the device accepts\n"
                "  GET_INFO 4 ms later. So wait for the banner rather than\n"
                "  sleeping a fixed interval -- that is what driver.reset()\n"
                "  does.\n"
            )
            banner = lidar.reset()
            if banner:
                print(f"  {len(banner)} bytes:")
                print(render.hexdump(banner))
                print()
                print("  as text:")
                for line in banner.decode("ascii", "replace").splitlines():
                    if line.strip():
                        print(f"    {line.rstrip()}")
            else:
                print("  (no banner received)")

    print()
    print("  Port closed, motor stopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
