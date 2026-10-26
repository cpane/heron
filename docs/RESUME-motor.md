# Motor controller workstream — resume point

Last updated 2026-10-25. Steps 1 to 4 done: the protocol, the link firmware, the driver layer and rig profiles, checked on the bench. Step 5, `heron_motor`, built; its bench check with the wheels turning is next.

## Read this first when resuming

Heron's motors are driven by an Arduino over USB serial (`/dev/arduino`):
4 motors for a holonomic (mecanum) drive. The Pi does the holonomic mixing
and sends four wheel values; the Arduino applies them, with a watchdog
that stops the motors if commands stop. Chris writes both sides.

The plan was worked out on a discussion page, 2026-10-19 to 2026-10-25,
and is written down here:

| Document | What |
|---|---|
| [`motor-protocol.md`](motor-protocol.md) | The wire format: Drive, Stop, Info, Status, Boot; the watchdog |
| [`motor-design.md`](motor-design.md) | Hardware, firmware layers and toolchain, the mixing on the Pi, test platforms |

**Hardware on the bench:** a Keyestudio Uno R3-compatible board with an
L293D shield (HW-130) and four two-wire DC motors, no encoders. Its USB
bridge is a CP2102 identical to the LiDAR's, so the udev rules name both
by USB port: Arduino 1-1.2, LiDAR 1-1.4 (`target/udev/`, merged in PR #16,
checked on the Pi).

## Decided

| Question | Answer |
|---|---|
| Board | Keyestudio Uno R3-compatible, ATmega328P |
| Driver | L293D shield, through our own driver, `L293DShield`; not AFMotor (2026-10-25; [`motor-design.md`](motor-design.md) §3 says why) |
| Encoders | None on the bench: open loop |
| What 1.0 means | Full PWM duty, open loop, on every platform; speed control closed on the Pi from the OTOS, later (2026-10-25) |
| Who writes it | Chris, both sides |
| Where the mixing runs | On the Pi; the protocol carries four wheel values |
| Motor order | 1 front-left, 2 front-right, 3 back-right, 4 back-left |
| Sign | Positive drives the robot forward; fixed in wiring or firmware, never on the wire |
| Command rate | 50 Hz, always; watchdog 250 ms |
| Framing | Start byte `0xC5`, type codes with the high bit for Arduino to Pi, CRC-16/CCITT-FALSE (2026-10-25; protocol §3) |
| What comes back | Status at 10 Hz, Boot on start and on request |
| Toolchain | PlatformIO in a container, like the Pi build; flash through the Pi with `avrdude` ([`motor-design.md`](motor-design.md) §3) |
| Baud rate | 115200 |
| Platforms | Bench, then a 3D-printed mini robot, then the full-size robot (about 4 ft) |

## Still open

1. **The full-size robot's motor driver**, not chosen.
2. **Does the shield twitch a motor during the 1.5 s reset,** while the
   pins float? Not observed either way during step 3.

AFMotor's licence, once open here, was checked 2026-10-25: public domain.
It is not used ([`motor-design.md`](motor-design.md) §3).

## Picking up: the first steps

As with the other workstreams, the link comes first: define it, then
measure it on the hardware before building motor logic on top.

1. **`protocol/motor/`: done** (2026-10-25). The shared, header-only
   protocol and its frame parser, tested by `motor_protocol_test` on the
   host and built warning-free by Clang and GCC 14. Protocol §8.
2. **`firmware/motor/`: done** (2026-10-25). The protocol, watchdog,
   Boot and Status; no motor is driven. `MotorController` passes
   `motor_controller_test`; the firmware builds with avr-gcc (303 of 2048
   bytes of RAM, 3.5 KB of flash) and passed the bench checklist below.
3. **The driver layer: done** (2026-10-25). Our own driver for the shield
   (`L293DShield`), the ramp (4 units per ms) and the dead band and sign
   mapping (`MotorOutput`, tested by `motor_controller_test`), and
   `motor_link sweep`. Firmware 0.2.0: 339 bytes of RAM, 4.9 KB of flash.
   Passed the step 3 checklist below; the bench's terminal map and dead
   band are in `PlatformConfig.h`.
4. **Rig profiles first** (agreed 2026-10-25,
   [`rig-profiles.md`](rig-profiles.md)): the bench's numbers move from
   `PlatformConfig.h` into `profiles/bench.conf`, sent to the Arduino by a
   Configure message after every Boot, with a dead band per motor and
   direction. Its §8 has the order of work. **Part 1, the profile and the
   rig name, built** (2026-10-25): `heron_profile`, `profiles/bench.conf`,
   `rig_profile`, `set_rig.sh`. **Part 2, Configure (protocol version 2),
   built** (2026-10-25): the firmware (0.3.0) starts unconfigured and
   refuses Drive until the Pi sends the profile's motor section; Status
   reports its checksum; `PlatformConfig.h` is gone and the build is per
   platform (`uno-l293d`). `motor_link` configures from the profile, and
   again after a mid-run Boot. Passed its bench checklist below, including
   a reset mid-drive. **Part 3, the calibration procedure, written**
   (2026-10-25): [`rig-calibration.md`](rig-calibration.md), the bench's
   calibration as steps for the next rig. Open: measuring the dead band
   under load, settled when the mini robot is calibrated.
5. **`heron_motor`: built** (2026-10-25):
   [`motor-cpp-design.md`](motor-cpp-design.md). `motors_test` (45 checks,
   against a fake board running the firmware's logic) passes on the host
   and the Pi; `motor_adapter` started it on the bench rig in 1627 ms,
   250 polls all OK. `Motors`: `start()` blocks
   until the Arduino is configured; the robot's loop calls `drive()` every
   20 ms (no heartbeat thread); a reader thread publishes Status and
   configures again after a reset. Then the stick mapping and mixing.

## Step 2 bench checklist

The Uno on the Pi in USB port 1-1.2, no motors needed. Once, after this
branch: re-run provisioning for `avrdude`, then build and flash.

```
scripts/provision_pi.sh   <user>@<pi>          # adds avrdude
scripts/build_firmware.sh
scripts/flash_firmware.sh <user>@<pi>
scripts/build_cpp.sh
```

| # | Check | Command | Expect |
|---|---|---|---|
| 1 | Boot after opening the port, and on request | `scripts/deploy_cpp.sh <user>@<pi> motor_link info` | Boot within a second or two; firmware 0.1.0, platform bench. Record the delay |
| 2 | Drive at 50 Hz | `... motor_link drive 100 -200 300 -400 --seconds 3` | Status every 100 ms: running, the sequence counting, target and applied as sent, crc, ver and disc at 0 |
| 3 | Stop | (the end of 2) | stopped, reason stop, wheels 0 |
| 4 | Watchdog | `... motor_link watchdog` | reported 250–350 ms after the last Drive; wd counts 1 |
| 5 | Noise | `... motor_link noise` | PASS: errors counted, the link kept running |
| 6 | Loop time and free RAM | the Status and Boot above | Record them |

Results, 2026-10-25, on the bench Uno through the Pi **[ours]**: all six
pass.

| # | Measured |
|---|---|
| 1 | Boot 1519 and 1521 ms after opening (two runs); firmware 0.1.0, protocol 1, platform bench. InfoRequest answered within 4 ms |
| 2 | `drive 500 -500 1000 -1000`: Status every 100 ms, target and applied as sent, crc, ver and disc 0 |
| 3 | stopped, reason stop, wheels 0 |
| 4 | Watchdog stop reported 303 ms after the last Drive; wd 1 |
| 5 | PASS: crc +10, disc +200 for 10 injections; all 100 good Drives received |
| 6 | Longest loop 572–620 µs (the loop that sends a Status); free RAM 1507 bytes |

Open questions the checklist answers:

- **Does opening the port reset the Uno?** Yes **[ours]**: Boot's uptime
  starts at 0, and Boot arrives about 1.5 s after opening, most of it the
  bootloader waiting for a new program. A program must wait for Boot
  before driving.
- **Does the reset cause survive the bootloader?** No **[ours]**: Boot
  reports no cause after the reset on opening, so the bootloader clears
  the flags first. The reset-cause field is always 0 with this
  bootloader, so a brown-out cannot be seen through it. Whether the
  bootloader hands the flags on some other way (later Optiboot versions
  pass them in a register) is **[unverified]**; until then, a brown-out
  shows only as an unexpected Boot.

## Step 3 bench checklist

**Wheels off the ground; the shield's PWR jumper off; the motor battery on
EXT_PWR.** Build and flash 0.2.0 as for step 2, then, with
`scripts/deploy_cpp.sh <user>@<pi> motor_link ...`:

| # | Check | Command | Expect, and record |
|---|---|---|---|
| 1 | Nothing moves at start-up | flash, then `motor_link info` | No twitch during the 1.5 s reset or after; firmware 0.2.0 |
| 2 | Which wheel each motor turns, and which way | `drive 600 0 0 0 --seconds 2`, then `0 600 0 0`, `0 0 600 0`, `0 0 0 600` | Record wheel and direction for each. Should be 1 front-left, 2 front-right, 3 back-right, 4 back-left, all forward |
| 3 | Fix the map | edit `kMotorTerminal` and `kMotorSign` in `PlatformConfig.h`, rebuild, flash, repeat 2 | All four right |
| 4 | Dead band | `sweep 1 --step 20`, and each other motor | The value where each starts to turn; the slowest one, as a duty, is `kMinDuty` |
| 5 | Ramp | `drive 1000 1000 1000 1000 --seconds 2` | Status shows applied climbing over about 250 ms; no unexpected Boot (a jolt that reset the Uno) |
| 6 | Stop and watchdog with motors turning | Ctrl-C during a drive; `watchdog 600 600 600 600` | Wheels coast to a stop at once, not on the ramp; wd counts 1 |
| 7 | Loop time and free RAM | the Status and Boot above | Record them |

Results, 2026-10-25, on the bench rig through the Pi, AA cells on
EXT_PWR **[ours]**:

| # | Measured |
|---|---|
| 1 | Firmware 0.2.0, Boot 1519 ms after opening. A twitch during the reset was not watched for (open item 2) |
| 2 | Each terminal drives one motor, forward for a positive value: the HW-130 matches the Adafruit pin table. Wired: front-left M1, front-right M2, back-left M3, back-right M4 |
| 3 | `kMotorTerminal` {M1, M2, M4, M3}, all signs +1; the re-run turned front-left, front-right, back-right, back-left, all forward |
| 4 | First steady turn from rest, as a duty, forward / backward: motor 1 66 / 61, motor 2 71 / 66, motor 3 77 / 66, motor 4 82 / 92. `kMinDuty` 100; at value 1 all four turn |
| 5 | 0 to 1000: applied 368, 768, 1000 at 100 ms intervals; a smooth spin-up, no reset |
| 6 | `watchdog 600 600 600 600`: stop reported 303 ms after the last Drive, applied straight to 0, wd 1; the wheels coasted to a stop |
| 7 | Longest loop about 810 µs while driving, 620 µs stopped (the duty arithmetic: 32-bit division, four times a pass); free RAM 1461 bytes |

## Step 4 part 2 bench checklist: Configure

Firmware 0.3.0 flashed; the Pi set to rig `bench` (`set_rig.sh`), profiles
deployed with `deploy_cpp.sh`. Wheels off the ground, PWR jumper off, motor
battery on EXT_PWR for checks 4 to 6.

| # | Check | Command | Expect |
|---|---|---|---|
| 1 | Boot | `motor_link info` | firmware 0.3.0, protocol 2, platform uno-l293d; Status stopped, *unconfig*, cfg 0000 |
| 2 | Unconfigured refuses Drive | `motor_link drive 600 600 600 600 --unconfigured` | No wheel moves; target stays 0; still *unconfig* after the Stop |
| 3 | Configure takes | `motor_link drive 0 0 0 0` | "Configured: checksum ... confirmed"; Status cfg the same |
| 4 | Configured drives as in step 3 | `motor_link drive 600 0 0 0`, then each other motor | Front-left, front-right, back-right, back-left, all forward |
| 5 | Per-motor dead band | `motor_link drive 1 1 1 1`, then `-1 -1 -1 -1` | All four turn slowly both ways |
| 6 | A reset mid-drive releases the motors | `motor_link drive 600 600 600 600 --seconds 10`; press the Uno's reset button midway | Wheels stop at the reset; "Boot mid-run ... Configuring again", then they run again |
| 7 | A bad profile | `motor_link drive 0 0 0 0 --profile /nonexistent.conf` | An error before the port opens; the Uno not reset |

Results, 2026-10-25 **[ours]**: all seven pass.

| # | Measured |
|---|---|
| 1 | firmware 0.3.0, protocol 2, platform uno-l293d, Boot 1522 ms after opening; *unconfig*, cfg 0000 |
| 2 | 50 Drives refused: target 0, applied 0, *unconfig* throughout and after the Stop |
| 3 | Checksum 3B1B confirmed; `noise` also passes when configured |
| 4 | Front-left, front-right, back-right, back-left, all forward: the terminal map now comes from `profiles/bench.conf` |
| 5 | All four turn slowly at 1 and at −1: the per-motor, per-direction dead band |
| 6 | Two presses of the shield's reset during a 600 drive. Each time: Boot with *unconfig*, checksum 0000, applied 0; `motor_link` configured again 9 and 17 ms after the Boot; running again within 100 ms, ramping up. Status went quiet for 2.4 s and 1.1 s (the bootloader's wait runs from the button's release). Drives sent into the bootloader did no harm: no checksum or framing errors after |
| 7 | `cannot read /nonexistent.conf`, before the port opens |
