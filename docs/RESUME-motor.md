# Motor controller workstream — resume point

Last updated 2026-10-25. Protocol and the step 2 firmware done, and checked on the bench; the driver layer (step 3) is next.

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
| Driver | L293D shield, through Adafruit's Motor Shield Library v1 (AFMotor) |
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

1. **AFMotor's licence,** to check and record before it is used.
2. **The bench pin table** is from the Adafruit design, unverified on the
   HW-130.
3. **The full-size robot's motor driver**, not chosen.

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
3. **The driver layer:** drive each motor alone, to verify the pin table,
   the order and the sign (protocol §1). Bench rig PWR jumper off first.
4. **`heron_motor`** on the Pi, then the stick mapping and mixing.

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
