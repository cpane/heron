# Motor controller workstream — resume point

Last updated 2026-10-25. Planned; nothing built.

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
| What comes back | Status at 10 Hz, Boot on start and on request |
| Toolchain | PlatformIO Core on the Mac; flash through the Pi with `avrdude` |
| Platforms | Bench, then a 3D-printed mini robot, then the full-size robot (about 4 ft) |

## Still open

1. **Start byte, type codes and checksum,** to choose when the protocol
   code is written.
2. **AFMotor's licence,** to check and record before it is used.
3. **The bench pin table** is from the Adafruit design, unverified on the
   HW-130.
4. **The full-size robot's motor driver**, not chosen.

## Picking up: the first steps

As with the other workstreams, the link comes first: define it, then
measure it on the hardware before building motor logic on top.

1. **`protocol/motor/`:** the shared, header-only protocol, with its
   `ctest` tests on the host. Settle open question 1 there.
2. **`firmware/motor/`:** the PlatformIO project, with the protocol,
   watchdog and Status, but motors released. Check on the bench that the
   link and watchdog behave.
3. **The driver layer:** drive each motor alone, to verify the pin table,
   the order and the sign (protocol §1). Bench rig PWR jumper off first.
4. **`heron_motor`** on the Pi, then the stick mapping and mixing.

The coding standard needs its "no standard library on the AVR" exception
written in when step 1 lands ([`motor-design.md`](motor-design.md) §3).
