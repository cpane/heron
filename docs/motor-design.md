# Motor controller — design and plan

How Heron's motor control is to be organised, built and tested: the
Arduino firmware, the Pi side, the holonomic mixing and the test
platforms. Steps 1 to 3 (protocol, link firmware, driver layer) are
built and checked on the bench (2026-10-25). The wire format is in
[`motor-protocol.md`](motor-protocol.md).

## 1. The pieces

| Side | What | Written by |
|---|---|---|
| Pi | Stick mapping, the safety arbiter, the holonomic mixing, `heron_motor` (the serial link) | Chris |
| Wire | [`motor-protocol.md`](motor-protocol.md): four wheel values, Status, Boot | — |
| Arduino | Protocol parser, watchdog, dead band and ramp, the motor driver | Chris |

## 2. Hardware: the bench rig

| Part | Detail |
|---|---|
| Board | Keyestudio Uno R3-compatible, ATmega328P: 16 MHz, 32 KB flash, 2 KB RAM. One hardware serial port, which is the Pi link, so debug output travels in Status |
| USB bridge | Silicon Labs CP2102, 10c4:ea60, serial `0001`: identical to the LiDAR's adapter, so `/dev/arduino` is named by USB port, 1-1.2 (`target/udev/`) |
| Motor driver | L293D Motor Driver Shield (HW-130), a clone of Adafruit's original Motor Shield: two L293D chips for 4 DC motors, directions through a 74HC595 shift register |
| Motors | Plain two-wire DC motors, **no encoders**: the bench runs open loop |

### Pins the shield takes

From the Adafruit v1 design the HW-130 copies, as its library (AFMotor)
wires it; **checked on the bench** 2026-10-25: each terminal drives its
own motor, the right way. The full table, with the shift register's bits, is in
`firmware/motor/src/L293DShield.h`.

| Uno pins | Used for | Note |
|---|---|---|
| 0, 1 | USB serial to the Pi | The only hardware UART |
| 11, 3 | PWM, shield motors M1, M2 | Timer 2, about 490 Hz (the Arduino core's setting, through `analogWrite()`) |
| 6, 5 | PWM, shield motors M3, M4 | Timer 0, about 980 Hz. Timer 0 also runs `millis()`; `analogWrite()` leaves its prescaler alone, so the watchdog's timing holds |
| 4, 7, 8, 12 | Directions, through the 74HC595 | Clock, output enable (active low), data, latch |
| 9, 10 | Servo headers | Free if no servos are used |
| A0 | Motor battery voltage | A resistor divider from EXT_PWR, scaled so full battery voltage stays under 5 V |
| 2, 13, A1–A5 | Free | Pin 2 is the one interrupt pin left |

### Notes

- **The PWR jumper must be off.** With it on, the shield's motor supply is
  joined to the Uno's Vin, and USB 5 V leaks backwards through the Uno's
  regulator into it: the motors turn weakly from USB alone. On the robot
  that USB is the Pi's, already marginal with the LiDAR spinning
  ([`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md), power). Jumper
  off: the Pi powers only the Uno, and the motors run from their battery
  on EXT_PWR.
- **Speed:** 8-bit PWM, so −1000..1000 maps to a duty plus a direction:
  1..1000 to the dead band's minimum..255, and 0 to released
  (`MotorOutput.h`).
- **The L293D's limits:** roughly 0.6 A per motor continuous and a drop of
  1.5–2 V **[community: datasheet figures, unverified here]**. The bench
  motors' stall current should stay under that.

## 3. The firmware

### Toolchain: PlatformIO Core, in a container

Decided 2026-10-25, to mirror the Pi build: the compiler for a target lives
in a container, never on the host, so a Mac or Linux developer needs only
Docker.

| | Pi C++ | Arduino firmware |
|---|---|---|
| Compiler | `heron-build:trixie` (`docker/Dockerfile.build`) | `heron-firmware:trixie` (`docker/Dockerfile.firmware`) |
| Build | `scripts/build_cpp.sh` | `scripts/build_firmware.sh` |
| Versions pinned in | the image (Debian trixie, as on the Pi) | the image (PlatformIO 6.1.19) and `platformio.ini` (atmelavr 5.3.0, Arduino core 5.4.0, avr-gcc 7.3.0) |
| Tests | `ctest` | `ctest`: the firmware's logic has no Arduino code, so CMake builds it for the host too |
| Onto the target | `scripts/deploy_cpp.sh`: rsync, run | `scripts/flash_firmware.sh`: rsync the `.hex`, run `avrdude` on the Pi |
| On the Pi | nothing but provisioning | `avrdude`, from provisioning |

- **PlatformIO's downloads** (the compiler and the Arduino core, about
  290 MB) go to `build-firmware/platformio/` in the repository, git-ignored
  like `build-pi/`, so they are fetched once. The firmware lands in
  `build-firmware/motor/<platform>/firmware.hex`.
- **The container runs at the host's own architecture.** The output is AVR
  code either way, and PlatformIO's AVR compiler runs natively in an arm64
  container (checked on an Apple Silicon Mac, 2026-10-25).
- **The Arduino stays plugged into the Pi,** on the bench as on the robot,
  so there is one way to flash and one way to talk to it. Docker Desktop on
  macOS cannot reach USB devices anyway.
- Considered and not chosen: PlatformIO installed on the host (a second kind
  of host setup); `arduino-cli` (imposes the sketch layout); avr-gcc driven
  from our CMake (much work for no gain).

### Layers

- **`main.cpp`:** the serial port, the clock, and what only the chip knows
  (reset cause, free RAM, loop time). Nothing else.
- **Our own driver, `L293DShield`** in `src/`: `begin()`, `set(terminal,
  duty and direction)`, `apply()`. The only code that knows the shield's
  wiring, and the swap point when a platform changes driver (§6): another
  driver is another class with the same three calls.
- **No rig settings in the firmware.** The ramp rate, the dead band per
  motor and direction, and each motor's terminal and direction come from
  the rig's profile on the Pi, in a Configure after every Boot
  ([`rig-profiles.md`](rig-profiles.md), protocol §4). One build per
  platform (`uno-l293d`) serves every rig of it.
- **Arduino-free logic** in `lib/`, tested by `ctest`: `MotorController`
  (what each command does, the watchdog, the Status schedule, the ramp)
  and `MotorOutput` (a value to a duty and a direction, with the direction
  flip and the dead band); also Configure: held only while stopped, Drive
  refused until one arrives. Stop and the watchdog release all four motors at once,
  without the ramp.

The dead band and the ramp stay on the Arduino: the Pi sends the values it
wants, and the firmware skips the duty range where a motor only hums and
limits how fast each motor changes, which protects the L293D.

**Why our own driver, not AFMotor** (decided 2026-10-25). AFMotor,
Adafruit's library for the v1 shield, was the first plan. Checked
2026-10-25: its files say "this code is public domain", so its licence is
no obstacle, but the repository is archived and its README calls it
deprecated, with the hardware discontinued. It knows only this shield, so
it would not carry to the full-size robot's driver; the growth happens
behind our own driver interface either way. Our driver is about 100 lines,
keeps the timers under our control, and the wiring it needs is checked on
the bench regardless. AFMotor does leave Timer 0's prescaler alone (the
line that would change it is commented out), so that was not the reason.

The Arduino core is LGPL: linking it into the firmware is fine, and
creates obligations only if firmware binaries are distributed.

### No C++ standard library in shared code

AVR is the family of 8-bit chips on classic Arduino boards. Their compiler
has no C++ standard library: no `std::array`, `std::span` or exceptions. The
shared protocol code uses `<stdint.h>`, fixed-size buffers and plain
structs. This is an exception to `CPP_CODING_STANDARD.md`'s "standard
library only", written into the standard ("Shared code for the AVR").

### Building and flashing

| Step | Where | How |
|---|---|---|
| Build | host, in a container | `scripts/build_firmware.sh` |
| Test | host | `ctest --test-dir build-host` (`motor_protocol_test`, `motor_controller_test`) |
| Flash | through the Pi | `scripts/flash_firmware.sh <user>@<pi>` |
| Talk to it | the Pi | `scripts/deploy_cpp.sh <user>@<pi> motor_link <command>` |

The Pi never builds anything; it only runs `avrdude` and `motor_link`.

## 4. Holonomic mixing, on the Pi

Decided 2026-10-24; built 2026-10-25 as `heron_drive`
(`cpp/include/heron/drive/Drive.h`: `sticksToVelocity()`, `mix()`,
`ArmGate`) and run by `heron_teleop`, the first driving program. All the motion maths lives in one place and one
language, tested on the Mac with `ctest` rather than reflashed. The OTOS
feedback that will close the loop is on the Pi, and the correction has to
happen before the mixing. Per-wheel trims change without a reflash.

### Each 20 ms

| Step | What |
|---|---|
| Read the remote | `RemoteControl::getLatest()`. If `isOk()` is false (stale, disarmed or radio lost), send Stop and skip the rest |
| Sticks to body velocity | `normalizeAxis()`, then multiply by the throttle (CH3, 0 to 1) as a speed limit. vx forward, vy left, wz anticlockwise, each −1 to 1 |
| Safety arbiter | Phase 1: pass-through. Phase 2, SwC on Assisted: limits only the part of the command that moves toward an obstacle, from the LiDAR scan and the OTOS's measured speed |
| Mix | The formulas below, then scale all four by the largest if any exceeds 1, so the direction is kept |
| Send | `heron_motor`: Drive, four values × 1000, rounded and clamped |
| Apply (Arduino) | Check the frame, reset the watchdog, convert, ramp, drive the shield |

Settled when it was built (2026-10-25):

- **Arming needs the throttle at the bottom.** The throttle stick does not
  spring back, so after SwA arms nothing moves until the throttle has been
  seen at or below 5%; disarming, radio loss or a stale link closes it
  again (`ArmGate`).
- **Dead zone:** 30 counts around each stick's centre, 3% of its travel.
- **Turn gain 1.0** until tuned on the mini robot.
- These are constants in `StickConfig` for now; the turn gain moves to the
  rig profile with the mini's geometry.

### Stick mapping

| Control | Channel | Gives |
|---|---|---|
| Right stick up and down | CH2 | vx = +stick (up is forward) |
| Right stick left and right | CH1 | vy = −stick (stick right is positive; REP 103 y is left) |
| Left stick left and right | CH4 | wz = −stick (stick right turns clockwise; REP 103 positive is anticlockwise) |
| Left stick up and down | CH3 | Speed limit, 0 to 1; the stick does not spring back |
| SwA | CH7 | Arm. Disarmed or radio lost: Stop |
| SwC | CH9 | Mode: Manual, Assisted, Auto later. Every position is Manual in Phase 1 |

Channel numbers are those measured in
[`remote-ibus-protocol.md`](remote-ibus-protocol.md) §4.

### The formulas, in protocol order

```
// vx forward, vy left, wz anticlockwise: REP 103, each -1..1
m1 = vx - vy - wz;   // front-left
m2 = vx + vy + wz;   // front-right
m3 = vx - vy + wz;   // back-right
m4 = vx + vy - wz;   // back-left
// if the largest |m| exceeds 1, divide all four by it
```

- **Assumes mecanum wheels whose rollers form an X seen from above,** the
  standard layout. Mounted as an O, the robot barely rotates or strafes;
  swap the left and right wheels.
- **Check each move on the bench:** forward, all four positive; strafe
  left, m1 and m3 negative, m2 and m4 positive; anticlockwise, m1 and m4
  negative, m2 and m3 positive. A wrong sign on vy or wz mirrors the robot
  without any error, the same failure the LiDAR's frame conversion guards
  against.

### Speed control: open loop first, then closed on the Pi

Decided 2026-10-25. On the wire and in the firmware, 1.0 is full PWM duty
on every platform ([`motor-protocol.md`](motor-protocol.md) §1); the
Arduino never holds a speed.

- **Phase 1, remote driving, runs open loop:** stick × throttle gives a
  body velocity as a fraction of full drive, which is mixed into wheel
  values.
- **Later, once the mini robot drives,** the loop is closed on the Pi:
  the commanded body velocity (vx, vy in m/s, wz in rad/s, REP 103) is
  compared with the OTOS's measured velocity, and the command is corrected
  before the mixing. No wheel encoders are needed.
- **The protocol does not change,** only the meaning of the Pi's inputs:
  today a fraction of full drive, later a real velocity. Turning m/s into
  wheel values uses the per-platform configuration (§6): wheel radius,
  track, wheelbase and maximum speed.

Why: the same 0.5 is a different speed on the bench, the mini and the
full-size robot, and changes with battery and load. A loop on the Pi,
from the OTOS, makes 0.3 m/s mean 0.3 m/s on every platform, keeps the
firmware identical across them, and keeps all the motion maths on the Pi.

## 5. Repository layout

```
protocol/motor/include/      the wire format: header-only, compiled by both sides
firmware/motor/              the PlatformIO project
  platformio.ini             pinned versions; one environment per platform
  src/main.cpp               setup()/loop(): serial, clock, Boot and Status
  src/L293DShield.*          the bench shield's driver
  lib/MotorController/       the logic, no Arduino code; also built by CMake:
                             MotorController, MotorOutput
cpp/include/heron/motor/     MotorPort: the Pi's serial port to the Arduino
cpp/tools/MotorLink.cpp      motor_link: the bench tool
cpp/tests/Motor*Test.cpp     ctest: the protocol and the controller
docker/Dockerfile.firmware   the AVR toolchain
scripts/build_firmware.sh    pio run, in the container
scripts/flash_firmware.sh    copies the .hex to the Pi, runs avrdude there
(step 5) heron_motor         the Pi's motor library, like heron_remote
                             (motor-cpp-design.md)
```

- **`protocol/motor/`** is written once and compiled by both sides: CMake
  includes it for the Pi, `platformio.ini` for the Arduino, so the two ends
  cannot drift apart. Pure and fed bytes, like the iBUS parser, and tested
  by `ctest`.
- **`heron_motor`** on the Pi is shaped like `RemoteControl`: a thread, the
  newest Status, and a send call. Robot code never sees the wire format.

## 6. Test platforms

Three pieces of hardware, tested in order. The mini and the full-size
robot have the same proportions, so the same layout and the same mixing;
size, mass, motors and driver differ.

| Platform | What it proves | Hardware |
|---|---|---|
| Bench | Motor control alone: protocol, motor order and sign, watchdog, ramp, dead band. No movement on the floor | Uno and L293D shield, small DC motors |
| Mini robot | The whole robot on the floor at small scale: remote driving, mixing on real mecanum wheels, then Phase 2 with the LiDAR and odometry | 3D-printed, a scaled copy of the full-size robot |
| Full-size robot | The same software at about 4 feet tall | Larger motors and a driver to match; to be chosen |

**The same on all three:** the protocol, the Pi code (only configuration
differs) and the firmware's shape; only its driver layer changes with the
driver.

**Per-platform configuration:**

| Setting | Why it differs | Side |
|---|---|---|
| Wheel radius, track and wheelbase | Needed to turn m/s and rad/s into wheel values once the loop is closed (§4); the rotation term grows with size | Pi |
| Maximum speed and acceleration | A 4-foot robot is heavier and slower to stop | Pi |
| Stop and slow zones | Stopping distance grows with speed and mass | Pi |
| LiDAR and OTOS mounting offsets | Different positions on each body | Pi |
| Dead band and ramp rate | Different motors start at different duties | Arduino |
| Motor driver | See below | Arduino |

**The full-size robot outgrows the L293D.** About 0.6 A per channel and a
1.5–2 V drop are fine on the bench and probably the mini, but not for
motors that move a 4-foot robot. It needs a driver rated for its motors'
stall current, and possibly encoders. The firmware's driver layer and ramp
change; the protocol does not. A bigger driver may need more pins or a
different board, which the protocol already allows for.
