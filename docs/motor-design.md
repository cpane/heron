# Motor controller — design and plan

How Heron's motor control is to be organised, built and tested: the
Arduino firmware, the Pi side, the holonomic mixing and the test
platforms. **Agreed plan, nothing built yet** (2026-10-25). The wire format
is in [`motor-protocol.md`](motor-protocol.md).

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

From the Adafruit v1 design the HW-130 copies; **[unverified]** on this
board until the firmware drives each motor in turn.

| Uno pins | Used for | Note |
|---|---|---|
| 0, 1 | USB serial to the Pi | The only hardware UART |
| 11, 3 | PWM, shield motors M1, M2 | Timer 2; AFMotor sets the frequency per motor |
| 6, 5 | PWM, shield motors M3, M4 | Timer 0, about 980 Hz. Timer 0 also runs `millis()`; AFMotor is expected to leave its prescaler alone, so the watchdog's timing holds **[unverified]** |
| 4, 7, 8, 12 | Directions, through the 74HC595 | Clock, enable, data, latch |
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
- **Speed:** 8-bit PWM, so −1000..1000 maps to a duty of 0..255 plus a
  direction.
- **The L293D's limits:** roughly 0.6 A per motor continuous and a drop of
  1.5–2 V **[community: datasheet figures, unverified here]**. The bench
  motors' stall current should stay under that.

## 3. The firmware

### Toolchain: PlatformIO Core, on the Mac

- `platformio.ini` pins the board, compiler, platform and library versions:
  the firmware version of "reproducible from scripts in this repository".
- `pio test -e native` runs the firmware's pure logic on the Mac, as
  `ctest` does for the other libraries.
- Installed from `requirements-host.txt` into the existing `.venv`. The Pi
  build and its container are not touched.
- Considered and not chosen: `arduino-cli` (imposes the sketch layout, no
  host test runner) and avr-gcc driven from our CMake (much work for no
  gain).

### Layers

- **`main.cpp`:** the serial port and the loop; nothing else.
- **A thin driver over AFMotor,** Adafruit's Motor Shield Library v1
  (`AF_DCMotor`, `setSpeed(0..255)`, `run(FORWARD / BACKWARD / RELEASE)`).
  This is the swap point when a platform changes driver (§6).
- **Arduino-free logic,** tested on the Mac: the protocol (shared, §5),
  −1000..1000 to a duty and a direction, the per-index sign, the dead band,
  the ramp and the watchdog. Stop and the watchdog both mean `run(RELEASE)`
  on all four motors.

The dead band and the ramp stay on the Arduino: the Pi sends the values it
wants, and the firmware skips the duty range where a motor only hums and
limits how fast each motor changes, which protects the L293D.

AFMotor is pinned in `platformio.ini`. **Its licence must be checked and
recorded before use**, in a licence table like the remote's
([`remote-ibus-protocol.md`](remote-ibus-protocol.md) §8). The Arduino core
is LGPL: linking it into the firmware is fine, and creates obligations only
if firmware binaries are distributed.

### No C++ standard library in shared code

AVR is the family of 8-bit chips on classic Arduino boards. Their compiler
has no C++ standard library: no `std::array`, `std::span` or exceptions. The
shared protocol code uses `<stdint.h>`, fixed-size buffers and plain
structs. This is an exception to `CPP_CODING_STANDARD.md`'s "standard
library only", written into the standard ("Shared code for the AVR").

### Building and flashing

| Step | Where | How |
|---|---|---|
| Build | Mac | `scripts/build_firmware.sh`, wrapping `pio run` |
| Test | Mac | `pio test -e native`, and `ctest` for the shared protocol |
| Flash at the bench | Mac | `pio run -t upload`, the Arduino on the Mac's USB |
| Flash on the robot | through the Pi | `scripts/flash_firmware.sh <user>@<pi>`: copies the `.hex` and runs `avrdude` on the Pi |

The Pi never builds anything; it only runs `avrdude`. Provisioning gains
`avrdude` in its apt list.

## 4. Holonomic mixing, on the Pi

Decided 2026-10-24. All the motion maths lives in one place and one
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

## 5. Repository layout (planned)

```
firmware/motor/              PlatformIO project
  platformio.ini             board, platform version, libraries; envs: board + native
  src/main.cpp               setup()/loop(): serial, nothing else
  lib/                       motor logic, Arduino-free where possible
  test/                      native tests, run on the Mac
protocol/motor/include/      the wire format: header-only, pure, compiled by both sides (written)
cpp/ ... heron_motor         the Pi side: port + link object, like heron_remote
scripts/build_firmware.sh    wraps pio run
scripts/flash_firmware.sh    copies the .hex to the Pi, runs avrdude there
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
