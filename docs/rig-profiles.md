# Rig profiles — design

How each physical Heron is described and calibrated, so the same code runs
on every one of them. **Agreed 2026-10-25; §8 step 1 (the profile and the
rig name), step 2 (Configure) and step 3 (the calibration procedure)
done.** The
calibration lives on the Pi and is sent to the Arduino at connect (§4);
every Pi has its own name (PR #23); the review points are settled (§9).

## 1. Platforms and rigs

Two levels, which `HERON_PLATFORM` mixes up today:

| | What it is | Decides | Lives in |
|---|---|---|---|
| **Platform** | A hardware design | Which code runs: the firmware build and its motor driver class | Code: a PlatformIO environment, a driver class |
| **Rig** | One physical unit of a platform: its own Pi, Arduino and motors | The numbers: dead band, motor map, signs, ramp, geometry, sensor offsets | Data: a profile, `profiles/<rig>.conf` |

| Platform | Rigs |
|---|---|
| `uno-l293d`: Uno, HW-130 L293D shield, yellow DC motors | `bench` (a small robot with mecanum wheels, since 2026-11-03; first measured with no wheels), `mini` (3D-printed robot, big wheels), a second controller if built |
| Full-size: driver not chosen | the 4-ft robot |

The bench and the mini have identical parts and run identical firmware, but
not identical numbers: wheels and weight raise the duty a motor needs to
start. That is why calibration belongs to the rig, not to the build.

A rig is the whole assembly: each has its own Pi, and an Arduino stays with
its motors.

### Powering the Pi

The Pi feeds the LiDAR (its motor too) and the Arduino from its own USB
5 V, so its supply must hold 5 V under that load. Check with
`vcgencmd get_throttled` on the Pi: `0x0` is good, bit 0 (`0x1`) means
under-voltage now, bit 16 (`0x10000`) that it happened since boot; the
kernel logs each dip (`journalctl -k | grep -i voltage`).

Measured on the bench rig, 2026-11-08 [ours]. The power block is an
Anker power bank, 72 Wh, 87 W (the user's description; its 5 V current
rating has not been checked) [unverified]. The AC supply is the Pi's own
AC adapter.

First, with `GET_INFO` sent from a script:

| Pi supply | `get_throttled` | Under-voltage | LiDAR, GET_INFO answered |
|---|---|---|---|
| The power block | `0x50005` | every few seconds, 19 in the first minutes | 4 of 20 (motor running), 0 of 20 (stopped); the SDK's start-up failed |
| AC | `0x50000` | once, at boot | 20 of 20 (stopped), 11 of 20 (running, requests 50 ms apart); the SDK started every time and recorded cleanly |

Then with `python/probes/lidar_07_requests.py`, which sends 50 requests
with the motor stopped, 50 with it running, and 50 stopped again, and
classifies every reply. Answered, of 50 each:

| Pi supply | LiDAR adapter board | Stopped / running / stopped | `get_throttled` |
|---|---|---|---|
| AC | unit A's | 13 / 27 / 49, then 49 / 28 / 18 (3 garbled) | `0x0` |
| AC, plug re-seated and taped | unit A's | 50 / 42 / 50, then 48 / 35 / 15 | `0x0` |
| The power block | unit A's | 0 / 5 / 2 | 4 dips in the first 3 minutes |
| AC | unit B's | 50 / 50 / 50, twice | `0x0` |
| The power block | unit B's | 49 (1 garbled) / 33 / 50, 50 / 46 / 50, 50 / 30 / 50 | see below |

The head (motor, laser) was unit A's throughout; only the small adapter
board between its cable and the USB lead was swapped. The kernel's USB
log showed no errors in any run, only the disconnect when the plug was
re-seated.

What this shows:

- **Unit A's adapter board is faulty.** With it, the LiDAR lost requests
  on AC too, even with the motor stopped and the plug taped home; with
  unit B's, nothing on AC. Whether it is the board's micro-USB socket or
  something else on it is not established.
- **On the power block, the LiDAR goes silent while its motor runs,**
  even with a good board. In the third run the misses came in two blocks
  of 10, each about 5 s of silence. With the motor stopped it answered.
- **The Pi's under-voltage flag does not catch it.** In the probe's own
  samples, taken during each phase, bit 0 was never set; `0x50005` showed
  once, just after the second run, and the kernel logged 4 dips that
  boot, 3 of them in the first 30 s. That the LiDAR's 5 V, through the
  Pi's USB port, sags further than the Pi's own input is an inference,
  not measured.

So the power block does not run the Pi and the LiDAR. A rig that drives
untethered with the LiDAR needs a 5 V supply that holds under the LiDAR's
motor: one that keeps bit 0 of `get_throttled` clear is not enough, so
check a new supply with the probe, motor running, against the AC rows
above. Which supply that is has not been chosen. The captures are on the
Pi under `~/heron/captures` (`battery_*`, `ac-*`, `unitB-ac-*`, which was
unit B's board on AC, and `boardB-battery-*`).

#### The power block through a buck (2026-11-10)

Why the power block failed: **its own 5 V output measured 4.89 V at
near-zero load** (the user, with a Rigol DHO814) [ours, scope], below the
5 V the Pi's USB ports pass on. The bench rig now takes 12 V from the same
bank and steps it down itself:

```
Anker bank, USB-C -> PD trigger (303PDSink01, SW1 + SW2 on: 12 V;
SW2 alone falls back to 5 V) -> XY-3606 buck, fixed 5.2 V -> USB-A to
micro-USB -> the Pi.  The LiDAR from a Pi USB port, as before.
The drive motors on their own supply.
```

With the LiDAR's motor spinning, under load [ours, scope]:

| Point | Average | Minimum |
|---|---|---|
| Buck output | 5.15 V | |
| The Pi's 5 V rail (GPIO pin 2) | 5.00 V | 4.91 V |
| The LiDAR's 5 V (red wire at its adapter board) | 4.89 V | 4.85 V |

The 0.11 V from the Pi's rail to the LiDAR is the Pi's USB port and the
cable. Requests with `lidar_07_requests.py` (unit B's adapter board, unit
A's head; 50 each, 10 ms apart, 100 ms timeout), answered:

| Runs | Stopped / running / stopped | Scope on the LiDAR's 5 V |
|---|---|---|
| 1, 2 (22:25–22:26) | 46 / 31 / 46, 18 / 50 / 50 | Fell below 3.5 V for 20–30 ms at a time, the Pi's rail flat |
| 3, 4 (22:28–22:29) | 50 / 50 / 50, twice | |
| Soak: 10 runs back to back (22:37:14–22:38:33), the robot driving for the second half | 1499 of 1500: one miss, the first request at a motor start | Never below 4.70 V (a falling-edge trigger), the miss included |

`get_throttled` read `0x0` in every sample, and the kernel logged no USB
errors. Runs 1 and 2 are most likely the hand-held probe shorting the
adjacent 5 V and GND joints at the adapter board: clean once the probe
was secured [the user's inference, not established]. `heron_teleop`'s
log puts the driving at 22:37:49–22:38:14 and 22:38:22 to past the end,
wheels +0.19 to +0.66, `motors OK, resets 0` throughout.

So **through the buck the power block runs the Pi and the LiDAR**,
driving included. The LiDAR's 4.85 to 4.89 V is thin: below the 4.9 V
minimum the user reads in SLAMTEC's A1M8 datasheet [unverified: the
datasheet is not in `docs/lidar-references.md`]. The plan is a dedicated
5 V feed to the LiDAR from the buck's spare output terminal, then the
soak again. If the LiDAR's USB lead stays on the Pi, its VBUS must not
also be tied to the buck's output, or the two supplies are joined; how
unit B's adapter board takes separate power is not checked. Captures:
`buck-pd-1_*` to `-4_*` and `soak-1_*` to `-10_*`.

## 2. The profile file

One per rig, in the repository, so every calibration has a history in git.
The bench's, [`profiles/bench.conf`](../profiles/bench.conf), shortened:

```
# The bench rig: Uno + L293D, four yellow motors, mecanum wheels.
format = 1

[rig]
name = bench
platform = uno-l293d

[motor]
# Protocol motor 1..4 (front-left, front-right, back-right, back-left):
# the shield terminal (1..4 for M1..M4) and the sign.
terminal = 1 2 4 3
sign = + + + +
ramp_per_ms = 4
# Duty where each motor first turns steadily from rest, plus a margin.
min_duty_forward = 76 81 87 92
min_duty_backward = 71 76 76 102

[motor.calibration]
# Free text: how and when the values above were measured.
date = 2026-10-25
method = motor_link sweep, step 20, from rest; +10 margin
supply = AA cells on EXT_PWR
```

- **Format:** `key = value` lines in `[sections]`. `#` starts a comment only
  at the start of a line, so a value may contain one. Simple enough for our
  standard-library-only C++ to read without a parser library, readable in a
  diff. Every rule and key is listed in the reader's header,
  `cpp/include/heron/profile/RigProfile.h`.
- **`[motor.calibration]`** is a free-text record: any keys, all optional.
- **Strict:** an unknown key in a known section, a missing required key or
  a value out of range is an error, with the file and line. A typo must not
  silently leave a default in place. Unknown sections are errors too, so a
  section added by a newer version is not ignored by an older one; `format`
  changes only when an old reader would misread a file.
- **Grows by section.** The motor section comes first. Others are added by
  their own workstreams when they need them: `[odometry]` (the OTOS's linear
  and angular scalars, its mounting offset), `[lidar]` (mounting offset),
  and `[geometry]` (wheel radius, track, wheelbase, maximum speed) when the
  mixing needs it. The bench, now on mecanum wheels, gets one too.
- **Per-motor dead band.** The bench measured 61 to 92 across motors and
  directions. One value for all (step 3's `kMinDuty`, 100) makes the easier
  motors run fast at low commands; eight values let each start at its own
  edge, for eight bytes on the wire.

## 3. Which rig a Pi is

Each Pi holds one line naming its rig, `~/heron/rig` (for example
`bench`), written once by a new script:

```
scripts/set_rig.sh <user>@<pi> bench
```

`deploy_cpp.sh` copies `profiles/` to `~/heron/profiles/` with the
binaries, so a Pi always has the profiles of the code it runs. Programs
load `~/heron/profiles/$(cat ~/heron/rig).conf`, or the file named by a
`--profile` option. No sudo, and the hostname (`heron-<mac>`) stays a
network name only, never written into the repository.

## 4. Getting the numbers to the Arduino: Configure (protocol version 2)

The Arduino holds the motor section in RAM, sent by the Pi after every
Boot. Nothing is stored on the Arduino and nothing rig-specific is compiled
in.

**Why this way** (decided 2026-10-25), against storing it in the Arduino's
EEPROM or compiling one build per rig:

- One copy of the truth, the profile, rather than a repository and an
  EEPROM that can drift apart.
- Recalibrating needs no rebuild or reflash, and a calibration tool can try
  a value live.
- A replaced Arduino works at once; a fresh one has no stale EEPROM.
- The Arduino is useless without its Pi in any case: every command comes
  from it.

The cost is a handshake at connect, after the Boot the Pi waits for anyway
(about 1.5 s), and the protocol changes below. A change of meaning is a new
protocol version, so all of it goes into **version 2** together:

| Change | What |
|---|---|
| **Configure** (new, Pi to Arduino) | The motor section: terminal and sign per motor, ramp per ms, min duty forward and backward per motor. About 15 bytes of payload. Accepted only while stopped |
| **Unconfigured state** | From reset until a Configure: Drive is refused, the motors stay released, and Status says so (a new stop reason, *unconfigured*). Stop and Info work as before. On the 4-ft robot, uncalibrated must mean not moving |
| **Status: configuration checksum** | A CRC of the Configure the firmware holds, 0 if none. The Pi compares it with what it sent, so a lost or mangled Configure is seen rather than assumed |
| **Boot: platform** | The field means the platform (`uno-l293d` = 1, later the full-size one), not the rig. The rig is the Pi's business |

**After any Boot, the Pi configures again.** An unexpected Boot means the
Arduino reset (a brown-out, say) and lost its RAM; until the Pi sends
Configure it will not drive, so a reset mid-drive ends in released motors,
not in motors running on defaults.

`motor_link` gains `--profile` (default: the Pi's rig) and sends Configure
before driving. For calibration it takes overrides, for example
`--min-duty 0` for the sweep, so measuring needs no edits and no reflash.

## 5. On the Pi

A small library, `heron_profile`: reads and validates a profile file into a
plain struct, with ctest tests for the format and its errors. Pure, like
the protocol parsers: no I/O beyond reading the file.

- `heron_motor` (motor step 5) loads the motor section and owns the
  handshake: wait for Boot, send Configure, check the checksum in Status,
  and repeat after any Boot.
- The Pi-side motor settings (geometry, maximum speed) come from the same
  file when the mixing needs them.
- Other workstreams read their own sections when they add them.

## 6. Calibrating a rig

The procedure is [`rig-calibration.md`](rig-calibration.md), one section
per profile section, each saying what to measure, with which tool, and
what to write where. The motor section is what step 3 did by hand:

| # | Measure | How |
|---|---|---|
| 1 | Which terminal each motor is on | `motor_link drive` each motor alone, with an identity map; watch which wheel turns |
| 2 | Its sign | the same runs: forward or backward |
| 3 | Dead band, per motor and direction | `motor_link sweep --min-duty 0`; first steady turn from rest, plus a margin |
| 4 | Ramp | step changes on the floor (the mini, the robot): firm, no jolt, no reset |

A `motor_link calibrate` that walks through these and prints a ready
`[motor]` block was considered and not built: the steps need a person
watching the wheels anyway, and the document is enough until calibrating
becomes routine. The person calibrating commits the profile, with the
record filled in.

A rig is recalibrated when its hardware changes (motors, wheels, driver,
battery type) or when its behaviour drifts (a wheel that no longer starts).

## 7. What changes in the repository

| Now | After |
|---|---|
| `firmware/motor/src/PlatformConfig.h`: the bench's numbers, compiled in | Removed; the numbers move to `profiles/bench.conf` |
| PlatformIO environment `bench`, `HERON_PLATFORM=1` | Environment `uno-l293d`, the platform; one build for the bench and the mini |
| `MotorController(RampConfig)`, `toMotorOutput(value, sign, minDuty)` | Configured at run time from Configure; per-motor, per-direction min duty |
| Protocol version 1 | Version 2, as in §4; `motor-protocol.md` updated with it |

## 8. Order of work

1. **The profile and the rig name: built** (2026-10-25). `heron_profile`
   (`RigProfile.h`) and `rig_profile_test`; `profiles/bench.conf` from the
   step 3 measurements; the `rig_profile` tool (`check`, `show`), with a
   ctest that checks every file in `profiles/`; `set_rig.sh`;
   `deploy_cpp.sh` copies `profiles/`.
2. **Protocol version 2 and the firmware: built** (2026-10-25). Configure,
   the unconfigured state, the checksum; `PlatformConfig.h` removed, the
   environment renamed `uno-l293d`; firmware 0.3.0; `motor_link` configures
   from the profile, with `--profile`, `--min-duty` and `--unconfigured`.
   Checked on the bench, including a reset mid-drive (`RESUME-motor.md`).
3. **The calibration document: written** (2026-10-25),
   [`rig-calibration.md`](rig-calibration.md). `motor_link calibrate` not
   built (§6). Open: how to measure the dead band under load, on a rig
   with wheels on the floor; settled when the mini robot is calibrated.
4. Then motor step 5, `heron_motor`, on top.

## 9. Decisions from review (2026-10-25)

1. **Dead band per motor and direction:** eight values, not one (§2).
2. **Strict files:** unknown keys and sections, missing keys and values
   out of range are errors (§2).
3. **Configure only while stopped** (§4): a calibration tool stops first.
4. **Tracking:** profiles cross workstreams, so this document and the
   profile format are shared, like `cpp-build.md`, and listed in
   `CLAUDE.md`. The motor parts (Configure, the firmware) are motor steps,
   recorded in `RESUME-motor.md`.
