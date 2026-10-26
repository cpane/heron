# Rig profiles — design

How each physical Heron is described and calibrated, so the same code runs
on every one of them. **Agreed 2026-10-25; §8 step 1 (the profile and the
rig name) built, the rest not yet.** The
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
| `uno-l293d`: Uno, HW-130 L293D shield, yellow DC motors | `bench` (no wheels, unloaded), `mini` (3D-printed robot, big wheels), a second controller if built |
| Full-size: driver not chosen | the 4-ft robot |

The bench and the mini have identical parts and run identical firmware, but
not identical numbers: wheels and weight raise the duty a motor needs to
start. That is why calibration belongs to the rig, not to the build.

A rig is the whole assembly: each has its own Pi, and an Arduino stays with
its motors.

## 2. The profile file

One per rig, in the repository, so every calibration has a history in git.
The bench's, [`profiles/bench.conf`](../profiles/bench.conf), shortened:

```
# The bench rig: Uno + L293D, four yellow motors, no wheels.
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
  mixing needs it. The bench, with no wheels, will have none.
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

## 4. Getting the numbers to the Arduino: protocol v2

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

- `heron_motor` (motor step 4) loads the motor section and owns the
  handshake: wait for Boot, send Configure, check the checksum in Status,
  and repeat after any Boot.
- The Pi-side motor settings (geometry, maximum speed) come from the same
  file when the mixing needs them.
- Other workstreams read their own sections when they add them.

## 6. Calibrating a rig

A procedure document, `docs/rig-calibration.md`, one section per profile
section, each saying what to measure, with which tool, and what to write
where. The motor section is what step 3 did by hand:

| # | Measure | How |
|---|---|---|
| 1 | Which terminal each motor is on | `motor_link drive` each motor alone, with an identity map; watch which wheel turns |
| 2 | Its sign | the same runs: forward or backward |
| 3 | Dead band, per motor and direction | `motor_link sweep --min-duty 0`; first steady turn from rest, plus a margin |
| 4 | Ramp | step changes on the floor (the mini, the robot): firm, no jolt, no reset |

Later, `motor_link calibrate` can walk through these and print a ready
`[motor]` block to paste into the profile. The person calibrating commits
the profile, with the record filled in.

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
2. **Protocol v2 and the firmware:** Configure, the unconfigured state, the
   checksum; `PlatformConfig.h` removed; `motor_link --profile` and its
   overrides. Bench checks: unconfigured refuses Drive; configured drives
   as step 3 did; a reset mid-drive releases the motors.
3. **The calibration document**, and `motor_link calibrate` if it earns its
   place.
4. Then motor step 4, `heron_motor`, on top.

## 9. Decisions from review (2026-10-25)

1. **Dead band per motor and direction:** eight values, not one (§2).
2. **Strict files:** unknown keys and sections, missing keys and values
   out of range are errors (§2).
3. **Configure only while stopped** (§4): a calibration tool stops first.
4. **Tracking:** profiles cross workstreams, so this document and the
   profile format are shared, like `cpp-build.md`, and listed in
   `CLAUDE.md`. The motor parts (protocol v2, the firmware) are motor steps,
   recorded in `RESUME-motor.md`.
