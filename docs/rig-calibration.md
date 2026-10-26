# Calibrating a rig

How to measure a new rig and write its profile, step by step. A rig is
one physical Heron with its own Pi, Arduino and motors: the bench rig, the
mini robot, later the full-size robot. Its profile,
`profiles/<rig>.conf`, holds the numbers measured on it; the code is the
same on every rig. Why it works this way, and the file format, are in
[`rig-profiles.md`](rig-profiles.md).

Today the profile has one section to measure, `[motor]`. Sections for the
odometry sensor, the LiDAR and the robot's geometry are added here as their
workstreams need them (§8).

The procedure below is the one the bench rig was calibrated with on
2026-10-25, written out so the next rig takes an afternoon rather than a
rediscovery. Its results are in `profiles/bench.conf` and
[`RESUME-motor.md`](RESUME-motor.md).

## 1. Before you start

| Need | How |
|---|---|
| The Pi provisioned | `scripts/provision_pi.sh <user>@<pi>`. It names the Pi `heron-<mac>`, so several rigs share a network |
| The Arduino in USB port 1-1.2 | Only that port is named `/dev/arduino` (`target/udev/`) |
| The firmware for the rig's platform | `scripts/build_firmware.sh`, then `scripts/flash_firmware.sh <user>@<pi>` (default `uno-l293d`) |
| The C++ built | `scripts/build_cpp.sh` |
| On an L293D shield: the PWR jumper **off** | Otherwise the Pi's USB 5 V back-feeds the motor supply |
| The motor battery on EXT_PWR | The battery the rig will run on: the dead band depends on its voltage |
| **The wheels off the ground** | Steps 3 and 4 drive one motor at a time; on the floor the rig would spin |

Every `motor_link` command below runs on the Pi. Either deploy and run in
one go:

```
scripts/deploy_cpp.sh <user>@<pi> motor_link drive 600 0 0 0
```

or, to watch the output live while you watch the wheels (needed for the
sweep), from a second terminal:

```
ssh -t <user>@<pi> '~/heron/bin/motor_link sweep 1 --step 20'
```

Ctrl-C always sends Stop.

## 2. Start the profile

Make a branch for the calibration, then a starting profile with nothing
assumed: every motor on its own terminal, all the same way round, no dead
band. Copy this into `profiles/<rig>.conf`, with the rig's name in both
places:

```
# The <rig> rig: <board, driver and motors; wheels or not>. Its own Pi.
format = 1

[rig]
name = <rig>
platform = uno-l293d

[motor]
# Starting values, to be measured (docs/rig-calibration.md).
terminal = 1 2 3 4
sign = + + + +
ramp_per_ms = 4
min_duty_forward = 0 0 0 0
min_duty_backward = 0 0 0 0
```

Rig names are lower case, digits and `-`, and the file must be named after
the rig. Check it, tell the Pi which rig it is, and deploy:

```
build-host/bin/rig_profile check profiles/<rig>.conf
scripts/set_rig.sh <user>@<pi> <rig>
scripts/deploy_cpp.sh <user>@<pi> rig_profile show
```

**After every edit to the profile, deploy again** (`deploy_cpp.sh` copies
`profiles/` with the binaries). No rebuild and no reflash: `motor_link`
sends the profile to the Arduino each time it starts.

## 3. Which motor is where, and which way it turns

The protocol numbers the motors clockwise from front-left, and positive
drives the robot forward:

| Motor | Wheel |
|---|---|
| 1 | front-left |
| 2 | front-right |
| 3 | back-right |
| 4 | back-left |

Drive each protocol motor alone and note what happens:

```
motor_link drive 600 0 0 0 --seconds 2
motor_link drive 0 600 0 0 --seconds 2
motor_link drive 0 0 600 0 --seconds 2
motor_link drive 0 0 0 600 --seconds 2
```

With the identity map, protocol motor *n* drives shield terminal M*n*, so
each run tells you which wheel is on that terminal and which way it turns
for "forward". Fill in a table like the bench's:

| Terminal | Wheel | Turned |
|---|---|---|
| M1 | front-left | forward |
| M2 | front-right | forward |
| M3 | back-left | forward |
| M4 | back-right | forward |

Then write the profile from it:

- **`terminal`:** for each protocol motor in order (front-left,
  front-right, back-right, back-left), the terminal number of that wheel.
  The bench's table gives `terminal = 1 2 4 3`.
- **`sign`:** `-` for any wheel that turned backwards, `+` otherwise.
  Swapping that motor's two wires instead is equally right; then keep `+`.

Deploy and run the four drives again. Now they must go front-left,
front-right, back-right, back-left, clockwise round the rig, all forward.
If one runs that should not, or two at once, the wiring is not what the
table says: check it before going on.

## 4. The dead band

A DC motor does not turn at a low duty; it only hums. The firmware skips
that range: the smallest non-zero value maps to the duty where the motor
starts to turn. That duty is measured for each motor in each direction.

With the dead band switched off for the run (`--min-duty 0`), sweep each
motor from low to full, forward then backward, 1 s per value, and watch it:

```
ssh -t <user>@<pi> '~/heron/bin/motor_link sweep 1 --step 20 --min-duty 0'
```

Note the **first value where the wheel turns steadily for the whole second
from rest**, even slowly. A hum, a twitch or a partial turn that stops does
not count; a slow turn that is still speeding up does. Note it going
forward and going backward, then Ctrl-C and repeat for motors 2, 3 and 4.
Each line shows the value and its duty (about value × 255 / 1000).

Write the duties down, add a margin of 10 to each, and put them in the
profile in protocol motor order:

```
# PWM duty where each motor first turned steadily from rest, plus 10.
# Measured forward 66 71 77 82, backward 61 66 66 92.
min_duty_forward = 76 81 87 92
min_duty_backward = 71 76 76 102
```

Keep the measured values in the comment: they are what a later
recalibration compares against. Deploy, then check the edge of the band:

```
motor_link drive 1 1 1 1 --seconds 3
motor_link drive -1 -1 -1 -1 --seconds 3
```

All four wheels must turn, slowly, both ways. A wheel that does not needs a
higher value; one that jumps off fast has more margin than it needs.

**On a rig with wheels on the floor** the robot's weight raises the duty a
motor needs to start, so the bench's numbers do not carry over, and a
wheels-off sweep measures too low. How best to measure under load (all four
together, on the floor, for example) is **not yet worked out**; it will be
written here when the mini robot is calibrated.

## 5. The ramp

`ramp_per_ms` limits how fast each wheel's value may change: 4 takes 0 to
full in 250 ms. Start with 4 and check a full-speed start:

```
motor_link drive 1000 1000 1000 1000 --seconds 2
```

Status shows `applied` climbing over about 250 ms. The start must be
smooth, and **no unexpected Boot** may appear in the output: one means the
start drew enough current to reset the Arduino, so the ramp must be slower
(a smaller number). On the bench nothing moves, so 4 is a starting point;
a robot on the floor is where it is tuned, for starts that are firm without
jolting.

## 6. Check and record

Check the finished profile on the host, and what the Pi reads:

```
build-host/bin/rig_profile check profiles/<rig>.conf
ctest --test-dir build-host -R rig_profiles_check
scripts/deploy_cpp.sh <user>@<pi> rig_profile show
```

Fill in `[motor.calibration]`: free text, any keys, so whoever recalibrates
can tell what was done. The bench's:

```
[motor.calibration]
date = 2026-10-25
method = motor_link sweep, step 20, 1 s per value, from rest; +10 margin on each
supply = AA cells on EXT_PWR, PWR jumper off
terminal_and_sign = each motor driven alone at 600, wheel and direction watched
ramp = 0 to 1000 in about 250 ms, smooth, no reset
```

Commit the profile on the branch and open a pull request, as for any other
change: the profile is the rig's calibration record.

## 7. When to recalibrate

| Change | Redo |
|---|---|
| A motor rewired or moved to another terminal | §3 |
| A motor replaced | §3 for it, §4 for it |
| Different wheels, more weight, or wheels on the floor for the first time | §4, §5 |
| Another battery type or voltage | §4 |
| A wheel that no longer starts at the smallest values | §4: the battery may be low, or the motor wearing |
| A new motor driver | A new platform: the firmware's driver changes, then every section |

## 8. Later sections

Each is written here when its workstream adds the matching profile
section:

- **`[geometry]`**, for the mixing: wheel radius, track, wheelbase,
  maximum speed. Rigs with wheels only.
- **`[odometry]`**: the OTOS's linear and angular scalars and its mounting
  offset ([`odometry-otos-reference.md`](odometry-otos-reference.md) §5,
  §6).
- **`[lidar]`**: the LiDAR's mounting offset.
