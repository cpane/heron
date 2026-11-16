# Odometry workstream — resume point

Last updated 2026-10-18. Everything is merged into `develop` (PRs #10,
#12).

## Read this first when resuming

**Paused 2026-10-18** at a clean stopping point. The sensor is understood
and measured, and there is a reusable C++ library for it, verified on the
hardware.

The sensor is a **SparkFun Optical Tracking Odometry Sensor (OTOS)**,
PAA5160E1, on the Pi's I2C bus 1 at 0x17, 100 kHz:
<https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html>.
Wired through the HAT (3V3, SDA pin 3, SCL pin 5, GND; header use is in
`CLAUDE.md`). Scope was the sensor and a reusable reader, not robot
localisation: mounting, fusion with the LiDAR and recalibration policy wait
for the robot design.

| Piece | Where | State |
|---|---|---|
| Sensor reference | [`odometry-otos-reference.md`](odometry-otos-reference.md) | sourced and tagged; bus (§3), start-up (§4), scalars (§5) and drift (§8) measured |
| Library design | [`odometry-cpp-design.md`](odometry-cpp-design.md) | built as designed |
| `heron_otos`: register map, scalings, codec | `cpp/src/odometry/OtosCodec.*` (private) | `otos_codec_test`, 63 checks, host and Pi |
| `heron_otos_device`: `I2cBus`, `LinuxI2cBus`, `OtosDevice` | `cpp/src/odometry/` (private) | `otos_device_test`, 29 checks, host and Pi |
| **`heron_odometry`: `Odometry`**, what robot code links | `cpp/include/heron/odometry/` | `odometry_test`, 44 checks, host and Pi; live on the sensor |
| `odometry_adapter`: probe O6, `Odometry` as a 50 Hz loop sees it | `cpp/probes/OdometryAdapter.cpp` | two live runs |
| `otos_device`: probe O5, `OtosDevice` live | `cpp/probes/OtosDeviceLive.cpp` | one live run |
| Probes O1–O4: `otos_bus`, `otos_start`, `otos_move`, `otos_spin` | `cpp/probes/Otos*.cpp` | raw-register diagnostics, all run |
| I2C at 100 kHz in provisioning | `target/provision/base.sh` | run on the Pi |

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh <user>@<pi> odometry_adapter 30
scripts/deploy_cpp.sh <user>@<pi> otos_bus 150000
ssh <user>@<pi> /usr/sbin/i2cdetect -y 1     # full path: not on a non-login PATH
```

## What the robot must know

- **Start still.** `Odometry::start()` calibrates the IMU: about 0.63 s,
  and the sensor must be still and level throughout. Start odometry with
  the motors off, before accepting commands. Restarting after a failure
  needs the same.
- **One test:** `OdometrySnapshot::isOk()` is fresh (no older than 100 ms)
  and no chip or IMU error. The tilt and tracking warnings do not fail it:
  the tilt bit was set through accurate spins (reference §5). Distrust x
  and y, not the heading, on a tracking warning.
- **Heading:** use `totalHeading`, which counts whole turns; `pose.h` wraps
  at ±180°.
- **Drift:** about 0.2°/min after a calibration, growing to 0.6–0.9°/min
  over 10 min (19–35°/h); once 2.2°/min straight after a hand spin
  (reference §8, §4). Long runs need a heading correction: recalibrating
  while stopped, or the LiDAR.
- **Failure:** one second without a good read sets `Failed` and stops the
  reader, because a sensor that lost power lost its calibration and
  settings. `start()` again, still.
- **Still the robot's:** the mounting offset (`OtosConfig::offset`) and
  its sign conventions, when to recalibrate, and fusion with the LiDAR.

## Picking up: optional, none blocks the robot

1. **Offset conventions** (reference §6, §10 question 2): which way the
   sensor's axes and heading point, and whether an offset write takes
   effect. Needs a probe, and is needed once the sensor is on the robot.
2. **Scalars on the robot:** both measured at 1.0 on a hand-pushed sled
   (reference §5). Re-measure linear and angular once the sensor is
   mounted and driven by motors.
3. **Drift after a spin:** 2.2°/min after a hand spin, against 0.23°/min
   otherwise (reference §4). Repeat to see whether it is real.
4. **Velocity near zero:** 0.3–1.1 mm/s reported while the pose held at 0
   (reference §4). Matters only if the robot uses velocity at rest.
5. **Left out of v1** (design §6): `recalibrate()` and `resetPose()` while
   running, setting the pose, detecting a sensor restart, register 0x0E,
   IO9.
6. **Beyond ±10 m** the position registers may wrap (reference §8).

## Verified so far

On the Pi with the sensor, 2026-10-13 to 2026-10-18 (details in the
reference):

- **Bus:** 150 000 reads of 37 bytes in 977 s with the LiDAR and remote
  running, 0 failures; 3.66–6.28 ms per read.
- **Start-up:** calibration 609–619 ms, reset 11 ms; without them a tilt
  warning and 12°/min drift. Scalars 0.995 and 1.002 read back as
  written.
- **Distance:** 5 guided pushes of 1 m, all within 0.7%, mean +0.18%.
- **Turns:** out-and-back spins of 5 turns, no difference between
  directions, the steadiest within 0.05%.
- **`Odometry` live:** 100 readings a second, readings 4–6 ms old, 0
  failures in 3002 reads, `stop()` 10 ms; `totalHeading` unwrapped
  correctly through 1.5 turns and back.
- The three test suites pass on the host and the Pi.

**Not verified:** the offset, the scalars on a motor-driven robot,
behaviour beyond ±10 m.

## Open questions

The rest are in the reference, §10.

## Inbox

Facts other workstreams found that belong here, waiting to be checked and
folded into this workstream's docs when it resumes (`CLAUDE.md`,
"Workstreams"). Newest last; delete an entry once folded in.

### From navigation, 2026-11-11: the OTOS's scale depends on the floor

[ours] On the bench rig, sensor 14–15 mm above the floor, tape off,
default scalars. **Carpet:** about 405 mm pushed by hand read 297 mm
forward and 275 mm sideways (about −27%), status 0x00, no warnings. A
board laid on the same carpet: 517 mm and 496 mm for 500 mm.
**Hardwood** (`otos_move`, five 1000 mm pushes along a straight edge):
mean 1101.8 mm, +10.2%, sd 8.7 mm, correcting scalar 0.908 (0.902 to
0.920). Your sled's wood floor read +0.2% (reference §5). The reference's
"more than 5% points to poor tracking" and the LUT "built at 10 mm"
[community] may bear on it: whether height explains the hardwood's +10%
is [unverified]. Full numbers: `RESUME-navigation.md`, "Picking up" 2.

### From navigation, 2026-11-13: driven spins read +0.9%, hand spins −0.2%

[ours] On the bench rig, hardwood, `otos_spin`, the robot spun in place
by its own motors (`heron_teleop`), 5 turns out and 5 back, squared up
by hand at the turnaround: six legs +0.64% to +1.14%, mean +0.89%, sd
0.20%. All 20 driven legs that day read long or about zero, mean +0.98%.
Your hand-spun sled read −0.20% (reference §5, run 3), same chip. The
bench rig's profile now has `angular_scalar = 0.991`. Why driven spins
differ is [unverified]: motor vibration on the gyro is a guess, untested.
Full numbers: `RESUME-navigation.md`, "Picking up" 2, and the profile's
`[odometry.calibration]`.

### From navigation, 2026-11-14: a speed with the robot still, and sideways while driving straight

[ours, seen in four runs, not studied] On the bench rig, hardwood,
`heron_robot --lines 10` printing `PoseTracker`'s velocity (the OTOS's,
robot frame). **Still:** after a run that ended with the robot coasting
to a stop, x read −0.02 to −0.05 m/s and y +0.01 to +0.02 for as long as
it was watched (over 10 s), the robot not moving; before the first run
it read ±0.00. **Driving straight** at full speed: y read 0.05 to 0.09
m/s to the right, and x alternated 0.31 to 0.41 from line to line.
Whether the robot crabs or the sensor reads across is [unverified]. It
matters to navigation: the limiter's look-ahead uses this speed.
`RESUME-navigation.md`, "Picking up" 3.

### From navigation, 2026-11-14: nonsense speeds after a lift, called fresh

[ours, five times in one session] On the bench rig, hardwood,
`heron_robot --lines 10` printing `PoseTracker`'s velocity (the OTOS's,
robot frame). After the robot was lifted and set down between runs, the
forward speed read −2.6 m/s and −3.4 to −3.6 m/s for seconds, through a
whole 3.7 s strafe left, while the sideways speed stayed sane (0.14 m/s);
and −1.23 m/s with the robot standing disarmed. It recovered after the
robot had driven, or stood, for a while. `PoseTracker` still called the
pose fresh (Odometry OK, no gap). Whether the OTOS's status or warning
bits flagged it was not printed [unverified]. Navigation proposes to
treat such readings as not fresh (`navigation-design.md` §19, P16); what
the OTOS is doing is yours. Log: `captures/strafe-2026-11-14.log` (not
in git; ask navigation). `RESUME-navigation.md`, "Picking up" 3.

### From navigation, 2026-11-14: the OTOS's velocity is in its odom frame

[ours] On the bench rig, hardwood, `heron_robot --lines 10` printing the
OTOS's velocity and heading through `Odometry`. After the robot had
turned, straight runs printed a sideways part that turning the velocity
by −heading removed: forward at a heading of +13° read (0.200, +0.040)
m/s, turned (0.204, −0.005); backward at −13° read (−0.300, +0.070),
turned (−0.308, +0.001). A taped strafe that turned the robot about 68°
agreed with the tape only with the velocity read in the odom frame. So
the velocity registers are in the same frame as the pose, not the
sensor's own. Navigation now turns it in `PoseTracker`; `Odometry`'s
docs may want to say which frame it is. This also explains the earlier
entry's "0.05 to 0.09 m/s sideways at full speed straight ahead": the
robot had turned. Log `captures/table-hardwood-1-2026-11-14.log` (not in
git). `RESUME-navigation.md`, "Picking up" 3.

### From navigation, 2026-11-16: a failed start after boot, and TILT that stayed on

[ours, causes unverified] Bench rig, hardwood. (1) About a minute after
the Pi booted, `heron_robot` reported `OtosDevice::start() : scalars
read back 0xA4 0x00, written 0xA4 0xF7` (the angular scalar did not take)
and ran without odometry; started again a minute later, the OTOS came up
normally. (2) In the same session, after the robot was lifted and set
down flat, the status line kept `(not fresh) TRACKING TILT` for the rest
of the run, three 4 s runs and the pauses between them, the robot flat
on the floor throughout; the speeds read during it matched clean runs
(forward 0.31 m/s against 0.31). Restarting `heron_robot` cleared it.
Log `captures/left-diagonals-hardwood-1-2026-11-16.log` (not in git);
navigation's `RESUME-navigation.md`, step 5.

### From navigation, 2026-11-16: a lift left TRACKING on and nonsense speeds for over a minute

[ours] Bench rig, hardwood, `heron_robot --dry-run`. The robot picked up
and turned by hand (Chris): the heading swung from −1° to −51° and back
with TILT, then TRACKING came on with the pose not fresh and speeds about
−3.0 m/s forward and −1.1 to −2.2 m/s sideways, drifting slowly, for the
rest of the run (over a minute) with the robot back on the floor and
still. The odometry library never recovered by itself; a restart of the
program did. Earlier lifts (2026-11-14, in this Inbox) cleared in about
0.3 s. Also the same day, a second failed start shortly after power-on:
the scalars read back 0x00 0x00 for 0xA4 0xF7 (the first, 2026-11-14,
read 0xA4 0x00); `heron_robot` now retries the start. Write-up:
`RESUME-navigation.md`, "Picking up" 0.

### From navigation, 2026-11-16: the phantom speed does not move the position

[ours, two runs; write-up in `navigation-cpp-design.md` §7.5, "As built:
starting from rest"] On the bench rig, hardwood, `heron_robot --lines 10`
in Navigate: standing still after a run, the OTOS's velocity read
−0.07 m/s (x) for several seconds in one run and −0.05 in another, the
LiDAR's nearest point unchanged; in the second, the pose's position moved
0.00 m/s over 0.5 s throughout (`PlanInput::movedMps`). So the phantom
speed (this Inbox, 2026-11-14) is in the velocity registers only, and it
reached −0.07, past the −0.05 seen before. Navigation now takes the
robot as still when either says so.

