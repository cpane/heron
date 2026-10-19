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
