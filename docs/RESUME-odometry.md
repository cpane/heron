# Odometry workstream — resume point

Last updated 2026-10-17. Branch **`feature/odometry`**.

## Read this first when resuming

The sensor is a **SparkFun Optical Tracking Odometry Sensor (OTOS)**,
PAA5160E1, Qwiic, on the Pi's I2C bus:
<https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html>.
The aim of this stage is the same as for the LiDAR and the remote: understand
the sensor, build a reusable reader, and verify it on the hardware. It is not
robot localisation yet.

**Measured so far:** it answers on the bus, reads cleanly, and holds still after the start-up sequence. Wired 2026-10-13 through the
HAT's SDA/SCL/3V3/GND (the HAT passes GPIO2/3 through); `i2cdetect -y 1`
shows 0x17 and nothing else.

| Piece | Where | State |
|---|---|---|
| Sensor reference | [`odometry-otos-reference.md`](odometry-otos-reference.md) | researched and tagged; address measured |
| I2C bus enabled in provisioning, 100 kHz | `target/provision/base.sh` | run on the Pi; `/dev/i2c-1` present |
| `otos_bus`: probe O1, read reliability | `cpp/probes/OtosBus.cpp` | clean on the Pi: 2000 iterations, 0 errors (reference §3) |
| `otos_start`: probe O2, start-up then still | `cpp/probes/OtosStart.cpp` | run on the Pi: start-up needed; still drift measured (reference §4, §8) |
| `otos_move`: probe O3, pushed a measured distance | `cpp/probes/OtosMove.cpp` | run 2, guided: within 0.7%, mean +0.18%; scalar stays 1.0 (reference §5) |

## Picking up

Steps 1–3 are done: wired (reference §2), provisioned, 0x17 answers.

1. **Probes O1 and O2 are done:** the bus is clean; the start-up sequence
   (IMU calibration, reset) is required, and with it a still board holds
   status 0x00 and its heading (reference §3, §4).
2. **Still drift, two 600 s runs:** 35°/h then 19°/h. Calibration resets
   the rate to about 0.2°/min and it grows from there (reference §8). Good
   enough to move on; recalibrating while stopped, and LiDAR heading
   correction, are robot-design questions.
3. **Moving, probe O3:** `scripts/deploy_cpp.sh <user>@<pi> otos_move`.
   Run 1 was mixed: the sled turned and tipped. Run 2, rigid and guided
   along a straight edge: all five within 0.7%, mean +0.18%, status 0x00;
   the linear scalar stays 1.0 (reference §5).
4. **Next: the angular scalar.** Spin the sled about 10 full turns in
   place and compare the reported heading with the true angle (reference
   §5). Needs a probe that unwraps the heading across ±180°.
5. Later, a long `otos_bus` soak with the LiDAR and remote running.

Checking the bus by hand: `ssh <user>@<pi> /usr/sbin/i2cdetect -y 1`. The
full path is needed: `/usr/sbin` is not on the PATH of a non-login ssh
command.

## Open questions

- Mounting: on a rigid test sled, not the robot yet. Height must be 10–27 mm
  above the floor, rigid and level; the offset from the robot's centre is
  written to the sensor.
- The rest are in the reference, §10.
