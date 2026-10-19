# Odometry workstream — resume point

Last updated 2026-10-18. Branch **`feature/odometry-library`**.

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
| Sensor reference | [`odometry-otos-reference.md`](odometry-otos-reference.md) | researched and tagged; measured on the Pi |
| Library design | [`odometry-cpp-design.md`](odometry-cpp-design.md) | built |
| `heron_otos`: register map and codec | `cpp/src/odometry/OtosCodec.*` | built; `otos_codec_test` passes on host and Pi |
| `heron_otos_device`: bus and start-up | `cpp/src/odometry/` | built; `otos_device_test` passes on host and Pi |
| `heron_odometry`: `Odometry`, what robot code links | `cpp/include/heron/odometry/Odometry.h` | built; `odometry_test` passes on host and Pi |
| `odometry_adapter`: probe O6, Odometry live | `cpp/probes/OdometryAdapter.cpp` | run 1: 100 Hz, 0 failures; unwrap not yet exercised live |
| `otos_device`: probe O5, OtosDevice live | `cpp/probes/OtosDeviceLive.cpp` | run: start-up works on the board; scalars read back (reference §4) |
| I2C bus enabled in provisioning, 100 kHz | `target/provision/base.sh` | run on the Pi; `/dev/i2c-1` present |
| `otos_bus`: probe O1, read reliability | `cpp/probes/OtosBus.cpp` | clean: 150 000 iterations with the LiDAR and remote running, 0 errors (reference §3) |
| `otos_start`: probe O2, start-up then still | `cpp/probes/OtosStart.cpp` | run on the Pi: start-up needed; still drift measured (reference §4, §8) |
| `otos_move`: probe O3, pushed a measured distance | `cpp/probes/OtosMove.cpp` | run 2, guided: within 0.7%, mean +0.18%; scalar stays 1.0 (reference §5) |
| `otos_spin`: probe O4, spun a counted number of turns | `cpp/probes/OtosSpin.cpp` | run 4, out and back: no direction difference, steadiest spins within 0.05%; scalar stays 1.0 (reference §5) |

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
4. **Angular, probe O4:** `scripts/deploy_cpp.sh <user>@<pi> otos_spin`.
   Runs 1–2 were spoiled by the cable winding up. Run 4, out and back
   (5 turns each way, Ethernet unplugged): no difference between
   directions; the error fell trial by trial with steadier handling, to
   0.05%. The angular scalar stays 1.0 (reference §5). Both scalars are
   to be checked again on the robot.
5. **Bus soak done:** 150 000 iterations in 977 s with the LiDAR and
   remote running, 0 errors (reference §3).
6. **Library, branch `feature/odometry-library`:** design drafted in
   [`odometry-cpp-design.md`](odometry-cpp-design.md), reviewed.
   - **`heron_otos` built:** `src/odometry/OtosCodec.h` (private register
     map, scalings, snapshot decoding, scalar and offset encoding, angle
     wrapping) and `include/heron/odometry/Planar.h`. `otos_codec_test`
     passes on the host and the Pi, 63 checks.
   - **`heron_otos_device` built:** `I2cBus` (interface), `LinuxI2cBus`
     (i2c-dev; throws off Linux), `OtosDevice` (start-up sequence,
     snapshot reads), public `OtosConfig.h`. `otos_device_test` passes on
     the host and the Pi, 29 checks, against `tests/FakeOtosBus.h`.
   - Live check, probe O5: `scripts/deploy_cpp.sh <user>@<pi> otos_device`
     (optional: seconds, then linear and angular scalars). Run
     2026-10-18 with 0.995 and 1.002: scalars read back as written,
     calibration 619 ms, status 0x00, 0 failed reads (reference §4).
   - **`heron_odometry` built:** `include/heron/odometry/Odometry.h`
     (`Odometry`, `OdometrySnapshot`, `OdometryHealth`) and
     `src/odometry/Odometry.cpp`. `odometry_test` passes on the host and
     the Pi, 44 checks, repeated runs stable.
   - Live check, probe O6: `scripts/deploy_cpp.sh <user>@<pi>
     odometry_adapter 30` (optional scalars after the seconds). Run 1,
     2026-10-18: 100 readings a second, 0 failures, 0 not OK at 1500
     polls (reference §4). The turn stayed under 180°, so unwrapping is
     still to be seen live: run again turning past half a turn.
   - Then: push and PR the library.

Checking the bus by hand: `ssh <user>@<pi> /usr/sbin/i2cdetect -y 1`. The
full path is needed: `/usr/sbin` is not on the PATH of a non-login ssh
command.

## Open questions

- Mounting: on a rigid test sled, not the robot yet. Height must be 10–27 mm
  above the floor, rigid and level; the offset from the robot's centre is
  written to the sensor.
- The rest are in the reference, §10.
