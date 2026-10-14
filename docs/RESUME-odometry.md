# Odometry workstream — resume point

Last updated 2026-10-12. Branch **`feature/odometry`**.

## Read this first when resuming

The sensor is a **SparkFun Optical Tracking Odometry Sensor (OTOS)**,
PAA5160E1, Qwiic, on the Pi's I2C bus:
<https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html>.
The aim of this stage is the same as for the LiDAR and the remote: understand
the sensor, build a reusable reader, and verify it on the hardware. It is not
robot localisation yet.

**Measured so far:** it answers on the bus. Wired 2026-10-13 through the
HAT's SDA/SCL/3V3/GND (the HAT passes GPIO2/3 through); `i2cdetect -y 1`
shows 0x17 and nothing else.

| Piece | Where | State |
|---|---|---|
| Sensor reference | [`odometry-otos-reference.md`](odometry-otos-reference.md) | researched and tagged; address measured |
| I2C bus enabled in provisioning, 100 kHz | `target/provision/base.sh` | run on the Pi; `/dev/i2c-1` present |
| `otos_bus`: probe O1, read reliability | `cpp/probes/OtosBus.cpp` | clean on the Pi: 2000 iterations, 0 errors (reference §3) |

## Picking up

Steps 1–3 are done: wired (reference §2), provisioned, 0x17 answers.

1. **Probe O1 is clean** (reference §3). Status read 0x01 (tilt) every time,
   with the board sensor side up on the bench: re-run with it sensor side
   down, raised 10–25 mm, to see whether the warning clears.
2. Then the start-up sequence (reference §4): calibration, scalars, reset.
   Later, a long `otos_bus` soak with the LiDAR and remote running.

Checking the bus by hand: `ssh <user>@<pi> /usr/sbin/i2cdetect -y 1`. The
full path is needed: `/usr/sbin` is not on the PATH of a non-login ssh
command.

## Open questions

- Mounting: not mounted yet (loose on the bench). Height must be 10–27 mm
  above the floor, rigid and level; the offset from the robot's centre is
  written to the sensor.
- The rest are in the reference, §10.
