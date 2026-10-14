# Odometry workstream — resume point

Last updated 2026-10-12. Branch **`feature/odometry`**.

## Read this first when resuming

The sensor is a **SparkFun Optical Tracking Odometry Sensor (OTOS)**,
PAA5160E1, Qwiic, on the Pi's I2C bus:
<https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html>.
The aim of this stage is the same as for the LiDAR and the remote: understand
the sensor, build a reusable reader, and verify it on the hardware. It is not
robot localisation yet.

**Nothing has been measured yet.** The sensor has not been attached or read.

| Piece | Where | State |
|---|---|---|
| Sensor reference | [`odometry-otos-reference.md`](odometry-otos-reference.md) | researched and tagged; nothing measured |
| I2C bus enabled in provisioning, 100 kHz | `target/provision/base.sh` | written; not yet run on the Pi |

## Picking up

1. **Wire it** (reference §2): 3V3 to Pi pin 1, SDA pin 3, SCL pin 5, GND
   to a ground pin; IO9 and IO10 unconnected; never pin 2 or 4 (5 V). The
   Pi has no Qwiic socket: use a Qwiic-to-female-jumper cable, or solder a
   header into the board's edge holes.
2. **Run provisioning** (`scripts/provision_pi.sh <user>@<pi>`, then
   reboot) and check `/dev/i2c-1` exists and the user is in group `i2c`.
3. **Confirm it answers:** `ssh <user>@<pi> /usr/sbin/i2cdetect -y 1`
   should show 0x17. The full path is needed: `/usr/sbin` is not on the
   PATH of a non-login ssh command.
4. **First C++ probe:** read the product ID (0x00 = 0x5F), then 36-byte
   bursts from 0x20 in a loop, counting failures. The first risk is the Pi
   3's I2C and the board's clock stretching (reference §3, §10).

## Open questions

- Mounting: not mounted yet (loose on the bench). Height must be 10–27 mm
  above the floor, rigid and level; the offset from the robot's centre is
  written to the sensor.
- The rest are in the reference, §10.
