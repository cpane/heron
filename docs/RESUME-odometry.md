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
| I2C bus enabled in provisioning | `target/provision/base.sh` | written; not yet run on the Pi |

## Picking up

1. **Research the sources** (product page, hookup guide, register map, the
   SparkFun libraries and their licenses) into a sensor reference,
   `docs/odometry-otos-reference.md`, with every fact tagged as in the
   remote's protocol doc.
2. **Run provisioning** (`scripts/provision_pi.sh <user>@<pi>`, then
   reboot) and check `/dev/i2c-1` exists and the user is in group `i2c`.
3. **Attach the sensor** (Qwiic: 3.3 V, GND, SDA on GPIO2 / pin 3, SCL on
   GPIO3 / pin 5) and confirm it answers with `i2cdetect -y 1`.
4. Then a first C++ probe that reads its product ID, and on from there.

## Open questions

- Is the sensor mounted on the robot, and at what height and offset from
  the robot's centre?
