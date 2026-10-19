# Heron wiring guide

How to connect Heron's sensors to the Raspberry Pi: the RPLIDAR on USB, the
remote control receiver on the serial port, and the odometry sensor on I²C.
Every pin here is the one used on the working robot, as of 2026-10-18.

This is shared hardware, not one workstream's: the facts come from
`CLAUDE.md` (header use), the remote's protocol reference §1, the odometry
reference §2 and §6, and the LiDAR's udev rule in `target/udev/`.

## Wiring diagram

Only pins 1–10 of the 40-pin header are used. Pin 1 is the corner pin at the
end of the header farthest from the USB ports; odd pins are on the inner
row, even pins on the board's outer edge.

```
                  outer edge of the board
            ┌─────┬─────┬─────┬─────┬─────┐
   even  →  │  2  │  4  │  6  │  8  │ 10  │
            │ 5V  │ 5V  │ GND │ TXD │ RXD │
            ├─────┼─────┼─────┼─────┼─────┤
   odd   →  │  1  │  3  │  5  │  7  │  9  │
            │ 3V3 │ SDA │ SCL │  –  │ GND │
            └─────┴─────┴─────┴─────┴─────┘
              pin 1                          pins 11–40 unused

   Receiver, SERVO (i-BUS) port        Odometry sensor, Qwiic cable
     S  (signal) ──── pin 10             3V3 ──── pin 1
     +           ──── pin 4  (5 V)       SDA ──── pin 3
     −           ──── pin 6  (GND)       SCL ──── pin 5
                                         GND ──── pin 9
   Pin 8 (TXD) stays empty.

   USB:    RPLIDAR A1M8 via its USB adapter board   → /dev/rplidar
           Arduino motor controller, 4 motors        (planned, not built)
   Power:  5 V supply into the Pi's micro-USB port
   Network: Ethernet, or Wi-Fi
```

## Header connections

| Pin | Pi signal | Goes to | Notes |
|---|---|---|---|
| 1 | 3.3 V | Odometry sensor 3V3 | **3.3 V only:** the sensor has no regulator |
| 3 | GPIO2, I²C SDA | Odometry sensor SDA | I²C bus 1, 100 kHz, address 0x17 |
| 4 | 5 V | Receiver + | Any 5 V pin works: 2 or 4, or a HAT's 5 V |
| 5 | GPIO3, I²C SCL | Odometry sensor SCL | |
| 6 | Ground | Receiver − | Any ground pin works |
| 9 | Ground | Odometry sensor GND | Any ground pin works |
| 10 | GPIO15, UART RXD | Receiver S (signal) | Fixed: the only pin the receiver's data can use. Its signal is 3.3 V, safe for the Pi |
| 8 | GPIO14, UART TXD | nothing | Leave empty: the link is receive-only |

**Check pins 2 and 4 before powering on.** They carry 5 V and sit right
beside pins 1 and 3, the odometry sensor's 3.3 V and SDA. A wire one pin
over can destroy the sensor.

## Parts

| Part | Connects by | Notes |
|---|---|---|
| Raspberry Pi 3B | — | Debian trixie, set up by `scripts/provision_pi.sh` |
| 5 V power supply, micro-USB | Pi power port | A weak supply caused under-voltage while the LiDAR spun (`docs/rplidar-a1m8-findings.md`). Use one that keeps `vcgencmd get_throttled` at `0x0` with everything running |
| SLAMTEC RPLIDAR A1M8, with its USB adapter board | USB cable to any Pi USB port | Powered from USB. The adapter is a CP2102 USB-serial bridge |
| FlySky FS-iA6B receiver and FS-i6X transmitter | 3 jumper wires from the SERVO (i-BUS) port | Use the SERVO port, not SENS |
| SparkFun Optical Tracking Odometry Sensor (OTOS), PAA5160E1 | Qwiic cable to female jumpers, 4 wires | Either Qwiic socket; the other is for chaining |
| Jumper wires, female to female | — | 3 for the receiver |
| A pass-through GPIO HAT | The Pi header | Optional. Heron uses one for spare 5 V and ground connections; pins pass straight through |
| Arduino motor controller, driving 4 motors (holonomic) | USB serial | Planned, not built. The LiDAR's udev rule already allows for it (`target/udev/`) |

## Build steps

1. **Provision the Pi first.** From the repository, run
   `scripts/provision_pi.sh <user>@<pi>`, then reboot the Pi. It enables I²C
   at 100 kHz, frees the serial port on pin 10 from Bluetooth and the
   console, and installs the rule that names the LiDAR `/dev/rplidar`.
2. **Power the Pi off** before wiring anything to the header.
3. **Odometry sensor.** Remove the yellow tape over the lens. Connect the
   Qwiic cable's 4 wires: 3V3 to pin 1, SDA to pin 3, SCL to pin 5, GND to
   pin 9. A common Qwiic colour code is red 3.3 V, black ground, blue SDA,
   yellow SCL, but check your cable's labels.
4. **Receiver.** On the SERVO (i-BUS) port, connect S to pin 10, + to pin 4,
   and − to pin 6. On FlySky receivers the 3-pin order is usually S, +, −
   with − at the board edge; check the labels on yours.
5. **Check pins 1 to 6** against the table, especially that nothing 5 V
   touches pin 1 or 3.
6. **LiDAR.** Plug its USB adapter board into the sensor, and the USB cable
   into any Pi USB port.
7. **Mount the odometry sensor** rigid and level, within 3°, sensor face
   down, 10–27 mm above the floor. It must not tip or rise as the robot
   moves.
8. **Power on** with a good supply, and run the checks below.

## Checks after wiring

From the repository on the host. `<user>@<pi>` is your login and the Pi's
address.

```
ssh <user>@<pi> vcgencmd get_throttled          # throttled=0x0
ssh <user>@<pi> /usr/sbin/i2cdetect -y 1        # 17, and nothing else
scripts/deploy_cpp.sh <user>@<pi> odometry_adapter 30
scripts/deploy_cpp.sh <user>@<pi> remote_adapter 20
ssh <user>@<pi> ls -l /dev/rplidar
scripts/deploy_cpp.sh <user>@<pi> lidar_adapter 20
```

- **`odometry_adapter`:** keep the robot still until lines print; each
  should say `RUNNING yes` with about 100 readings.
- **`remote_adapter`:** switch the transmitter on first. Each line should
  show `link OK` and about 130 frames, and the channels should move with
  the sticks.
- **`lidar_adapter`:** the LiDAR spins and reports about 7.6 scans a
  second.

## Transmitter set-up

On the FS-i6X. The robot stops through the arm switch, so this is needed
before it can drive safely (details in `docs/RESUME-remote.md`):

- **10 channels:** System setup › Aux switches, switches on, channel count
  10.
- **Switches on channels:** Functions setup › Aux channels: CH7 SwA, CH8
  SwB, CH9 SwC, CH10 SwD.
- **Failsafe:** System setup › RX setup › Failsafe: CH7 −100% (SwA up,
  disarmed) and CH3 −100%; every other channel Off. Hold CANCEL to save.
  These settings have been lost once, so check them after any model change.

Then confirm with `remote_adapter`: switch the transmitter off, and
`failsafe YES` should appear about 1 second later.
