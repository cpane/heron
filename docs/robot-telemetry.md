# Robot telemetry — requirements

External programs will read the robot's sensor data live while it drives:
first a graphical client on a computer, drawing the LiDAR's view and the
odometry pose as the robot moves. **This is not a design**: the design that
keeps these rules is [`telemetry-design.md`](telemetry-design.md), the
telemetry workstream's (`RESUME-telemetry.md`). What is fixed here,
2026-10-25, are the rules the robot and its libraries must keep now so that
they can be added without rework.

## What already fits

The sensor libraries are shaped for it. `Lidar`, `Odometry`,
`RemoteControl` and `Motors` each run a reader thread of their own and
publish the newest state, latest wins, through `getLatest()`, which any
thread may call at any rate without blocking
([`lidar-cpp-design.md`](lidar-cpp-design.md),
[`odometry-cpp-design.md`](odometry-cpp-design.md),
[`remote-cpp-design.md`](remote-cpp-design.md),
[`motor-cpp-design.md`](motor-cpp-design.md)). So the robot's control loop
and a telemetry thread can read the same data side by side. **A new
library must keep that shape.**

## The rules

1. **The robot process is the only owner of each device.** A serial LiDAR
   or an I2C sensor cannot be shared between processes, and starting
   odometry recalibrates the sensor. Clients never open a device: they
   connect to a **telemetry server, a thread inside the robot process**,
   which reads `getLatest()` like any other consumer.
   `lidar_gui_server` owns the LiDAR itself and lets its GUI start and stop
   the motor; that is right for a bench tool, and not the model for a
   running robot.
2. **Read-only.** Clients receive data; they send no commands. Driving or
   changing the robot from a computer is a separate, later decision, and
   only ever through the safety arbiter, never around the remote's arm
   switch ([`motor-design.md`](motor-design.md) §4).
3. **A client never slows the robot.** The control loop never waits on the
   network, and nothing a client does reaches it. Each client is sent the
   newest data; what a slow client cannot take is dropped (latest wins,
   bounded buffers, non-blocking sockets). A client connecting, stalling or
   vanishing mid-run changes nothing for the robot.
4. **Every item carries its measurement time, on the Pi's steady clock.**
   The client then draws each scan from the pose the robot had when the
   scan was taken, which matters once the robot moves. All the libraries
   already stamp on `std::chrono::steady_clock`, so their times compare
   directly. Clients use these times only relative to each other; they mean
   nothing as wall-clock times.
5. **Every item states its frame.** Scans are in the robot frame (REP 103:
   x forward, y left, anticlockwise positive), poses in the odometry frame.
   The client places a scan using the pose and the sensors' mounting
   offsets, which come from the rig's profile
   ([`rig-profiles.md`](rig-profiles.md)) once those sections exist.
6. **One message format.** The LiDAR viewer's newline-delimited JSON over
   TCP (`python/heron/link/wire.py`, port 5555) is extended with odometry
   messages, not replaced by a second format.

## Known gaps

| Gap | Change, when the server is built |
|---|---|
| `OdometrySnapshot` carries `age` (from the reading to the call), not the reading's own time | Add the reading's timestamp, as `Scan` has. Until then it can be derived as call time minus `age` |
| `wire.py` has scan messages only | Add an odometry message, and a hello that lists what the robot publishes |
| Mounting offsets for the LiDAR and the OTOS | The profile's `[lidar]` and `[odometry]` sections, added by those workstreams |

## Bandwidth

Not a constraint. A scan is a few kilobytes at about 7–10 Hz; `wire.py`
estimates the worst case (about 530 points at 7.3 Hz) at well under
100 KB/s, not measured. Odometry is read at 100 Hz but can be sent at
20–50 Hz. Comfortable over the Pi's Wi-Fi.
