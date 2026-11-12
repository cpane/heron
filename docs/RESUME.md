# Resume points, by workstream

Work in this repository is split into workstreams that pause and resume
independently. Each keeps its own resume file, and work on one never edits
another's. Start from the workstream you are picking up.

| Workstream | Resume file | State | Pick up at |
|---|---|---|---|
| LiDAR (RPLIDAR A1M8) | [`RESUME-lidar.md`](RESUME-lidar.md) | paused 2026-10-04; sessions 2026-11-08 (navigation's Inbox entry), 2026-11-10 and -11 (the power path through a buck, a scope, unit A's board) | the soak on the LiDAR's own 5 V feed once wired (its "Picking up"), then park the motor (§9a), then lifecycle hardening (§9b) |
| Remote control (FlySky iBUS) | [`RESUME-remote.md`](RESUME-remote.md) | paused 2026-10-12 at a clean stopping point | optional checks only (its "Picking up") |
| Odometry (SparkFun OTOS, I²C) | [`RESUME-odometry.md`](RESUME-odometry.md) | paused 2026-10-18 at a clean stopping point | optional checks only (its "Picking up") |
| Motor controller (Arduino, 4 motors) | [`RESUME-motor.md`](RESUME-motor.md) | bench rig with mecanum wheels drives every move on the floor 2026-11-03; teleop recovers from an Arduino unplug 2026-11-08; dead band re-measured, wheels on, 2026-11-11 | the mini robot (its "Picking up next", step 1), once it is built |
| Telemetry (live data to other computers) | [`RESUME-telemetry.md`](RESUME-telemetry.md) | started 2026-10-28, design proposed | reviewing the design (step 1) |
| Builder (scripts and guide for builders) | [`RESUME-builder.md`](RESUME-builder.md) | `wheel_check.sh` run on the bench rig 2026-11-03, passing | the next builder script |
| Navigation (obstacle avoidance) | [`RESUME-navigation.md`](RESUME-navigation.md) | started 2026-11-04, design accepted (D1–D5, P1–P12), C++ interfaces accepted (I1–I9), N2 done (maths and simulator, checked against a recording), N3 done (pose and perception), N4 limiter and N5 planner built and passing in simulation (2026-11-09); profile reader takes N1's sections, `[lidar]` measured (2026-11-11) | the N1 measurements on the bench rig: §8.5's angular scalar next (its "Picking up" 2), then §8.6, then `heron_robot` |

Planned, not started: the robot design that ties the workstreams together. The three sensor workstreams are paused; the motor controller is next, since the robot cannot move without it.

## Starting a new workstream

1. Create `docs/RESUME-<area>.md`, written as a handover: what exists, how to
   run it, and where to pick up. End it with an empty `## Inbox` section,
   copied from another resume file, for facts other workstreams find.
2. Add a row to the table above.
3. Name its other documents `docs/<area>-*.md`, so they never collide with
   another workstream's.
4. Add a short section for it under "Workstreams" in `CLAUDE.md`.
