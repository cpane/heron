# Resume points, by workstream

Work in this repository is split into workstreams that pause and resume
independently. Each keeps its own resume file, and work on one never edits
another's. Start from the workstream you are picking up.

| Workstream | Resume file | State | Pick up at |
|---|---|---|---|
| LiDAR (RPLIDAR A1M8) | [`RESUME-lidar.md`](RESUME-lidar.md) | paused 2026-10-04; one session 2026-11-08 (navigation's Inbox entry: a loose micro-USB) | park the motor while unused (its §9a), then lifecycle hardening (§9b) |
| Remote control (FlySky iBUS) | [`RESUME-remote.md`](RESUME-remote.md) | paused 2026-10-12 at a clean stopping point | optional checks only (its "Picking up") |
| Odometry (SparkFun OTOS, I²C) | [`RESUME-odometry.md`](RESUME-odometry.md) | paused 2026-10-18 at a clean stopping point | optional checks only (its "Picking up") |
| Motor controller (Arduino, 4 motors) | [`RESUME-motor.md`](RESUME-motor.md) | bench rig with mecanum wheels drives every move on the floor 2026-11-03; teleop recovers from an Arduino unplug 2026-11-08 | re-measure the bench dead band (its "Picking up next", step 0), then the mini robot |
| Telemetry (live data to other computers) | [`RESUME-telemetry.md`](RESUME-telemetry.md) | started 2026-10-28, design proposed | reviewing the design (step 1) |
| Builder (scripts and guide for builders) | [`RESUME-builder.md`](RESUME-builder.md) | `wheel_check.sh` run on the bench rig 2026-11-03, passing | the next builder script |
| Navigation (obstacle avoidance) | [`RESUME-navigation.md`](RESUME-navigation.md) | started 2026-11-04, design accepted (D1–D5, P1–P12), C++ interfaces accepted (I1–I9), N2 done (maths and simulator, checked against a recording), N3 done (pose and perception), N4 begun (the limiter's core, Mac) | the limiter on the Pi, then N4's scenarios |

Planned, not started: the robot design that ties the workstreams together. The three sensor workstreams are paused; the motor controller is next, since the robot cannot move without it.

## Starting a new workstream

1. Create `docs/RESUME-<area>.md`, written as a handover: what exists, how to
   run it, and where to pick up. End it with an empty `## Inbox` section,
   copied from another resume file, for facts other workstreams find.
2. Add a row to the table above.
3. Name its other documents `docs/<area>-*.md`, so they never collide with
   another workstream's.
4. Add a short section for it under "Workstreams" in `CLAUDE.md`.
