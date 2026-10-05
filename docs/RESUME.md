# Resume points, by workstream

Work in this repository is split into workstreams that pause and resume
independently. Each keeps its own resume file, and work on one never edits
another's. Start from the workstream you are picking up.

| Workstream | Resume file | State | Pick up at |
|---|---|---|---|
| LiDAR (RPLIDAR A1M8) | [`RESUME-lidar.md`](RESUME-lidar.md) | paused 2026-10-04 at a clean stopping point | lifecycle hardening (its §9) |

Planned, not started: odometry (I²C), remote control (human control), and,
after those, the robot design that ties them together.

## Starting a new workstream

1. Create `docs/RESUME-<area>.md`, written as a handover: what exists, how to
   run it, and where to pick up.
2. Add a row to the table above.
3. Name its other documents `docs/<area>-*.md`, so they never collide with
   another workstream's.
4. Add a short section for it under "Workstreams" in `CLAUDE.md`.
