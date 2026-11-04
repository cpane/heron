# Navigation workstream — resume point

Last updated 2026-11-04. Started 2026-11-04 with its design; nothing
built, and it waits on other workstreams first.

## Read this first when resuming

**Phase N0 done: the design is agreed.** Heron is to see obstacles, slide
around them and return to its course, in a new SwC mode, Navigate. Five
layers, each its own library: sensors, pose, perception (a rolling
obstacle map), a safety limiter on the raw LiDAR points, and a local
planner.

| Document | What it holds |
|---|---|
| [`navigation-design.md`](navigation-design.md) | The design: goal, decisions, layers, the obstacle map, modes, testing, prerequisites, limits, a later ROS 2 path, phases |
| [`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md) | The LiDAR's measured behaviour (LiDAR workstream's doc) |
| [`odometry-otos-reference.md`](odometry-otos-reference.md) | The OTOS's measured behaviour (odometry workstream's doc) |
| [`motor-design.md`](motor-design.md) §4 | The safety arbiter and SwC, which the limiter and Navigate fill in (motor workstream's doc) |
| [`telemetry-design.md`](telemetry-design.md) | How each layer will be watched live (telemetry workstream's doc) |

The plan was reviewed as a page first, with the user's comments answered
there; the design doc is now the source of truth.

## Decided (2026-11-04)

D1 to D4 in `navigation-design.md` §2, accepted as recommended: Navigate
is held, letting go stops; position since start-up from the OTOS for
now; avoid by sliding, keeping the heading; our own code, not ROS 2 and
Nav2, on this hardware (§11 sketches a later move for a more powerful
Heron).

## Picking up

Phase N1, the prerequisites (`navigation-design.md` §9), is work in
other workstreams; check each one's resume file for where it stands:

1. The bench rig, which carries navigation (a small robot on mecanum
   wheels since 2026-11-03), with the LiDAR and OTOS mounted, and their
   mounting poses in its profile (motor; shared `rig-profiles.md`).
2. The robot's outline and wheel geometry in the profile (shared).
3. The speed loop closed with the OTOS (motor).
4. LiDAR restart, unplug and replug (`RESUME-lidar.md` §9).
5. Telemetry steps 2 to 4, and an odometry channel (telemetry).

Work this workstream can do before those: phase N2, the simulator and
replay test bench on the Mac, which needs no robot.

## Still open

`navigation-design.md` §13: unknown cells, the robot's outline in the
planner, and library names. Settled 2026-11-04: the bench rig carries it.
