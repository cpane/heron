# Navigation workstream — resume point

Last updated 2026-11-04. Started 2026-11-04; nothing built. **In
progress: the full design, P1 to P12, proposed for review.**

## Read this first when resuming

**Phase N0: the goal and D1 to D5 are agreed; the design that builds it
(P1 to P12) is written and awaiting review.** Heron is to see obstacles, slide
around them and return to its course, in a new SwC mode, Navigate. Five
layers, each its own library: sensors, pose, perception (a rolling
obstacle map), a safety limiter on the raw LiDAR points, and a local
planner.

| Document | What it holds |
|---|---|
| [`navigation-design.md`](navigation-design.md) | The design: modes, decisions and proposals, frames, the five libraries and their threads, pose, the obstacle map, the limiter, the planner, `heron_robot`, profile sections, telemetry channels, the simulator and tests, steps N1 to N7, a later ROS 2 path |
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
Nav2, on this hardware (§16 sketches a later move for a more powerful
Heron). D5, added the same day at the user's request: keep all three SwC
modes, and Manual is exactly today's remote control, untouched by
navigation, so SwC to Manual always takes over.

## Picking up

0. **Review P1 to P12** (`navigation-design.md` §2.2) with the user; record
   what is accepted or changed here and in the design, then N0 is done.

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

`navigation-design.md` §18: the OTOS axes once mounted, the Pi 3's time
budget, the grid counts and planner weights, which end of SwC is up.
Settled 2026-11-04: the bench rig carries navigation.

## Inbox

Facts other workstreams found that belong here, waiting to be checked and
folded into this workstream's docs when it resumes (`CLAUDE.md`,
"Workstreams"). Newest last; delete an entry once folded in.

Nothing waiting.
