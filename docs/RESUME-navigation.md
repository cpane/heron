# Navigation workstream — resume point

Last updated 2026-11-09. Started 2026-11-04. **Every navigation library
is built and passes in simulation on the Mac and the Pi, the limiter
(N4) and the planner (N5) included; what is left is `heron_robot`,
which waits on N1's profile sections.**

## Read this first when resuming

**Phase N0 is done: the goal, D1 to D5 and the design (P1 to P12) are
accepted. The C++ interfaces (`navigation-cpp-design.md`, I1 to I9) are
accepted. Phase N2 is done: `heron_geometry` and `heron_nav_sim`, tested on
the Mac and the Pi, the simulator checked against a recording. Phase N3 is
done: `heron_pose` and `heron_perception`, tested on the Mac and the Pi
(`navigation-cpp-design.md` §5.3). Phase N4's limiter is built
(`heron_limiter`, §6.1), and N5's planner (`heron_planner`, I10 to I16,
as built §8.9); scenarios 1 to 5 pass. Left: `heron_robot`, the program
that joins them, and everything on the floor.** Heron is to see obstacles, slide
around them and return to its course, in a new SwC mode, Navigate. Five
layers, each its own library: sensors, pose, perception (a rolling
obstacle map), a safety limiter on the raw LiDAR points, and a local
planner.

| Document | What it holds |
|---|---|
| [`navigation-design.md`](navigation-design.md) | The design: modes, decisions and proposals, frames, the five libraries and their threads, pose, the obstacle map, the limiter, the planner, `heron_robot`, profile sections, telemetry channels, the simulator and tests, steps N1 to N7, a later ROS 2 path |
| [`navigation-cpp-design.md`](navigation-cpp-design.md) | The C++ interfaces: each library's public types and functions, the simulator, the control cycle, tests by phase |
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

## Decided (2026-11-08)

- **P1 to P12 accepted** as proposed (`navigation-design.md` §2.2).
- **The interfaces are written before any code**
  (`navigation-cpp-design.md`), for N2 to N4; the planner's in N5.
- **N2 first, on the Mac; N1 alongside it** at the bench, which the user
  is always at. The bench rig is a complete Heron, LiDAR and OTOS
  mounted; their positions are measured in N1 (design §10's table).
- **I1 to I9 accepted** as proposed (`navigation-cpp-design.md` §1),
  after the user's review on the page. `Pose2` and `Vec2` keep the 2
  (§2.1 says why).
- **N2 built with three changes from the accepted interfaces**
  (`navigation-cpp-design.md` §3, end): `Base` takes a start time;
  `LidarConfig` gained the housing's yaw, the period's spread, the sync
  angle and the gap; the simulated OTOS moved to N3. Made while
  building; the user merged them (PR #39).
- **The simulated period spread is 0.2 ms**, from the recording, not
  the findings' 0.8 ms (§3, "The recording").

## Decided (2026-11-09)

- **Rule 4 as built is accepted for the first robot** (user,
  2026-11-08): every point in the margin is held by the gap-floor rule
  (`navigation-cpp-design.md` §6.2). A margin per side is the
  alternative, to weigh later with its pros and cons (design §18, 7).
- **I10 to I16 accepted** as proposed (user, on the review page): the
  planner checks a distance field, not the limiter's sweep (I10 changes
  P7's wording), keeps 8.3 cm more than the limiter (I11), and so on
  (§8.1).
- **The planner as built, merged** (user, PR #55): five score terms,
  progress toward a point on the line 1 m ahead, at least 1 cm of
  progress, the field interpolated (§8.9). The review page was updated
  to match, at the user's request.

## Picking up

1. **N1's profile sections, with the user.** `heron_robot` needs
   `[lidar]`, `[odometry]`, `[navigation]` and navigation's `[geometry]`
   in `profiles/<rig>.conf`; `heron_profile` refuses unknown sections
   today (`rig-profiles.md` §2). First decide, with the user, the two
   `[geometry]`s: the mixing's planned one (wheel radius, track,
   wheelbase, max speed) and navigation's (length, width, max_speed,
   max_turn_rate, accel, brake; design §10): one section or two names.
   Then the reader change is shared work on its own branch
   (`navigation-cpp-design.md` §7.1 has the structs).
2. **The N1 measurements on the bench rig** (`rig-calibration.md` §8,
   not yet run): needs the user at the bench. §8.3 (the LiDAR's yaw) on
   AC; §8.4 to §8.6 on the floor, the speeds and brake driven with
   `heron_teleop` on the power block, nothing reading the LiDAR. Numbers
   recorded on paper with how they were found until the reader takes
   them. Also the A1M8's range noise (design §18, 6): the spread of
   repeated ranges to a still wall.
3. **`heron_robot`** (`navigation-cpp-design.md` §7, §8.7): Manual and
   Assisted first, `robot_test` (Manual gives `heron_teleop`'s wheel
   values for the same frames), then Navigate with `CourseKeeper` and
   `LocalPlanner`. Checks: on the block a box stops the wheels; on the
   floor it stops short of a box at walking pace; then round a box and
   back (design §15). Which end of SwC is up is checked with
   `ibus_reader monitor` then.
4. **The planner's weights** are tuned to scenario 1's ten runs with
   little to spare (154 mm against 150, 85 mm against 100; §8.9): the
   floor settles them. `plan()` takes 15.8 ms of its 20 ms budget on
   the Pi, alone.
5. **Rule 4's alternative** (a margin per side, §6.2's end) is to be
   weighed later, with the user, once N1 has measured the A1M8's range
   noise (design §18, 6 and 7). Do not change rule 4 before then.
6. **Power for the LiDAR.** On the robot's power block the LiDAR goes
   silent for seconds while its motor runs, and the Pi's under-voltage
   flag does not catch it; on AC it answers every request (2026-11-08,
   `rig-profiles.md` §1). Driving with the LiDAR waits on a 5 V supply
   that holds under its motor, checked with
   `python/probes/lidar_07_requests.py`, motor running, before driving
   on battery. Unit A's own adapter board is faulty: the bench rig uses
   unit B's. Give the LiDAR's cable strain relief before the robot
   drives.

N5's planner (2026-11-09): `planner_test` and `nav_scenario_test` pass
on the Mac and the Pi. Scenario 1 over ten box positions: 154 to 178 mm
to spare on the Pi, back within 85 mm of the line 2 m past, the limiter
cut none of the planner's choices; scenario 4 Blocked 207 mm short. The
accepted four-term score could not pass a box and return to the line
together; the five-term score and three smaller changes are §8.9.
`plan()`: 15.8 ms on the Pi, 0.45 ms on the Mac optimised. Reviewed on a
page (as proposed, then updated as built).

N4's scenarios (2026-11-08): `nav_scenario_test` passes on the Mac and
the Pi (§6.1's table). Scenario 2 found that the first rule 4 ratcheted
the margin down and let Heron creep into a slanting wall (8 of 1000 runs);
the rule now holds every point in the margin (§6.2). Range noise still
erodes the margin, 23.5 to 35.6 mm kept of 50; the A1M8's noise is not
measured.

N4's limiter (2026-11-08): `limiter_test` passes on the Mac, each rule
and blind row by hand, and 2000 random scenes in which no limited command
brings a point within 4 cm of the body within T (scenes with nothing in
the margin). Rule 4 needed its own design, or a wall in the margin
stopped a slide along it (§6.2). Worst-case call on the Pi: 0.68 ms. A worst-case call: 4.8 ms first, 0.34 ms with
`Sweep`.

N3's results (2026-11-08): `pose_test`, `perception_test` and
`nav_sim_test` pass on the Mac and the Pi. A turning robot's wall, from
the simulator, lands within one cell (25 mm) placed at each point's own
time, 325 mm off (375 on the Pi's run) placed at the scan's end. With the accepted counts one hit blocks an unknown cell; design §6.1
said two scans, and is corrected.

N2's results (2026-11-08): both tests pass on the Mac and the Pi. A
220-revolution `lidar_record` capture (`captures/navsim-sdk1_20261108T185202Z`,
git-ignored) showed the true period spread is about 0.2 ms, not the
findings' 0.8 ms from SDK stamps, so the simulator now uses 0.2 ms.

Phase N1, the prerequisites (`navigation-design.md` §15), is work in
other workstreams; check each one's resume file for where it stands:

1. The bench rig, which carries navigation (a small robot on mecanum
   wheels since 2026-11-03), with the LiDAR and OTOS mounted, and their
   mounting poses in its profile (motor; shared `rig-profiles.md`).
2. The robot's outline and wheel geometry in the profile (shared).
3. The speed loop closed with the OTOS (motor).
4. LiDAR restart, unplug and replug (`RESUME-lidar.md` §9).
5. Telemetry steps 2 to 4, and an odometry channel (telemetry).

Nothing more can be built without them: every library that needs no
robot is done.

## Still open

`navigation-design.md` §18: the OTOS axes once mounted, the Pi 3's time
budget, the grid counts and planner weights, which end of SwC is up,
and the frame of the OTOS's velocity.
Settled 2026-11-04: the bench rig carries navigation.

- **N1's numbers wait on the profile reader.** The measurement
  checklist is `rig-calibration.md` §8 (2026-11-09), not yet run. The
  reader rejects unknown sections, so the numbers wait on paper until N1
  adds them, and the planned `[geometry]` section clashes with the
  mixing's `[geometry]`. `odometry_adapter` prints no velocity, so
  §18's velocity frame cannot be checked yet.
- **`plan()` with every thread running** on the Pi [unverified]: 15.8
  ms alone against a 20 ms budget; `PlannerHealth` reports it once
  `heron_robot` runs (design §18, 2).
- **Navigate cannot turn on the spot:** with the right stick centred
  there is no course, and Heron stops, turning included (§8.9). Fine for
  the first version; the user has not been asked whether to change it.
- **Perception can skip a cell a ray crosses** (Bresenham from cell to
  cell, §8.9): seen only with rays started 0.2 m out, in a test; rays
  from one point covered every cell [unverified on recordings].
- **The simulated no-returns are random; real ones cluster** (runs of
  4.4 to 5.2 samples on average, up to 70). A test that depends on clustering
  needs a recording (§3).

## Inbox

Facts other workstreams found that belong here, waiting to be checked and
folded into this workstream's docs when it resumes (`CLAUDE.md`,
"Workstreams"). Newest last; delete an entry once folded in.

### From LiDAR, 2026-11-10: the power block passes through a buck

[ours] With the Pi on 12 V from the power block through a PD trigger and
a 5.2 V buck, the LiDAR answered 1499 of 1500 requests with the robot
driving, a scope on its 5 V never below 4.70 V; the block's own 5 V reads
4.89 V at near-zero load, which is why it failed. The LiDAR's wire still
reads 4.85 to 4.89 V, so a dedicated feed is planned. `rig-profiles.md`
§1, "The power block through a buck"; findings trap 13. Bears on
"Picking up" 6 (power for the LiDAR).

### From LiDAR, 2026-11-11: unit A's adapter board is back, and passed

[ours] "Picking up" 6 says unit A's adapter board is faulty and the bench
rig uses unit B's. On 2026-11-11 unit A's board was refitted and, on the
supply through the buck, answered 1500 of 1500 requests with the robot
driving (1498 of 1500 disarmed, both misses at motor starts). Whether it
is faulty is no longer established; the bench rig keeps it.
`rig-profiles.md` §1, "Unit A's board again"; findings trap 12.
