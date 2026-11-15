# Navigation workstream — resume point

Last updated 2026-11-14: N1's profile complete for the bench rig, the
A1M8's range noise measured; `heron_robot` step 1 (Manual and Assisted)
built and checked on the block and on the floor: Assisted stops short of
a box at every speed and slides along it, once the motors' dead band was
modelled (`min_speed`). Sideways measured: slow, drifting, needing a
large command; design §19 (P13 to P18: a measured motion table, heading
hold, plausibility, surface profiles per floor) proposed for review
before step 2 (https://claude.ai/artifact/ECtmNj6ostveAPR7jyvpBU). Started 2026-11-04. **Every navigation library
is built and passes in simulation on the Mac and the Pi, the limiter
(N4) and the planner (N5) included; what is left is `heron_robot`,
whose profile sections are now measured on the bench rig.**

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

## Decided (2026-11-11)

- **One shared `[geometry]` section** (user): navigation's keys
  (length, width, max_speed, max_turn_rate, accel, brake; design §10) go
  in now; the mixing's wheel keys (wheel radius, track, wheelbase) are
  added to the same section by motor when the mixing needs them, so
  `max_speed` is held once.

## Decided (2026-11-13)

- **`[odometry]` for the bench rig** (Chris): `linear_scalar = 0.908`,
  for the hardwood floor at 14–15 mm, and `angular_scalar = 0.991`, from
  motor-driven spins; both in `profiles/bench.conf` with how they were
  found.

## Decided (2026-11-14)

- **`brake = 2.2`** (Chris), the shortest of three stops as
  `rig-calibration.md` §8.6 says, not the design's 1.0: the limiter's
  0.3 s reaction term adds 126 mm at full speed (0.42 m/s), more than
  the longest stop measured (110 mm). **Replaced the same day**, below.
- **`brake = 0.7` and `min_speed = 0.16`** (Chris, after the floor
  check): the motors coast at zero, so `brake` is the coasting rate,
  measured on the floor; and speed is not in proportion to the command
  (the firmware's dead band), so `heron_robot` models it with a new,
  optional `[geometry] min_speed`. 1.1, the middle stop, was tried
  between and changed nothing (item 3).
- **Under `min_speed`, a cut stops and the slow zone alone goes at
  `min_speed`** (Chris agreed, "go ahead with 1 and 2"): the stop is
  early rather than late, 7 to 10 cm short of the stop line. Stopping
  nearer it needs the limiter to report the cut and the slow zone
  apart.
- **Design §19, P13 to P18 accepted** (Chris): a measured motion table
  per floor, heading hold, the planner on commands, plausibility, the
  simulator by the table, surface profiles sorted by drive with a
  library of known surfaces. I17 to I22 (the interfaces) accepted the
  same day, I18 revised to points as measured. Reviewed on https://claude.ai/artifact/ECtmNj6ostveAPR7jyvpBU.

## Picking up

1. **N1's profile sections: built 2026-11-11** (`navigation-cpp-design.md`
   §7.1, "As built"). `heron_profile` reads `[geometry]` (one shared
   section), `[lidar]`, `[odometry]` and `[navigation]`, each optional,
   all-or-nothing, each with a `[<section>.calibration]` record;
   `rig_profile show` prints them; `rig_profile_test` passes on the Mac
   and the Pi (67 checks), and every host test still passes.
2. **The N1 measurements on the bench rig** (`rig-calibration.md` §8,
   not yet run): needs the user at the bench. §8.3 (the LiDAR's yaw) on
   AC; §8.4 to §8.6 on the floor, the speeds and brake driven with
   `heron_teleop` on the power block through its buck (item 6), which
   holds the LiDAR up while driving. Each section goes into
   `profiles/bench.conf` once all its numbers are measured, how they were
   found in its `[<section>.calibration]`; `accel` and the margins start
   from design §10's values. Also the A1M8's range noise (design §18, 6): the spread of
   repeated ranges to a still wall.
   **Started 2026-11-11** [ours, Chris with a tape; sheet
   https://claude.ai/artifact/Sbkk3VF5oHFHqoavtKAJp6]. §8.2: length and
   width 190.5 mm, the outline centred on the wheelbase centre; LiDAR x
   −8 mm, y 0, scan plane 270 mm above the floor; OTOS x 0, y 0. §8.3:
   the LiDAR's housing faces the robot's **back**, `yaw_deg` 180 (box
   ahead +177 and −177, box left −91), though it looked forward. `[lidar]`
   is in `profiles/bench.conf`. Waiting: `[geometry]` needs §8.6's
   speeds and brake (`length = 0.1905`, `width = 0.1905` until then);
   `[odometry]` needs §8.4's yaw and §8.5's scalars (x 0, y 0). Found:
   - **The blind circle fits with 10.45 mm to spare** [computed by hand;
     `checkBlindSpot()` agrees, 2026-11-14: fits at `stop_margin` 0.040,
     fails at 0.035]: 126.8 mm round the LiDAR at x −8 mm
     reaches 134.8 mm behind the centre, against a stop outline of
     95.25 + 50 = 145.25 mm. A `stop_margin` under about 40 mm fails it.
   - **The scan plane is at 270 mm**, higher than the robot is long:
     anything lower is invisible to it (design §6.2 assumes every
     obstacle reaches the plane). Floor checks need boxes taller than
     27 cm. Whether to mount the LiDAR lower is open, for Chris.

   **§8.4 done 2026-11-11** [ours, `odometry_adapter`, pushed by hand]:
   forward reads +x, left +y, turning left raises the heading. So
   `[odometry] yaw_deg` = 0, the axes are not mirrored, the heading's
   sign is right (design §18 item 1 settled for the bench rig).
   - **On carpet the OTOS under-reads about 27%, silently** [ours, one
     run each way]: about 405 mm pushed read 297 mm forward and 275 mm
     left, status 0x00, no warnings. On a board on the same carpet:
     517 mm and 496 mm for 500 mm. Sensor 14–15 mm above the floor, tape
     off. Heron's pose, map and course would be off by as much on that
     carpet: hard floors only, until the odometry workstream says
     otherwise (its Inbox).

   **§8.5 linear done 2026-11-11** [ours, `otos_move`, hardwood floor,
   five pushes of 1000 mm along a straight edge, forward each time]:
   reported 1087.4, 1108.1, 1104.6, 1108.6, 1100.2 mm; mean 1101.8, +10.2%,
   sd 8.7 mm; correcting scalar 0.902 to 0.920, mean **0.908**; heading
   change within ±1.31°, direction −88.3 to −89.6, status 0x00. Proposed:
   `linear_scalar = 0.908`, for this floor and height only, with a
   caveat: the reference calls more than 5% a sign of poor tracking
   [community], and the odometry workstream's sled read +0.2% on its wood
   floor [ours]. Agreed with Chris 2026-11-13.

   **§8.5 angular done 2026-11-13** [ours, `otos_spin`, hardwood,
   spun by `heron_teleop`, out and back]. Three sessions, 20 legs, every
   one long or about zero (−0.06% to +2.36%), mean +0.98%. The last three
   trials, stopped and squared up by hand at the turnaround, 5 turns each
   way: legs +1.00, +0.68, +0.64, +0.86, +1.04, +1.14%, mean **+0.89%**, sd
   0.20%; `angular_scalar` = **0.991**. Found on the way:
   - **The turnaround's lining up is the trial's error:** the probe
     takes the turnaround to be on the mark, so letting the robot run
     on 20–40° before reversing read +1.1 to +2.4% on both legs. Squared
     up by hand there, the spread fell to 0.20%. `rig-calibration.md`
     §8.5 now says so.
   - **Driven spins read +0.9%; the odometry workstream's hand-spun sled
     read −0.20%** [ours, both]. Why is not known; motor vibration on
     the gyro is a guess [unverified]. Told odometry (its Inbox).
   - Tilt warnings (0x01) in most trials, none when the robot was kept
     flat at the turnaround; the heading is gyro only.

   `[odometry]` is in `profiles/bench.conf` (x 0, y 0, yaw_deg 0, 0.908,
   0.991), `rig_profile check` passes.

   **§8.6 done 2026-11-14** [ours, hardwood, `heron_teleop`, throttle
   at the top, power block through its buck]:
   - `max_speed` **0.42 m/s**: 2 m timed after a 1 m run-up, 4.98, 4.73,
     4.63 s (0.40, 0.42, 0.43 m/s), quicker each run.
   - `max_turn_rate` **3.4 rad/s**: `otos_spin`, about 5 turns at full
     stick, peaks 200, 197, 198, 200°/s, × 0.991 = 196°/s, 3.42 rad/s,
     rounded down. Not the gyro's limit: the sled was read at 544°/s.
   - `brake` **2.2 m/s²**: SwA disarmed at a mark at full speed, stopped
     40, 80, 110 mm past it; the shortest, 0.42² / (2 × 0.040). The
     spread is the switch's timing (0.1 s is 42 mm).

   `[geometry]` (length and width 0.1905, the three, `accel` 0.6 from
   design §10) and `[navigation]` (design §10's 0.05 and 0.25, not
   measured) are in `profiles/bench.conf`; `rig_profile check` passes.
   **N1's profile is complete for the bench rig.** `accel` and the
   margins are tuned later.

   **The A1M8's range noise, 2026-11-14** [ours, `lidar_record --mode 1
   --scans 200`, Express as the robot runs it, robot still and disarmed,
   its back square to a flat painted wall, so the wall at the sensor's
   0°; analysed over ±10°; captures `captures/noise{030,100,200}-sdk1_*`,
   git-ignored]:

   | Tape to the spin axis | LiDAR at 0° | Same bearing, revolution to revolution (sd) | Worst off a line | No return |
   |---|---|---|---|---|
   | 425.5 mm (16¾ in) | 426.5 mm | 0.41 mm | 1.1 mm | 0% |
   | 1000.1 mm (39⅜ in) | 1004.0 mm | 0.65 mm | 3.0 mm | 0% |
   | 2000.3 mm (78¾ in) | 2009.4 mm | 1.76 mm | 8.6 mm | 0% |

   - **Far under the simulator's 5 mm** at the limiter's ranges; the
     ranges come in whole millimetres, so at 0.43 m the noise is the
     sensor's step. Every sample quality 188.
   - **The LiDAR reads 1, 4 and 9 mm over the tape** (0.2, 0.4, 0.5%):
     possibly a small scale error, possibly the tape held by hand
     [unverified].
   - **The LiDAR's x may be nearer 0 than −8 mm** [tape, a few mm]: at
     the first placement the spin axis was 425.5 mm from the wall and
     the back wheels 330 mm (13 in), 95 mm apart, which is half the
     outline. The profile keeps −0.008; at 0 the blind circle has more
     room, not less.
   - Told LiDAR (its Inbox): the findings have no range noise.
   - **Not done:** scenario 2 (`nav_scenario_test`) re-run with the
     measured noise, about 1 mm at its 0.6 to 1.5 m, to see how much of
     the margin's erosion (23.5 to 35.6 mm kept of 50 with 5 mm) is
     left. That decides how urgent rule 4's alternative is (item 5).
3. **`heron_robot`** (`navigation-cpp-design.md` §7, §8.7), in three
   steps, each its own PR (agreed with Chris 2026-11-14): 1, Manual and
   Assisted; 2, Navigate with `CourseKeeper` and `LocalPlanner`; 3, the
   telemetry channels (design §11). Checks: on the block a box stops the
   wheels; on the floor it stops short of a box at walking pace; then
   round a box and back (design §15).

   **Step 1 built 2026-11-14** (§7.4 as built): `heron_robot_core`
   (`src/robot/RobotCore.h`: the mode, the units, the settings from the
   profile, `assistedRefusal()`, `decide()`), `cpp/tools/HeronRobot.cpp`,
   `robot_test` (passes on the Mac and the Pi; Manual gave `heron_teleop`'s
   wheels for 5000 random frames). Navigate stops (`NAVIGATE_NOT_BUILT`).
   **Dry run on the bench rig, 2026-11-14** [ours, `heron_robot
   --dry-run`, run by Chris in a terminal on the Mac, robot on the hardwood]:
   - Start-up: profile, OTOS, LiDAR, "Assisted: ready".
   - **SwC: up reads Low, middle Mid, down High:** up is Manual, middle
     Assisted, down Navigate, with no flag (design §18 item 4 settled).
   - Arming as `heron_teleop`; Manual full forward at throttle 0.61 gave
     +0.60 on all four wheels.
   - **Assisted, the LiDAR's points land in front** (the yaw 180 applied):
     a wall ahead read `nearest +0.46 m`, CLEAR, just beyond the slow
     zone (0.395 m). At +0.22 m, full forward 0.63 came out 0.18
     (SLOWED; 0.075 / 0.25 = 0.30 of it, by hand 0.19); full back
     unchanged. At +0.13 m, inside the stop margin: BACKING_OFF (the
     stick not tried there). Scans 15 to 156 ms old.
   - `heron_teleop`'s autostart is off for this (Chris, sudo, in a Mac
     terminal); turn it back on after: `scripts/teleop_autostart.sh
     <user>@<pi> on`.

   **On the block, wheels turning, 2026-11-14** [ours, `heron_robot`,
   motors live, the box held at the LiDAR's height, Chris watching the
   wheels]: Assisted, box about 16 cm from the front wheels (0.26 m from
   the centre), forward slowed to 0.46 of the stick (0.115 / 0.25, as
   computed), all four wheels forward, slowly. The box brought in: 0.27 m
   SLOWED (+0.97 to +0.47), 0.18 m CUT (+0.35 to +0.04), 0.13 to 0.14 m
   BACKING_OFF, out 0.00 with the stick pushed: the wheels slowed, then
   stopped. With the box close, back (−0.26) passed unchanged. Manual
   (SwC up) with the box there: forward +0.20 to the wheels, turning.
   Motors OK, no resets.

   **On the floor, 2026-11-14** [ours, `heron_robot`, hardwood, a box
   taller than 27 cm about 1.5 m ahead, Chris driving and measuring
   from the front wheels; `--lines 10` from run 4, a status line every
   0.1 s with the OTOS's speed]:

   | Run | Model | Speed at the box | Stopped, front wheels to box | Past the stop line |
   |---|---|---|---|---|
   | 1, stick 0.2 | proportion, brake 2.2 | 0.08 m/s | 3.8 to 5.1 cm (LiDAR 4.5) | 5 mm |
   | 2, stick 0.6 | proportion, brake 2.2 | 0.25 to 0.3 | 1 to 2 cm (LiDAR 0.5) | 45 mm |
   | 3, stick 0.6 | proportion, brake 1.1 | the same | LiDAR 0.5 cm, as run 2 | 45 mm |
   | 4, stick 0.6 | `min_speed`, brake 0.7 | 0.28 | 10 to 11 cm (LiDAR 12.5) | none, 7 cm short |
   | 5, stick 1.0 | `min_speed`, brake 0.7 | 0.36 to 0.42 | 13 cm (LiDAR 8.5) | none, 3.5 cm short |

   - **Runs 2 and 3 overshot because the robot did not slow as told.**
     Between the slow zone and the box it kept about 0.3 m/s while the
     command fell to 0.07; `brake` changed nothing, since near the box
     the slow zone, not `brake`, sets the command. Manual at three
     throttles, the OTOS's speed held 2 s: out 0.20, 0.39, 0.59 gave
     0.19, 0.26, 0.30 m/s, where proportion said 0.08, 0.16, 0.25.
     `speed = 0.42 × (0.38 + 0.62 × out)` fits within 0.02: the
     firmware lifts any wheel command above zero to `min_duty`, about
     90 of 255, and on the floor speed follows the duty. So the least
     the robot moves at is about 0.16 m/s.
   - **The motors coast at zero**, at 0.6 to 0.8 m/s² (the OTOS's
     speed after letting go), and both overshoots work back to 0.7.
     Run 5 rolled 7 cm from 0.30 m/s; 0.7 predicts 6.4.
   - **The fix** (`RobotCore.h`, "Units"): `toTwist()` and
     `toBodyVelocity()` model the floor; under it a cut stops and the
     slow zone alone goes at `min_speed`; `brake` 0.7, `min_speed` 0.16.
   - **The slide, run 6** (stick 0.6): stopped at the box with forward
     held, forward stayed 0.00 for about 9 s while sideways passed
     (−0.03 to −0.60); forward came back when the box's corner was
     0.15 m to the side (half the width plus the margin, 0.145), and it
     went on past.
   - **Pure strafe barely moves it** [ours, one run]: wheels ±0.60
     alone, the OTOS read 0.01 to 0.06 m/s sideways and the corner moved
     about 3 cm in 6 s; forward and sideways together gave 0.13 to 0.16
     m/s sideways. Wheel slip is a guess. The limiter assumes a strafe
     is faster than this, the safe side.
   - **The OTOS reads −0.02 to −0.05 m/s with the robot still** after a
     run, for as long as it was watched; and 0.05 to 0.09 m/s sideways
     at full speed straight ahead. Told odometry (its Inbox).
   - **Told motor** (its Inbox): on the floor the smallest stick gives
     about 40% of full speed.
   - Start-up failed once, the Pi up 2 minutes: the OTOS's scalars read
     back 0xA4 0x00 for 0xA4 0xF7 (the angular one not taken); Assisted
     refused, as it should. The next start passed, and every one after
     [unexplained, once].

   **Sideways, 2026-11-14** [ours, `heron_robot --lines 10`, Manual,
   hardwood, log `captures/strafe-2026-11-14.log`, Chris driving; the
   first try had the motor battery off, so nothing moved and
   `heron_robot` still said `motors OK`]:
   - Strafe right at 0.3: nothing moved. At 0.6: 0.13 to 0.19 m/s right,
     creeping back 0.05 to 0.16. At 1.0: 0.28 m/s right. Left at 0.6:
     0.14 m/s left. About half forward's speed for the same command.
   - Strafe at 0.6 for about 4.4 s on a tape: 32 in (81 cm) right, about
     7 in (18 cm) back, front 34.5 in and back 30.75 in from the tape:
     about 30° clockwise, gradually. The OTOS added up to 62 cm right
     and 32 cm back, robot frame; half the backward is its standing
     offset (−0.04 m/s).
   - 45° forward-right at 0.6 (wheels 1, 0, 1, 0): 0.12 to 0.17 forward,
     0.11 to 0.24 right. Forward 0.59 with 0.16 right: 0.32 forward, 0
     to 0.09 right. Forward then sliding round to right: 0.15 to 0.23
     right, no better than from rest.
   - **The OTOS gave nonsense after lifts**, called fresh: −2.6 m/s and
     −3.4 to −3.6 m/s forward for seconds (all of the strafe left run),
     −1.23 m/s standing disarmed; the sideways reading stayed sane.
     Told odometry (its Inbox).
   - The remote's right stick reads +0.04 to +0.23 forward when pushed
     fully right: its trim or centring [unverified].
   - **A planner told `min_speed` per axis** (tried, not kept): scenario
     1 passed 9 of 10 boxes, stopped for good 368 mm short of the tenth
     (dead ahead), and ended 14 to 16 cm off the line, not 10. Per axis
     is the wrong model: the dead band is per wheel, and sideways has
     its own.

   **The motion table, hardwood, 2026-11-14** [ours, `heron_robot
   --lines 10` (with the turn rate, heading and OTOS warnings), Manual,
   log `captures/table-hardwood-1-2026-11-14.log`, Chris driving; 25
   runs, one repeated (out of room)]: each command held 2 s, the settled
   speed turned into the robot frame. The table is design §19.1, and the
   `hardwood-1` example in §19.2 holds every point.
   - Forward 0.19, 0.27, 0.31, 0.34, 0.39 m/s at 0.2 to 1.0, barely
     turning. "1 o'clock" on the stick came out at about 15° and its
     sideways part was lost: plain forward. "2 o'clock" came out at 45°.
   - Strafe right: nothing at 0.2, 0.06 at 0.4, 0.15 at 0.6, 0.21 at
     0.8, 0.27 at 1.0, turning clockwise 8 to 12°/s at 0.4 to 0.8. Left:
     0.04 at 0.6 (a quarter of right's), 0.25 at 1.0. Backward 0.31 at
     0.6.
   - Taped strafe right, 0.6, 4.9 s: front wheel centres 29 in from the
     tape, back 24 in, the back-left 13 in back along it; axles 4.5 in
     apart, so a turn of 65° or more. The OTOS said 68°.
   - **The OTOS's velocity is in the odom frame** (design §18 item 5,
     settled): after a turn, straight runs printed a sideways part that
     turning by −heading removed, and the taped strafe agreed only read
     that way (39 cm back and 52 cm right against the tape's 30 and 61;
     read as robot frame, 57 back and 8 right). `PoseTracker` passed it
     on as robot frame; fixed (it turns it by −heading), `pose_test`
     checks it. Every earlier speed in this file was in the odom frame:
     the strafe's "creeping back" and the "sideways while driving
     straight" were the robot's turn. Told odometry (its Inbox).
   - Yesterday's taped strafe, with the axles now measured: front 3.75
     in further than back over 4.5 in, about 56°, not the 30° written
     then.

   **§19.4 step 3 built 2026-11-14** (`navigation-cpp-design.md` §7.5,
   "As built"): `heron_motion`, surface profiles and the library
   (`profiles/surfaces/bench-mecanum/hardwood-1.conf`, `bench.conf`
   naming drive and surface), plausibility (P16), Assisted through the
   table, `heron_robot --surface`. `motion_test`, `rig_profile_test`,
   `pose_test`, `robot_test` pass on the Mac and the Pi, all 26 host
   tests. Deployed, not yet run on the floor.

   **Next: step 1's floor checks again, through the table** (design
   §19.4 step 3's end): Assisted at a box straight on at 0.6 and full
   (expect it to stop short as before, a little earlier at full since
   the table says it goes 0.39, not 0.42); a strafe right toward a box on
   the right (it should stop short; toward a wall ahead it now refuses a
   strafe, `navigation-cpp-design.md` §7.5 as built); a lift, then
   Assisted: `POSE_NOT_FRESH` until it settles. `heron_teleop` off, `ssh
   -t <user>@<pi> '~/heron/bin/heron_robot --lines 10'`, the start-up
   should print `Surface: hardwood-1` and "Assisted: ready, through the
   surface's motion table". Then design §19.4 step 4, heading hold.

   **Step 2, Navigate** (`navigation-cpp-design.md` §7, §8.7), after §19:
   `CourseKeeper` and `LocalPlanner` into `heron_robot`, SwC down; the
   floor check is round a box and back to the course (design §15). Its
   own PR. To drive: robot on the hardwood, `heron_teleop` off
   (`scripts/teleop_autostart.sh <user>@<pi> off`, sudo), `ssh -t
   <user>@<pi> '~/heron/bin/heron_robot --lines 10'` in a Mac terminal;
   SwA on, then the throttle to the bottom to arm (WAIT until then). SwC
   up takes over at any time. **Deploy with `scripts/deploy_cpp.sh
   <user>@<pi>` and no binary:** with a binary it also runs it, with no
   flags; on 2026-11-14 that started `heron_robot` with its motors live
   when a dry run was meant (no wheel turned). Afterwards,
   `heron_teleop`'s autostart back on: `scripts/teleop_autostart.sh
   <user>@<pi> on` (sudo).
4. **The planner's weights** are tuned to scenario 1's ten runs with
   little to spare (154 mm against 150, 85 mm against 100; §8.9): the
   floor settles them. `plan()` takes 15.8 ms of its 20 ms budget on
   the Pi, alone.
5. **Rule 4's alternative** (a margin per side, §6.2's end) is to be
   weighed later, with the user: the range noise is now measured
   (2026-11-14, under 2 mm to 2 m), and Heron has yet to drive near walls
   in Assisted (design §18, 6 and 7). Do not change rule 4 before then.
6. **Power for the LiDAR.** The power block straight into the Pi fails:
   the LiDAR goes silent for seconds while its motor runs, and the Pi's
   under-voltage flag does not catch it (2026-11-08); the block's own
   5 V reads 4.89 V at near-zero load. **Through a PD trigger (12 V) and
   a 5.2 V buck it holds** (2026-11-10 and -11, folded from LiDAR's
   Inbox) [ours]: 1499, 1498 and 1500 of 1500 requests in three soaks,
   two with the robot driving, a scope on the LiDAR's 5 V never below
   4.70 V. Still thin: the LiDAR's own wire reads 4.85 to 4.89 V, and
   LiDAR plans a dedicated feed from the buck (`rig-profiles.md` §1;
   findings traps 12 and 13). Unit A's adapter board is back on the
   bench rig and passed; whether it was faulty is not established.
   Check a supply with `python/probes/lidar_07_requests.py`, motor
   running. Give the LiDAR's cable strain relief before the robot
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

- **N1's numbers are measured** for the bench rig (2026-11-14), except
  `accel` and the margins, which are the design's starting values; and
  the strafe speed, for which `max_speed` and `min_speed` stand in.
- **Assisted stops 7 to 10 cm short of the stop line** (2026-11-14):
  under `min_speed` a cut stops. To stop nearer it, the limiter would
  report the cut's scale and the slow zone's apart; not asked for yet.
- **The turn's floor** is `max_turn_rate × min_speed / max_speed`, by
  the same dead band [unverified: not measured on its own]. `odometry_adapter` prints no velocity, so §18's velocity
  frame cannot be checked yet.
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

Nothing waiting.
