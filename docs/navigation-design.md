# Navigation — design

How the LiDAR and the odometry sensor turn Heron from a remote-controlled
base into a semi-autonomous one: it knows where it is, sees obstacles,
slides around them, and goes back to its course. **Goal and decisions
D1 to D4 accepted 2026-11-04; the design below (P1 to P12) proposed
2026-11-04, for review.** Nothing is built.

It builds on what exists and is measured: `Lidar` and its `Scan`
([`lidar-cpp-design.md`](lidar-cpp-design.md),
[`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md)), `Odometry`
([`odometry-cpp-design.md`](odometry-cpp-design.md),
[`odometry-otos-reference.md`](odometry-otos-reference.md)), the drive
code and the safety arbiter planned for SwC
([`motor-design.md`](motor-design.md) §4), and
[`telemetry-design.md`](telemetry-design.md). Numbers marked **[ours]**
were measured on our units. Every other number is a starting value, to be
tuned in the simulator and then on the floor, and says so.

The robot is the **bench rig**: since 2026-11-03 a small robot on
mecanum wheels, driven on the floor or on a block (`rig-profiles.md`).

## 1. What it is

You drive Heron in a direction. As it moves, it sees that it will hit
something in its path, slides around it, and returns to the course it was
on once the obstacle is behind it. If there is no way around, it stops and
says so. A three-position switch picks how much help you get:

| SwC | Mode | What happens |
|---|---|---|
| Up | Manual | As today: the sticks drive the wheels |
| Middle | Assisted | You drive; the limiter slows or stops any part of the motion that would hit something |
| Down | Navigate | You hold the stick in a direction; Heron goes around obstacles and back to that course |

`motor-design.md` §4 calls the third position "Auto, later"; Navigate is
that mode. The safety limiter is its "safety arbiter, Phase 2".

## 2. Decisions and proposals

### 2.1 Decided (2026-11-04)

| # | Decision | Why |
|---|---|---|
| D1 | **Navigate is driven by holding the stick in the direction you want; letting go stops Heron** | You stay in charge; the robot only steers around things. Setting a course and letting go is a later step |
| D2 | **"Where it is" means position since start-up, from the OTOS, for now** | Enough to avoid an obstacle and resume a course. Position on a map of the room is phase N6 |
| D3 | **Avoid by sliding sideways and keeping the heading** | What mecanum wheels are for; keeps a forward LiDAR pointed down the course; makes rejoining the course line simple |
| D4 | **Our own code, not ROS 2 and Nav2,** on this hardware | ROS 2 is heavy for a 1 GB Pi 3, not packaged for this Debian, and would replace the architecture already built. §16 maps a later move |

### 2.2 Proposed (2026-11-04), for review

| # | Proposal | Why | § |
|---|---|---|---|
| P1 | **Five new libraries:** `heron_geometry`, `heron_pose`, `heron_perception`, `heron_limiter`, `heron_planner`, plus `heron_nav_sim` for tests | One job each, tested alone; matches the plan's five layers, with the shared maths in one place | 4 |
| P2 | **Every library has a pure core that takes explicit times,** and a thin threaded wrapper where it needs one | The whole pipeline runs in the simulator in virtual time, faster than real time and repeatable, on the Mac | 4 |
| P3 | **Pose keeps a 2 s history of OTOS readings,** read from `Odometry::getLatest()`; `Odometry` is not changed | Placing a scan's points needs the pose at each point's own time (a scan takes ~137 ms), not just the newest. Another workstream's library stays as it is | 5 |
| P4 | **The LiDAR's yaw stays in `Lidar`** (`mountingOffsetRad`, as now); **its position is applied in perception and the limiter** | The bearing flip and yaw happen exactly once, in the adapter (`lidar-cpp-design.md` §2.1). The adapter knows nothing of the robot's size | 3 |
| P5 | **The obstacle map: an 80 × 80 grid of 5 cm cells, one signed count per cell,** rolling with Heron, in the pose frame | Small (6.4 KB), cheap to update per scan, and keeps unknown apart from free | 6 |
| P6 | **Unknown cells count as blocked within 0.6 m of Heron, free beyond** | Heron never drives into space it has not seen, yet still plans past the edge of what it has | 6, 8 |
| P7 | **The limiter sweeps the robot's outline along the commanded motion, point by point, and cuts only the axes that would hit** | Exact for a rectangle, cheap on 530 points, and lets Heron slide along a wall while the motion into it stops. The same sweep the planner uses | 7 |
| P8 | **The limiter fails closed:** no fresh, covered scan in Assisted or Navigate means stop | A LiDAR that stops answering must not read as "nothing there" | 7 |
| P9 | **The planner samples sideways and forward speeds** in a window around the current speed, keeps the stick's turn rate, and scores clearance, progress, distance off the course line and smoothness | The dynamic window approach, reduced to what D3 needs: heading is kept, so turning is not searched | 8 |
| P10 | **A new program, `heron_robot`;** `heron_teleop` stays as it is until Assisted works | The boot service and the builder's wheel check keep a known-good program while this is built | 9 |
| P11 | **New profile sections:** `[geometry]`, `[lidar]`, `[odometry]`, `[navigation]` | Every robot-specific number in one file per rig, as the motors' are | 10 |
| P12 | **Four telemetry channels:** `pose`, `map`, `plan`, `limiter` | Each arrow in §4's diagram watchable live, by the telemetry recipe | 11 |

## 3. Frames, time and units

| Frame | Origin and axes | Who uses it |
|---|---|---|
| **Robot** | The centre of the wheelbase on the floor; x forward, y left, heading anticlockwise (REP 103) | The outline, the limiter, commands |
| **LiDAR** | The LiDAR's spin axis. `Scan` bearings are already rotated to the robot's axes by `Lidar` (P4); only the position offset is left | Perception and the limiter add it |
| **Pose** (odom) | Where the robot frame was at start-up; fixed to the floor | The pose, the obstacle map, the course line |

- **Placing a point:** a LiDAR point at bearing β, range r becomes
  `(lx + r cos β, ly + r sin β)` in the robot frame, where `(lx, ly)` is
  the LiDAR's position from `[lidar]`. In the pose frame it is then
  rotated and moved by the robot's pose at `scan.timeOf(β)`.
- **The OTOS's own offset** is written to the sensor (`OtosConfig::offset`),
  so `Odometry` already reports the robot frame's pose. Its axis directions
  are to be checked when it is mounted (step N1): a sign error mirrors the
  map without any error.
- **Time:** the steady clock throughout, as every library. A reading's own
  time is `call time − snapshot.age` for odometry, and `Scan::timeOf()`
  for LiDAR points.
- **Units:** metres, radians, seconds inside navigation (telemetry D12).
  Commands to `mix()` stay fractions of full drive; `[geometry] max_speed`
  converts between the two until the OTOS speed loop exists (§15).

## 4. Libraries and threads

```
heron_geometry    pure: Pose2 (x, y, heading; compose, inverse, apply),
                  Vec2, Outline (the robot's rectangle), and the sweep: the
                  first time a point touches the outline moving at a given
                  twist. No I/O, no threads.
  ^
heron_pose        pure PoseHistory (2 s of timestamped poses, interpolated
                  at any time in it); PoseTracker: a thread copying each new
                  Odometry reading into it, getLatest() and poseAt(t).
  ^
heron_perception  pure ObstacleGrid (§6) and placeScan() (points into the
                  pose frame); Perception: a thread building a grid per
                  scan, getLatest() returning an immutable map.
heron_limiter     pure, no thread: Limiter::limit(command, scan, pose,
                  measured velocity, now) -> limited command and why.
                  Called by the control loop every 20 ms.
heron_planner     pure Planner::plan(map, pose, velocity, course, now);
                  LocalPlanner: a thread running it 10 times a second,
                  getLatest() returning the chosen motion and path.

heron_nav_sim     host only: a world of walls and boxes, a simulated LiDAR
                  producing real Scans, a simulated OTOS with drift, and
                  mecanum motion. Links all of the above, for tests.
```

Each row links the libraries it names, publicly; `heron_limiter` and
`heron_perception` link `heron_lidar_types` for `Scan` and
`SectorCheck.h`, `heron_pose` links `heron_odometry`. Private headers sit in `src/<library>/`, public ones in
`include/heron/<library>/`, as the existing libraries do.

The robot program's threads, Navigate mode:

```
              newest scan (raw points skip the map and the planner)
   +-----------------------------------------------------------------+
   |                                                                 v
 Lidar --Scan--> Perception --map--> LocalPlanner --twist--> control loop: Limiter
 7.3/s           per scan            10/s                    50/s (every 20 ms)
 thread          thread                ^      ^                 |  -> mix -> Motors
                    ^ poseAt(t)        |      | course          |
 Odometry ----> PoseTracker ---pose----+      |                 |
 100/s          100/s                         |                 |
 Remote -------------stick, SwC---------------+-----------------+
```

No thread waits for another: each reads the others' newest results
through `getLatest()`, as every library does now. In Manual and Assisted
the planner is idle and the stick goes straight to the limiter (Assisted)
or past it (Manual).

## 5. Pose

`PoseTracker` polls `Odometry::getLatest()` every 10 ms. Each new reading
(a new `seq`) goes into `PoseHistory` at its own time (`now − age`), with
its pose and velocity.

- **`poseAt(t)`** interpolates between the two readings around `t`
  (heading the short way round). Before the oldest reading it fails;
  after the newest, up to 50 ms, it extrapolates with the newest
  velocity; later than that it fails. A failed lookup is a reason, not a
  zero pose.
- **Capacity:** 2 s at 100 readings a second, a fixed ring of 256
  entries; a scan is 137 ms, so a whole scan always fits.
- **Not fresh:** `Odometry` not OK (stale, sensor error), or a seq gap
  over 10 readings. Perception and the planner then stop using new data,
  and the robot channel says why. Tilt and tracking warnings are passed
  on, not acted on, as `Odometry` does.
- **Its frame is the OTOS's,** reset when `Odometry` starts. A restart
  of the OTOS clears the history and the map (§6), since the old frame is
  gone.

## 6. Perception and the obstacle map

### 6.1 The grid

| Property | Value | Why |
|---|---|---|
| Size | 80 × 80 cells of 5 cm: 4 × 4 m | About 2 m each way covers the planner's 1.5 s at walking pace, with room to see round a box |
| Cell | `int8_t` count, starting at 0 | 6,400 bytes for the whole map |
| Hit | +3, up to +12 | A point seen in two scans is enough to block |
| Pass | −1, down to −8 | A beam through it twice makes it free; a wall needs several misses to vanish |
| Blocked | ≥ +3 | |
| Free | ≤ −2 | |
| Unknown | anything else, including never seen | Unknown is not free (P6) |
| Frame | The pose frame; cell edges on multiples of 5 cm | Obstacles stay put as Heron turns or slides |

All counts are starting values (P5), tuned against the simulator and the
recordings.

### 6.2 Each scan

1. **Check it:** the LiDAR's consumer rules, as `SectorCheck.h` applies
   them (fresh, covered). A scan that fails is skipped and counted.
2. **Place every point** at its own time (§3): `poseAt(scan.timeOf(β))`.
   A point whose pose cannot be found skips the whole scan.
3. **Pass:** lower each cell on the straight line from the LiDAR's cell to
   the point's, not including the point's own (Bresenham). A point
   outside the grid lowers the cells up to the edge.
4. **Hit:** raise the point's cell.

The `Scan` holds only valid returns, so a no-return clears nothing, as it
must: dark or shiny surfaces can return nothing while still being there.

**Height.** The A1M8 sees one plane. The first version assumes it is
level, so every point is at the LiDAR's height and counts. A tilted or
second LiDAR adds a height test here (`[lidar] height`, §13).

### 6.3 Rolling

When Heron is more than 0.5 m from the grid's centre, the grid shifts by
whole cells to re-centre. Cells shifted out are forgotten; cells shifted
in start at 0, unknown.

### 6.4 What the planner gets

`Perception::getLatest()` returns a `shared_ptr<const ObstacleMap>`: the
grid, its origin, the scan's `seq` and times, and the pose at the scan's
end. A new map is built per scan by copying the last and updating it, so
the planner never reads a grid being written. Cost estimate, not
measured: 530 rays of at most 80 cells, about 40,000 cell updates a scan.

## 7. The safety limiter

The last line between a command and the motors, in Assisted and
Navigate, every 20 ms. It reads the newest raw scan, never the map, so a
bug in perception or the planner cannot drive Heron into what the LiDAR
sees (P7, P8).

### 7.1 Inputs

- **The command** as a twist in m/s and rad/s: the stick (Assisted) or
  the planner's motion (Navigate), converted with `[geometry]`.
- **The newest scan,** its points placed in the robot frame (§3), then
  moved by the robot's motion since each was measured (`poseAt`), so they
  are where they are now.
- **The measured velocity** from the newest odometry reading, for the
  stopping distance: true speed even before the speed loop exists.
- **The outline,** the robot's rectangle from `[geometry]`, grown by
  `stop_margin`.

### 7.2 The sweep

For a point p and a motion held for a time T, the sweep asks: does the
outline touch p, and when? For a pure translation along a unit direction
u, the outline (half-length a, half-width b) touches p at the distances s
where `|px − s·ux| ≤ a` and `|py − s·uy| ≤ b`, the overlap of two
intervals: exact, a few operations a point. With rotation the motion is
stepped every 2 cm or 2°, whichever is finer, and each step tested the
same way. About 530 points × up to 20 steps every 20 ms, well under a
millisecond (estimate, to be measured on the Pi in N4).

T is the time to stop from the measured speed, plus the time the limiter
cannot see: `T = v / brake + reaction`, with `reaction` the scan's age plus
one scan period (about 0.3 s), and `brake` from `[geometry]`.

### 7.3 The rule

1. **Clear:** no point touched within T. The command passes unchanged.
2. **Cut by axis:** otherwise each of vx, vy and wz is tested on its own.
   An axis clear alone keeps its full value; a blocked one is scaled down
   to the largest value that keeps its own sweep clear. Then the combined
   motion is tested once more and scaled as a whole if it still touches.
   This is how Heron keeps sliding along a wall while the part of the
   motion into it stops.
3. **Slow zone:** a point inside the outline grown by `slow_margin` along
   the motion lowers the speed in proportion, so Heron does not brake hard
   from full speed at the last moment.
4. **Already too close:** a point inside the stop outline now blocks only
   motion that brings it closer; motion away is allowed, so Heron can
   always back off.

### 7.4 When it cannot see

| Condition | Assisted and Navigate |
|---|---|
| No scan, or the newest older than 300 ms | Stop |
| Scan not covered (`isCoverageOk()` fails) | Stop |
| The direction of motion falls in an unobserved arc (`isObserved()`) | That axis stops |
| Pose not fresh | Points are not moved for the robot's motion; `reaction` doubles |
| The LiDAR's near blind spot | Checked when the profile loads: the 13 cm circle round the LiDAR (126.8 mm measured **[ours]**) must lie inside the stop outline, or the profile is refused |

Manual is never limited: it is the way out when the LiDAR fails.

### 7.5 What it reports

The limited command, the scale per axis, the nearest point and its
bearing, and why (clear, cut, slowed, stale, not covered, unobserved),
on the `limiter` channel.

## 8. The local planner

Navigate only, 10 times a second, on its own thread (P9).

### 8.1 The course

- **Engaging:** with SwC down, the right stick's direction, turned into
  the pose frame, is the course heading; the course line runs through
  Heron's position at that moment along it. The speed asked for is the
  stick's size × the throttle × `max_speed`.
- **Moving the stick** more than 10° re-anchors the line at the current
  position. Letting go (stick centred) stops Heron and drops the course
  (D1).
- **Heading is kept** (D3): the planner chooses only vx and vy; wz comes
  from the left stick as in Manual, and is 0 if it is centred.

### 8.2 Each cycle

1. Take the newest map, pose and measured velocity. A map older than
   0.5 s, or a pose not fresh, means stop with a reason.
2. **Candidates:** 11 × 11 values of vx and vy in the window reachable
   from the measured velocity in 0.3 s at `[geometry] accel`, within
   `max_speed`.
3. **Simulate** each for 1.5 s in 0.1 s steps, sweeping the outline
   grown by `stop_margin` over the map. A blocked cell, or an unknown one
   within 0.6 m of Heron (P6), rejects the candidate.
4. **Score** the rest: clearance (nearest blocked cell, capped at 0.5 m),
   progress along the course heading, distance from the course line at
   the end, change from the last choice. Weights are starting values,
   tuned in the simulator.
5. **Choose** the best, or **Blocked** if none is left: zero, with the
   reason, on the `plan` channel.

The result, `getLatest()`, holds the twist, the predicted path for
telemetry, the state (following, avoiding, blocked) and its time. The
control loop treats a result older than 0.3 s as stop. Whatever it
chooses still goes through the limiter.

## 9. The robot program: `heron_robot`

`heron_teleop` grown up, as a new program (P10). Same start-up (profile,
`Motors::start()`, remote) plus the LiDAR, odometry, pose, perception,
planner and telemetry server.

Each 20 ms:

| Step | Manual | Assisted | Navigate |
|---|---|---|---|
| Remote, `ArmGate` | as today | as today | as today |
| Command | sticks | sticks | planner's result |
| Limiter | no | yes | yes |
| Mix, `Motors::drive()` | yes | yes | yes |
| Publish `robot` | yes | yes | yes |

- **Mode changes** take effect at once. Entering Navigate sets the
  course from the stick (§8.1). Entering Assisted or Navigate with the
  LiDAR not OK stops Heron and says why; it never falls back to Manual by
  itself.
- **SwC is CH9** (`motor-design.md` §4, with the i6X at 10 channels),
  read with the remote library's `decodeSwitch3()`. Its middle reads 1500
  **[ours]** (`remote-ibus-protocol.md` §4); which end is up is to be
  checked with `ibus_reader monitor` in step N4.
- **Start-up order:** odometry first, still (its calibration needs the
  robot still), then the LiDAR, motors and threads.
- **Failures** are reported, never fatal: a LiDAR, OTOS or planner
  failure leaves Manual working; telemetry failing changes nothing
  (telemetry D10).

## 10. The profile

New sections in `profiles/<rig>.conf` (P11), read by
`heron_profile` with its strict rules (`rig-profiles.md` §2). The
first three are shared facts other workstreams use too; they are added
in N1 with their owners.

```
[geometry]
length = 0.24            # m, the outline including wheels
width = 0.20             # m
max_speed = 0.35         # m/s at full command, measured
max_turn_rate = 2.0      # rad/s at full command, measured
accel = 0.6              # m/s^2, the planner's window
brake = 1.0              # m/s^2, measured from full speed

[lidar]
x = 0.06                 # m, spin axis from the robot frame's origin
y = 0.0
yaw_deg = 0              # becomes LidarConfig::mountingOffsetRad
height = 0.14            # m, for the height test later

[odometry]
x = -0.05                # m, becomes OtosConfig::offset
y = 0.0
yaw_deg = 0
linear_scalar = 1.0
angular_scalar = 1.0

[navigation]
stop_margin = 0.05       # m round the outline
slow_margin = 0.25       # m
```

The values shown are placeholders, not the bench rig's: each is measured
in N1 and recorded with its method, as `[motor.calibration]` is.

## 11. Telemetry

Four channels by the recipe (`telemetry-design.md` §9), SI units, the
pose frame unless named:

| Channel | Each item | Rate |
|---|---|---|
| `pose` | Time, x, y, heading, velocity, fresh, OTOS warnings | 20/s |
| `map` | Scan seq, time, origin, the 6,400 counts (base64), the pose at scan end | per scan, ~7/s; about 9 KB each |
| `plan` | Mode, course heading and line, chosen twist, predicted path, state and reason | 10/s |
| `limiter` | Command in and out, scale per axis, nearest point and bearing, reason | 10/s, the newest of the loop's |

A host viewer, `nav_view.py`, draws the map, the robot's outline, the
course line, the chosen path and the newest scan together: the debugging
view for N3 to N5.

## 12. Testing

### 12.1 Each library alone (ctest, the Mac)

| Library | Checks |
|---|---|
| `heron_geometry` | Compose and inverse round trips; the sweep against hand-worked cases, front, side, corner, rotation; heading wrap |
| `heron_pose` | Interpolation, the short way round; extrapolation limit; gaps and resets |
| `heron_perception` | A point marks one cell; a beam clears its line and not the end; a no-return clears nothing; rolling keeps obstacles in place; a skewed scan of a straight wall comes out straight once placed |
| `heron_limiter` | Each rule of §7.3 and row of §7.4; sliding along a wall keeps the slide; backing off is allowed; stale and uncovered stop |
| `heron_planner` | Straight on an empty map; round a box and back to within 10 cm of the line; blocked in a dead end; unknown near Heron avoided |

### 12.2 The simulator (`heron_nav_sim`)

- **A world** of line segments: walls and boxes.
- **A LiDAR** producing real `Scan`s with our measured numbers **[ours]**:
  7.3 revolutions a second, about 530 valid points each, nothing nearer
  than 126.8 mm, a share of no-returns, the clockwise sweep and its
  timing, so deskewing is tested against the skew it must undo.
- **An OTOS** with noise and a heading drift in the measured range,
  0.2 to 0.9°/min **[ours]**.
- **Mecanum motion** with acceleration limits and a lag.
- **Virtual time:** the pure cores run in a loop with no threads, so a
  minute of driving runs in well under a second and every run repeats.

**Scenarios as ctest checks,** for example:

1. Navigate at a box: pass it with at least 15 cm to spare and be back
   within 10 cm of the course line within 2 m.
2. Assisted, driven at a wall at full speed: stop with the outline
   outside it, every time, over 1,000 runs with random starts, speeds,
   angles and sensor noise.
3. Assisted, sliding along a wall: keep sliding.
4. Navigate into a dead end: Blocked, stopped.
5. LiDAR stops mid-run: stop within 300 ms.

### 12.3 Recordings

`lidar_record` sessions replay through `LidarConfig::replayFile` into
perception with a still pose, to check the map against real returns.

### 12.4 The robot

On the block first (the wheels turn, a box held in front stops them),
then the floor at low speed with cardboard boxes, watching `nav_view.py`.

## 13. Several LiDARs

- **Each is an input to perception** and the limiter: its points placed
  by its own `[lidar.<name>]` section, then merged. The first LiDAR's
  section stays `[lidar]`.
- **Device names by USB port,** as now: every A1M8 has the same USB
  bridge and serial.
- **A height test** in §6.2 once a LiDAR is tilted or sits in another
  plane.
- **Unverified on a Pi 3:** USB power with several A1s spinning, and
  whether their lasers interfere.

## 14. Limits to know now

- **One slice of the world:** the A1M8 sees one plane; a shoe below it or
  a table top above it is invisible.
- **Nothing closer than about 13 cm** (126.8 mm **[ours]**); black and
  glass surfaces return poorly.
- **Reaction time caps the speed:** a scan takes ~137 ms **[ours]**, plus
  processing and braking. Estimate, not measured: about 0.5 m/s with
  20 cm of margin, in Assisted and Navigate.
- **Heading drifts,** 19–35°/h in our runs **[ours]**: fine for minutes;
  longer runs need correction from the LiDAR (N6).

## 15. Prerequisites and steps

N1 is mostly other workstreams' work (record it in theirs):

| What | Owner |
|---|---|
| LiDAR and OTOS mounted on the bench rig; `[lidar]`, `[odometry]` measured; the OTOS axes checked | motor, shared profile |
| `[geometry]`: outline, `max_speed`, `max_turn_rate`, `accel`, `brake` measured on the floor | motor, shared profile |
| The profile reader taking the new sections | shared (`heron_profile`) |
| LiDAR restart, unplug and replug | LiDAR (`RESUME-lidar.md` §9) |
| Telemetry steps 2 to 4, and the odometry channel | telemetry |
| The OTOS speed loop | motor; wanted, not required: the limiter uses measured speed |

Then, each step checked before the next:

| Step | Builds | Checked by |
|---|---|---|
| N2 | `heron_geometry`, `heron_nav_sim` | Geometry tests; the simulated LiDAR's scans compared with recorded ones |
| N3 | `heron_pose`, `heron_perception`, the `pose` and `map` channels, `nav_view.py` | Unit tests; a recording replayed; on the robot, pushed by hand round a room, the map matches it |
| N4 | `heron_limiter`, `heron_robot` with Manual and Assisted | Sim scenarios 2, 3 and 5; on the block, a box stops the wheels; on the floor, it stops short of a box at walking pace |
| N5 | `heron_planner`, Navigate | Sim scenarios 1 and 4; on the floor, round a box and back |
| N6 | Heading correction from the LiDAR, then mapping | Later design |
| N7 | A second LiDAR | Later design |

## 16. Later: ROS 2 on a more powerful Heron

D4 is for this hardware. ROS 2 pays off through reuse (room mapping,
go-to-goal, hardware with ROS drivers, work split across people or
computers) and is comfortable on a Pi 4 or 5 with 4 GB or more. If a
later Heron moves to it, roughly:

| Here | In ROS 2 |
|---|---|
| `Lidar`, `Odometry` | sllidar_ros2; a small node wrapping `Odometry` |
| `[geometry]`, `[lidar]`, `[odometry]` | A URDF and robot_state_publisher (TF) |
| `heron_pose`; N6 | robot_localization; slam_toolbox and AMCL |
| `heron_perception` | Nav2 costmap_2d, obstacle layer |
| `heron_planner` | Nav2 controller server (DWB or MPPI) |
| `heron_limiter` | Nav2 collision_monitor |
| Remote, SwC | A teleop node and twist_mux |
| Mixing, `Motors`, the speed loop | Kept, wrapped as a cmd_vel node or a ros2_control controller |
| Telemetry | RViz or Foxglove, rosbag |
| `heron_nav_sim`, scenarios | Gazebo, launch_testing |

Nav2 has no stick-direction mode like Navigate (D1): that would stay our
own node. Separate libraries with pure cores keep the move cheap. This
mapping is from general knowledge of ROS 2, not tried here.

## 17. Left out of the first version

- Room maps and position in the room (N6).
- Going to a goal, or a course that carries on without the stick.
- Turning as part of avoidance (D3), and planning the turn rate.
- Moving obstacles beyond what clearing handles (a person walking away
  clears; one walking toward Heron is just an obstacle, re-seen each scan).
- A height test (§6.2) until a LiDAR is tilted or a second one added.

## 18. Open questions

1. **The OTOS axes** match the robot frame's once mounted (N1).
2. **The Pi 3's time budget** with all threads running: estimated
   comfortable, to be measured through the `robot` channel's loop timing
   in N4.
3. **The grid counts and planner weights** (§6.1, §8.2): starting values,
   settled in the simulator.
4. **Which end of SwC is up** on our transmitter (§9).
