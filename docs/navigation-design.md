# Navigation — design

How the LiDAR and the odometry sensor turn Heron from a remote-controlled
base into a semi-autonomous one: it knows where it is, sees obstacles,
steers around them, and goes back to its course. **Proposed 2026-10-30,
decisions accepted 2026-11-04**; nothing built. Built for one LiDAR first,
with room for several.

It builds on what is measured and built already: the sensors' behaviour
in [`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md) and
[`odometry-otos-reference.md`](odometry-otos-reference.md), the safety
arbiter planned for SwC in [`motor-design.md`](motor-design.md) §4, and
[`telemetry-design.md`](telemetry-design.md). Numbers marked **[ours]**
were measured on our units; the rest are estimates and say so.

## 1. The goal

You drive Heron in a direction. As it moves, it sees that it will hit
something in its path, goes around it, and returns to the course it was
on once the obstacle is behind it. If there is no way around, it stops
and says so.

## 2. Decisions (2026-11-04)

Proposed with the plan and accepted by the user as recommended.

| # | Decision | Why |
|---|---|---|
| D1 | **Navigate is driven by holding the stick in the direction you want; letting go stops Heron** | You stay in charge; the robot only steers around things. Setting a course and letting go, with Heron carrying on alone, is a natural later step once this works |
| D2 | **"Knowing where it is" means position since start-up, from the OTOS, for now** | Enough to avoid an obstacle and resume a course. Position on a map of the room is phase N6, and builds on this without redoing it |
| D3 | **Avoid by sliding sideways and keeping the heading, not by turning like a car** | It is what mecanum wheels are for, it keeps a forward-facing LiDAR pointed down the course, and it makes returning to the course line simple |
| D4 | **Our own code, not ROS 2 and Nav2** | ROS 2 and Nav2 already have obstacle maps from several sensors, these planners, and mapping, but they are heavy for a Pi 3 with 1 GB, are not packaged for this Debian, and would replace the architecture already built. Our own code uses the same proven methods, focused on Heron. Reconsider for a more powerful Heron (§11) |

## 3. Five layers, each its own library

Each layer does one job and hands its newest result to the next, the way
`Lidar` and `Odometry` already do through `getLatest()`. No layer waits
for another, so a slow one never holds up the motors.

```
                 newest raw points (skip the map and the planner)
   +--------------------------------------------------------------+
   |                                                              v
 LiDAR --scans--> 3 Perception --obstacle map--> 5 Planner --> 4 Limiter --> mixing
 ~7/s             per scan                       10/s          50/s           -> motors
                     ^                             ^  ^        (every 20 ms)
                     | pose at scan                |  |
 OTOS --x,y,θ---> 2 Pose ----------where it is----+  |
 100/s            100/s                               |
 Remote ------------stick direction = the course------+
```

Navigate mode is drawn. In Manual and Assisted the stick goes straight to
the limiter and the planner is idle.

One pass through:

1. **Sensors (1)** keep reading on their own threads: a LiDAR scan about
   7 times a second, the OTOS 100 times a second.
2. **Pose (2)** turns OTOS readings into "Heron is here, facing this
   way", measured from where it started. *Pose* is the usual robotics word
   for position and heading together.
3. **Perception (3)** takes each new scan, places every point using the
   LiDAR's mounting pose and Heron's pose while the scan was taken, and
   updates the obstacle map (§4).
4. **Planner (5)**, 10 times a second (every 100 ms), reads the newest
   map, pose and stick direction, imagines a range of motions 1 to 2 s
   ahead, and picks the one that stays clear and makes progress along the
   course.
5. **Limiter (4)**, 50 times a second (every 20 ms, the control loop),
   checks that motion against the newest raw LiDAR points. If anything is
   too close in the way it is heading, it slows or stops. What it lets
   through goes to mixing and the motors.

Why five separate libraries:

- **Each is tested alone.** The planner can be tested on the Mac with a
  made-up map, with no robot and no LiDAR.
- **The limiter does not trust the others.** It reads the LiDAR directly,
  so a bug in the map or the planner cannot drive Heron into a wall. That
  is why it is numbered before the planner: it is built and proven first.
- **A second LiDAR changes two layers only:** Sensors gets another entry,
  and Perception merges both into the same map.
- **Telemetry can show each one,** which is how a wrong turn gets traced
  to the layer that caused it.

| Layer | Job |
|---|---|
| 1. Sensors | Exist: `Lidar`, `Odometry`. Each LiDAR gets a name and a mounting pose (position, height, tilt) from the rig profile: `[lidar.front]`, later `[lidar.rear]`. One device is the first entry in a list |
| 2. Pose | Position and heading since start-up, from the OTOS. Its drift, about 0.2°/min after a calibration and 0.6–0.9°/min over 10 min **[ours]** (`RESUME-odometry.md`), is small enough to avoid an obstacle and rejoin a course over minutes |
| 3. Perception | Each LiDAR's points move into the robot's frame by its mounting pose, corrected for Heron's motion during the ~137 ms a scan takes **[ours]**. They feed the obstacle map, counting only what is within the robot's height |
| 4. Safety limiter | Works on the newest points, not the map. Checks the area the robot's outline will sweep at the commanded speed; slows the part of the motion heading toward an obstacle; stops inside a hard limit. A stale or failed LiDAR means stop. This is `motor-design.md` §4's safety arbiter, Phase 2 |
| 5. Local planner | Tries a range of candidate motions, projects each 1 to 2 s ahead on the map, and scores them on clearance, progress along the course, distance off the course line, and smoothness. The dynamic window approach; our own code |

## 4. The obstacle map

Heron's short-term memory of what is around it, so the planner can look
ahead. Not a map of the room; that is phase N6. Perception builds it, the
planner reads it, the limiter ignores it.

What it is:

- **A square grid with Heron in the middle,** 4 × 4 m in 5 cm cells:
  80 × 80 = 6,400 cells, about 6.4 KB.
- **Each cell holds a small count,** not a yes or no. Above a threshold
  the cell counts as blocked, so one stray point does not create a wall
  and one missed scan does not erase one.
- **Unknown is not free.** A cell no beam has reached, such as the space
  behind a box, stays unknown.
- **Fixed to the floor, not to Heron:** laid out in the pose frame, so
  when Heron turns or slides, obstacles stay where they are in it.

Each scan, about 7 times a second:

1. **Place the points** where they really were, using the LiDAR's
   mounting pose and Heron's pose at the instant each was measured. That
   undoes the smear from moving during a scan.
2. **Drop what cannot hit Heron:** points above or below the robot's
   height, which matters for a tilted LiDAR.
3. **Mark:** the cell each point lands in has its count raised.
4. **Clear:** the cells along the line from the LiDAR to the point have
   their counts lowered, because the beam passed through them. A beam with
   no return clears nothing, since dark or shiny surfaces can return
   nothing while still being there.

How it stays current: it **rolls**, re-centring on Heron as it moves;
cells that scroll off are forgotten and cells that scroll in start
unknown. It covers about 2 m each way and the last few seconds, so OTOS
drift does not build up in it.

Why a map and not just the newest scan: it smooths single missed or stray
points; it remembers what is beside Heron after it leaves the LiDAR's
view, for example by coming closer than the 126.8 mm minimum range
**[ours]**; and several LiDARs merge into one map, so the planner never
needs to know how many there are.

## 5. Going around, and back

```
            slides left              pulled back to the line
          .--------------------.
 [H]>----'      [obstacle]      '------>[H]     course line
```

- **The course** is the direction you are driving, taken from the stick
  (D1).
- **Heron slides past without turning** (D3), keeping its clearance
  margin, and the planner's pull toward the course line brings it back
  once the obstacle is passed.
- **Blocked:** with no way through, it stops, and the `robot` telemetry
  channel says why.
- **The limiter still runs underneath** the planner the whole time.

## 6. The robot program and SwC

`heron_robot`, today's `heron_teleop` grown up. The control loop, 50 times
a second, never waits: perception and the planner run on their own
threads and hand over their newest result, as every library does now.

| SwC | Mode | What happens |
|---|---|---|
| Up | Manual | As today |
| Middle | Assisted | You drive; the limiter slows or stops Heron near obstacles |
| Down | Navigate | You give a direction; Heron goes around obstacles and resumes the course |

`motor-design.md` §4 calls the third position "Auto, later"; Navigate is
that mode. Every layer publishes on the telemetry server: pose, the
obstacle map, the planner's chosen path, and why the limiter slowed
Heron. That is how this gets debugged, so telemetry comes first.

## 7. Several LiDARs

- **Each is an input to perception.** Points from all sensors are moved
  into one robot frame by each sensor's mounting pose, then filtered by
  height.
- **Other sensors plug in the same way:** bumpers, ultrasonic, a depth
  camera.
- **Device names by USB port,** as now: every A1M8 has the same USB bridge
  and serial, so udev names each by its port.
- **Unverified on a Pi 3:** whether its USB power holds with several A1s
  spinning, and whether the units' lasers interfere.

## 8. Testing

- **A simulator on the Mac first:** a 2D world of walls and boxes; a
  simulated A1M8 with our measured numbers (7.3 scans a second **[ours]**,
  about 530 points a scan, nothing closer than about 13 cm, dropouts); a
  simulated OTOS with drift; mecanum motion.
- **Whole scenarios as ctest checks,** for example: drive at a box, pass
  it with at least 15 cm to spare, and be back within 10 cm of the course
  line within 2 m.
- **Recorded LiDAR sessions** replay through the same code.
- **Then the mini on the floor,** with cardboard boxes.

## 9. Prerequisites

Mostly work in other workstreams, which this one waits on.

| What | Why | Where |
|---|---|---|
| A robot on the floor with the LiDAR and OTOS mounted, their mounting poses in its profile | Avoidance can only be tested moving | motor |
| The robot's outline and wheel geometry in the profile | The planner and the limiter need its size | shared: `rig-profiles.md` |
| The speed loop closed with the OTOS: 0.3 m/s means 0.3 m/s | Avoidance needs real speeds, not fractions of full power | motor (planned) |
| LiDAR restart, unplug and replug | A robot that relies on the LiDAR needs it to recover | `RESUME-lidar.md` §9 |
| Telemetry steps 2 to 4, and an odometry channel with the reading's own timestamp | To see what the robot thinks | telemetry |

## 10. Limits to know now

- **One slice of the world.** An A1M8 sees one horizontal plane at its
  height: a shoe below it or a table top above it is invisible. A second
  LiDAR in another plane is the fix.
- **Nothing closer than about 13 cm** (126.8 mm measured **[ours]**);
  black and glass surfaces return poorly. The robot's outline must keep
  obstacles outside that range.
- **Reaction time caps the speed.** A scan takes ~137 ms, plus processing
  and braking. Rough estimate, not measured: about 0.5 m/s with 20 cm of
  margin, in Assisted and Navigate.
- **Heading drifts,** 19–35°/h in our runs **[ours]**. Fine for minutes;
  longer runs need correction from the LiDAR (N6).

## 11. Later: ROS 2 on a more powerful Heron

D4 is for this hardware. ROS 2 pays off through reuse: room mapping and
go-to-goal (Nav2, slam_toolbox), hardware with ROS drivers, and work
split across people or computers. It is comfortable on a Pi 4 or 5 with
4 GB or more. If a later Heron moves to it, roughly:

| Here | In ROS 2 |
|---|---|
| `Lidar`, `Odometry` | sllidar_ros2; a small node wrapping `Odometry` |
| Mounting poses, outline in the profile | A URDF and robot_state_publisher (TF) |
| Pose; N6 | robot_localization; slam_toolbox and AMCL |
| Perception, the obstacle map | Nav2 costmap_2d, obstacle layer |
| Planner | Nav2 controller server (DWB or MPPI) |
| Limiter | Nav2 collision_monitor |
| Remote, SwC | A teleop node and twist_mux |
| Mixing, `Motors`, the speed loop | Kept, wrapped as a cmd_vel node or a ros2_control controller |
| Telemetry | RViz or Foxglove, rosbag |
| Simulator, scenario tests | Gazebo, launch_testing |

Nav2 has no stick-direction mode like Navigate (D1): that would stay our
own node, for example one placing a moving goal a few metres along the
course. Building the five layers as separate libraries keeps that move
cheap: each becomes a thin node or gives way to its Nav2 equivalent.
This mapping is from general knowledge of ROS 2, not tried here.

## 12. Phases

| Phase | What | Result |
|---|---|---|
| N0 | This design and its decisions; the navigation workstream | An agreed plan |
| N1 | The prerequisites (§9) | A robot that drives at known speeds |
| N2 | Simulator and replay test bench on the Mac | Scenarios run in ctest |
| N3 | Pose, motion correction, rolling obstacle map, on telemetry | Heron's view of the world, live |
| N4 | Safety limiter; SwC Assisted | It will not hit what it sees |
| N5 | Local planner; SwC Navigate | It goes around and resumes |
| N6 | Heading correction from the LiDAR, then mapping | Where it is in the room |
| N7 | Second LiDAR | Several planes |

## 13. Still open

- **Unknown cells:** treat them as blocked near Heron and free further
  out (recommended), so Heron never drives into space it has not seen.
- **The robot's outline:** check it in the planner, swept along each
  candidate motion (recommended), rather than growing every obstacle by
  the robot's size in the map, because with mecanum sliding the outline's
  orientation changes what fits.
- **Library names and their headers,** settled when N3 starts, following
  `docs/odometry-cpp-design.md`'s pattern (a pure layer, a device layer,
  a threaded one).
- **Which robot carries it:** the bench rig, now on the floor with
  mecanum wheels, or the mini robot.
