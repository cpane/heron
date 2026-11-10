# Navigation C++ — interfaces

The public headers of the navigation libraries, before any of them is
written: their types, their functions, who calls what, and how they fit
the libraries already built. The design they implement is
[`navigation-design.md`](navigation-design.md) (P1 to P12, accepted
2026-11-08); this document is its C++ shape, and section numbers like
"design §7" point there.

**Accepted 2026-11-08** (I1 to I9, as proposed). It covers what phases N2 to N4
build: `heron_geometry`, `heron_nav_sim`, `heron_pose`,
`heron_perception`, `heron_limiter`, and the parts of `heron_robot` that
glue them. The planner (N5), §8, was accepted 2026-11-09 (I10 to I16).

Every header follows `CPP_CODING_STANDARD.md` and starts with the MIT file
header; the sketches below leave the headers and most comments out.

## 1. Decisions

| # | Decision | Why |
|---|---|---|
| I1 | **`double` everywhere in navigation;** `Scan`'s `float` bearings and ranges are widened on the way in | Odometry's `Planar` and the drive's `BodyVelocity` are `double`; poses are composed many times, and a Pi 3 does `double` at full speed. `Scan` stays `float`: it is the LiDAR's boundary, and 530 points of 8 bytes is why (`Scan.h`, decision 3) |
| I2 | **Every length in metres, angle in radians, speed in m/s and rad/s, time as `steady_clock` points and durations** | Design §3.5 and telemetry D12. A fraction of full drive (−1..1) exists only in `heron_drive` and at the one place that converts (§7.2) |
| I3 | **The cores take a `TimePoint now` and every other time as arguments;** only the wrappers read the clock | P2, design §4.1. The simulator passes virtual times |
| I4 | **A pose and a twist are two types,** `Pose2` (where, in a parent frame) and `Twist` (how fast, in the robot frame) | They are both three numbers and easily swapped; the compiler should catch passing one for the other. `Odometry::Planar`, which holds both, stays in odometry |
| I5 | **A failure is a reason, never a zero:** lookups return a result with an `ok` flag and a reason enum | A zero pose is a real pose (the start point), and a zero command is a real command. Design §5 |
| I6 | **The libraries take their own config structs;** they never read the profile | As `Motors` does (motor D4): `heron_robot` loads the profile and fills each config, and tests fill them by hand with no files |
| I7 | **The pose history is copied out, not locked per point** | Perception places up to ~200 points a scan (16 to 39% of ~531 samples return indoors [ours]) and the limiter as many every 20 ms, each needing a pose. Copying the 14 KB history once is cheaper than 200 locked lookups, and the core then works on a plain value |
| I8 | **Each odom frame has a number, `epoch`,** raised whenever odometry restarts | A restart makes every stored pose meaningless (design §5). Comparing two integers is how perception knows to clear its map, without guessing from jumps |
| I9 | **Namespaces follow the libraries:** `Heron::Geometry`, `Heron::Pose`, `Heron::Perception`, `Heron::Limiter`, `Heron::Planner`, `Heron::NavSim`, as `Heron::Odometry` and `Heron::Lidar` | One library, one namespace, as now |

## 2. `heron_geometry`: the shared maths

Pure, no I/O, no threads, no time. Every other navigation library links
it. Headers in `include/heron/geometry/`.

### 2.1 `Pose2.h`: points, poses, twists

The 2 means planar: x, y and one heading, as in ROS's `Pose2D` and
GTSAM's `Pose2`. It also keeps the type apart from the `Heron::Pose`
namespace (I9), and leaves room for a `Pose3` and `Vec3` beside them if
a mount ever needs one. Type names follow the coding standard:
PascalCase, no prefix.

```cpp
namespace Heron::Geometry {

/** A point or a direction in some frame, metres. */
struct Vec2 {
  double x{0.0};
  double y{0.0};
};

Vec2 operator+(Vec2 a, Vec2 b);  Vec2 operator-(Vec2 a, Vec2 b);
Vec2 operator*(double k, Vec2 v);
[[nodiscard]] double norm(Vec2 v) noexcept;
[[nodiscard]] Vec2 rotate(Vec2 v, double angleRad) noexcept;  // anticlockwise

inline constexpr double kDegToRad = 0.017453292519943295;

/** Folds an angle into [-pi, +pi): wrapBearing()'s double twin. */
[[nodiscard]] double wrapAngle(double rad) noexcept;

/**
 * The pose of a child frame in a parent frame: where its origin is and
 * which way its x axis points. m, m, rad. Heron's pose is the robot frame
 * in the odom frame (design §3.1).
 */
struct Pose2 {
  double x{0.0};
  double y{0.0};
  double heading{0.0};  // wrapped to [-pi, +pi)
};

/** A child-frame point, in the parent frame: design §3.3, step 2. */
[[nodiscard]] Vec2 toParent(const Pose2 &pose, Vec2 p) noexcept;
/** A parent-frame point, in the child frame: design §3.4's reverse step. */
[[nodiscard]] Vec2 toChild(const Pose2 &pose, Vec2 p) noexcept;

/** b given in a's frame, expressed in a's parent. */
[[nodiscard]] Pose2 compose(const Pose2 &a, const Pose2 &b) noexcept;
[[nodiscard]] Pose2 inverse(const Pose2 &pose) noexcept;
/** Where `to` is, seen from `from`: compose(inverse(from), to). */
[[nodiscard]] Pose2 relative(const Pose2 &from, const Pose2 &to) noexcept;

/** How fast, in the robot frame: m/s forward, m/s left, rad/s anticlockwise. */
struct Twist {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/** Where a constant twist, held for dtS seconds, takes a pose: the exact
 *  arc, not a straight-line step. */
[[nodiscard]] Pose2 integrate(const Pose2 &start, const Twist &twist, double dtS) noexcept;

/** A LiDAR reading in the robot frame: design §3.3, step 1. */
[[nodiscard]] Vec2 polarToRobot(double bearingRad, double rangeM, Vec2 sensorOffset) noexcept;

}  // namespace Heron::Geometry
```

`polarToRobot()` takes plain numbers, not a `Heron::Point`, so geometry
does not depend on the LiDAR's types.

### 2.2 `Outline.h`: the robot's rectangle and the sweep

```cpp
namespace Heron::Geometry {

/** The robot's footprint: a rectangle centred on the robot frame's origin,
 *  its sides along the axes. [geometry] length and width, halved. */
struct Outline {
  double halfLength{0.0};  // along x
  double halfWidth{0.0};   // along y
};

[[nodiscard]] Outline grown(const Outline &outline, double marginM) noexcept;
[[nodiscard]] bool contains(const Outline &outline, Vec2 p) noexcept;  // edges included

/** How finely a motion with rotation is stepped (design §7.2). */
struct SweepStep {
  double maxStepM{0.02};
  double maxStepRad{0.0349};  // 2 degrees
};

/**
 * The first time the outline touches a fixed point while the robot moves
 * at a constant twist, the point given in the robot frame at time 0.
 *
 * Pure translation is solved exactly (the overlap of two intervals);
 * with rotation the motion is stepped by `step`.
 *
 * @return Seconds until contact: 0 if the point is already inside, no
 *         value if it stays clear for horizonS
 */
[[nodiscard]] std::optional<double> firstContact(const Outline &outline, Vec2 point,
                                                 const Twist &twist, double horizonS,
                                                 const SweepStep &step = {}) noexcept;

}  // namespace Heron::Geometry
```

**Why centred.** The design's `[geometry]` has a length and a width, so
the outline is centred on the wheelbase. A robot whose LiDAR or bumper
sticks out at one end gets an outline long enough to cover it, which
wastes a few centimetres at the other end. If that matters on the floor,
`Outline` grows four distances (front, back, left, right) and the profile
four numbers; nothing else changes shape.

### 2.3 A worked example

The box of design §3.3 and §3.4, in code:

```cpp
using namespace Heron::Geometry;
const Vec2 lidarOffset{0.06, 0.0};
const Vec2 inRobot = polarToRobot(30.0 * kDegToRad, 1.0, lidarOffset);  // (0.926, 0.500)
const Pose2 heron{2.0, 1.0, 90.0 * kDegToRad};
const Vec2 onFloor = toParent(heron, inRobot);                          // (1.500, 1.926)

const Pose2 later = integrate(heron, Twist{0.25, 0.0, 0.0}, 2.0);       // (2.0, 1.5, 90°)
const Vec2 nowSeen = toChild(later, onFloor);                           // (0.426, 0.500)

const Outline body{0.12, 0.10};                                         // 0.24 x 0.20 m
const Twist ahead{0.25, 0.0, 0.0};
firstContact(body, nowSeen, ahead, 3.0);                                // no value
firstContact(body, Vec2{0.426, 0.0}, ahead, 3.0);                       // 1.224 s
```

The last two lines: the box is 0.5 m to Heron's left, outside the 0.10 m
half-width, so driving straight on never touches it and `firstContact`
returns no value. A box straight ahead at the same distance is met when
Heron's front edge, 0.12 m ahead of its centre, reaches it:
(0.426 − 0.12) / 0.25 = 1.224 s. Both are ctest cases.

## 3. `heron_nav_sim`: the simulator (N2)

Host only, for tests: a world, a LiDAR that produces real `Scan`s, an
OTOS with drift, and a mecanum base. No threads; the test moves time on.
Headers in `src/nav_sim/`, private, as tests are its only users. Built in
N2, apart from the OTOS (below).

```cpp
namespace Heron::NavSim {

struct Segment { Geometry::Vec2 a; Geometry::Vec2 b; };

/** Walls and boxes, as line segments, in the odom frame. */
struct World {
  std::vector<Segment> segments;
  void addBox(Geometry::Vec2 centre, double length, double width);
  /** The first hit along a ray, or no value within maxRangeM. */
  [[nodiscard]] std::optional<double> cast(Geometry::Vec2 from, double angleRad,
                                           double maxRangeM) const;
};

/** Mecanum motion with an acceleration limit and a lag. */
struct BaseConfig { double accel{0.6}; double turnAccel{3.0}; Duration lag{std::chrono::milliseconds(60)}; };
class Base {
  public:
    Base(BaseConfig config, Geometry::Pose2 start, TimePoint startTime);
    void command(const Geometry::Twist &twist, TimePoint now);
    void advance(TimePoint now);
    [[nodiscard]] Geometry::Pose2 truePose() const;
    [[nodiscard]] Geometry::Twist trueVelocity() const;
};

/** The A1M8 as measured [ours]: 7.3 rev/s, ~531 samples a revolution,
 *  turning clockwise, one ~8 degree gap a revolution, nothing nearer than
 *  126.8 mm, a share of no-returns. */
struct LidarConfig {
  Geometry::Vec2 offset{0.06, 0.0};
  double yawRad{0.0};             // the housing's front: mountingOffsetRad
  double revolutionsHz{7.3};
  double periodStdevS{0.0008};
  std::int32_t samplesPerRevolution{531};
  double syncDeg{358.6};          // sensor frame, where a revolution starts
  double gapStartDeg{40.0};       // sensor frame
  double gapWidthDeg{7.3};
  double minRangeM{0.1268};
  double maxRangeM{12.0};
  double noReturnFraction{0.7};   // of samples that would hit
  double rangeNoiseM{0.005};
  std::uint32_t seed{1};
};
class Lidar {
  public:
    Lidar(const World &world, LidarConfig config, TimePoint start);
    /** Samples the world along the robot's path up to now; returns a Scan
     *  each time a revolution completes. */
    [[nodiscard]] std::optional<Heron::Scan> advance(TimePoint now, const Base &base);
};

/** The OTOS: the true pose plus noise and a heading drift. N3. */
struct OtosConfig { double driftDegPerMin{0.5}; double noiseM{0.001}; std::uint32_t seed{1}; };
class Otos {
  public:
    Otos(OtosConfig config, TimePoint start);
    /** One reading, as PoseTracker would store it. */
    [[nodiscard]] Pose::PoseSample read(TimePoint now, const Base &base);
};

}  // namespace Heron::NavSim
```

The simulated LiDAR samples the world at each sample's own time, from
where the base was then, so its scans carry the same skew a real one does
and deskewing is tested against it (design §12.2). It makes each sample as
the sensor sends it (a Q14 angle, a Q2 distance, the sync flag) and builds
each revolution with the adapter's own `buildScan()`, so its scans keep
every `Scan` invariant by construction. `nav_sim_test` holds them against
the measured figures:

| | Simulated (30 s, Mac) | Findings [ours] | Recorded 2026-11-08 [ours] |
|---|---|---|---|
| Samples a revolution | 529 to 533, mean 531.0 | 526 to 532, mean 531 (express) | 549 to 554, mean 550.8 |
| Widest gap a revolution | 7.3 to 8.0°, mean 7.96° | 8.0 to 8.1° (medians per capture, express) | 6.5 to 8.3°, mean 7.50° |
| No-returns | 65 to 75% | 61 to 84% | 36 to 40% |
| Period | 136.5 to 137.5 ms | 134.2 to 138.1 ms | 139.6 to 142.2 ms (stamps) |

On the Pi the same seed gives a different run, still inside the test's
bounds: `std::mt19937` is the same everywhere, but the standard leaves
the distributions built on it to each library, and the Mac uses libc++,
the Pi libstdc++. So a test checks ranges, never exact values.

**The recording.** 220 revolutions in express mode, the robot still on
the bench rig, the Pi on its mains adapter (`lidar_record --mode 1
--scans 220`, replayed through the real adapter by `lidar_replay_check`:
217 scans, every invariant held, nothing dropped). What it settled:

- **The revolution time barely varies.** The sample rate is steady (3908
  a second here, 3904 in the findings), so the count follows the period;
  the count's spread of 0.75 samples puts the true period spread near
  0.2 ms. The findings' 0.8 ms is measured from SDK timestamps, which
  jitter (the count and the stamped period correlate only 0.40). The
  simulator used 0.8 ms at first, so its count spread to 521 to 538; it
  now uses 0.2 ms. A second recording, made for this by the LiDAR
  workstream, confirmed it: count-implied spread 0.14 ms, stamped 0.59
  ms, correlation 0.00 (findings, "Rate and timing").
- **The rotation rate was the LiDAR's connection, not the supply.** 7.10
  Hz here, 7.35 Hz in October. The LiDAR workstream found the LiDAR's
  micro-USB plug poorly seated at its adapter board; re-seated, on the
  same AC adapter, it turned at 7.52 Hz, period 131.9 to 134.1 ms, 519 to
  521 samples a revolution (findings, trap 12) [ours]. Later that day
  unit A's adapter board itself proved faulty, and unit B's replaced it
  (`rig-profiles.md` §1). Tests take the rate from `LidarConfig`, never
  assume 7.3 Hz.
- **No-returns depend on the room:** 36 to 40% here, 33 to 39% in the
  re-recording, 61 to 84% in the findings. They cluster: runs of 5.2
  samples on average, up to 70 (re-recording: 4.4, up to 25), where the
  simulator's random ones at 70% would average 3.3.
- **The gap sits at one bearing a capture,** 210 to 221° in 204 of 219
  revolutions here, 188° in the re-recording in the same room (the
  findings saw 40° and 106°), and the second-widest gap is 4.7°, so there
  is one gap each revolution, as simulated. Nothing may rely on where it
  is. The sync node sat at 355.2° here, 359.6° with the plug seated.
- **On the power block, the LiDAR misses commands.** Before this
  recording the Pi ran from the robot's power block, reported
  under-voltage every few seconds (`get_throttled` 0x50005), and the
  LiDAR answered only 4 of 20 GET_INFO requests; the SDK's start-up then
  failed. Re-tested the same evening with unit B's adapter board: on AC
  it answered every request, motor stopped or running; on the power
  block it answered with the motor stopped, but went silent for about
  5 s at a time with it running, 30 to 46 of 50 answered, while the Pi's
  under-voltage flag stayed mostly clear (`rig-profiles.md` §1) [ours].
  So recordings are made on AC until a supply is chosen.

One known difference remains: the simulated no-returns are scattered at
random, so a test that depends on clustering needs a recording.

Changes from the interfaces as accepted, made while building N2:

- **`Base` takes a start time,** as `Lidar` does. Without it, the first
  `advance()` could only set the clock, and the time before it was lost.
- **`LidarConfig` gained** the housing's yaw, the period's spread, the
  sync angle and the gap, each from the findings doc; the no-return share
  went from 0.6 to 0.7, which puts the simulated share inside the measured
  61 to 84% in a closed room.
- **The OTOS waits for N3:** it returns `Pose::PoseSample`, which
  `heron_pose` defines, and no N2 test needs it.

## 4. `heron_pose`: where Heron is, and was

Headers in `include/heron/pose/`. Links `heron_geometry` and
`heron_odometry` (for `PoseTracker` only).

### 4.1 `PoseHistory.h`: the core

```cpp
namespace Heron::Pose {

/** One odometry reading, at its own time. */
struct PoseSample {
  TimePoint time{};
  Geometry::Pose2 pose{};      // robot frame in the odom frame
  Geometry::Twist velocity{};  // robot frame
};

enum class LookupFailure : std::uint8_t { None, Empty, BeforeOldest, PastExtrapolation };

struct PoseLookup {
  bool ok{false};
  LookupFailure why{LookupFailure::Empty};
  Geometry::Pose2 pose{};
};

/**
 * The last 2 s of readings, a fixed ring of 256.
 *
 * Thread Safety: a plain value. PoseTracker holds one under its lock and
 * hands out copies.
 */
class PoseHistory {
  public:
    static constexpr std::size_t kCapacity = 256;
    static constexpr Duration kMaxExtrapolation = std::chrono::milliseconds(50);

    /** Adds a reading; one not later than the newest is ignored. */
    void add(const PoseSample &sample) noexcept;
    void clear() noexcept;

    /** Interpolated between the two readings around t, heading the short
     *  way round; up to 50 ms past the newest, extrapolated with its
     *  velocity (design §5). */
    [[nodiscard]] PoseLookup at(TimePoint t) const noexcept;

    [[nodiscard]] std::optional<PoseSample> newest() const noexcept;
    [[nodiscard]] std::size_t size() const noexcept;

  private:
    std::array<PoseSample, kCapacity> m_ring{};
    std::size_t m_next{0};
    std::size_t m_count{0};
};

}  // namespace Heron::Pose
```

A `PoseSample` is 56 bytes, so the history is about 14 KB: a copy is a
few microseconds on the Pi (estimate).

### 4.2 `PoseTracker.h`: the wrapper

```cpp
namespace Heron::Pose {

struct PoseConfig {
  Duration pollPeriod{std::chrono::milliseconds(10)};
  std::uint64_t maxSeqGap{10};  // more readings missed than this: not fresh
};

/** The newest pose, as of the call. */
struct PoseSnapshot {
  bool fresh{false};         // Odometry OK and no gap over maxSeqGap
  std::uint64_t epoch{0};    // which odom frame (I8); 0 before the first
  PoseSample newest{};
  bool tiltWarning{false};   // passed on, not acted on
  bool trackingWarning{false};
  std::string reason;        // why not fresh, for the robot channel
};

class PoseTracker {
  public:
    explicit PoseTracker(const Odometry::Odometry &odometry, PoseConfig config = {});
    ~PoseTracker();  // stops the thread
    PoseTracker(const PoseTracker &) = delete;
    PoseTracker &operator=(const PoseTracker &) = delete;

    bool start();
    void stop();

    [[nodiscard]] PoseSnapshot getLatest() const;
    /** A copy of the history and its epoch, taken together under the lock. */
    [[nodiscard]] std::pair<PoseHistory, std::uint64_t> copyHistory() const;
};

}  // namespace Heron::Pose
```

Each poll: `Odometry::getLatest(now)`; a new `seq` becomes a sample at
`now − age`, with `pose` and `velocity` from the snapshot. A `seq` lower
than the last one means odometry restarted: the history is cleared and
`epoch` goes up by one.

**The velocity's frame is unverified.** `PoseSample::velocity` is in the
robot frame. Whether the OTOS reports its velocity in its own (robot)
frame or in the odom frame is not in the reference yet; if the odom
frame, `PoseTracker` turns it with `rotate(v, −heading)`. It is checked
with the axes in N1 (design §18).

## 5. `heron_perception`: the obstacle map

Headers in `include/heron/perception/`. Links `heron_geometry`,
`heron_pose`, `heron_lidar_types`, and `heron_lidar` (for `Perception`
only).

### 5.1 `ObstacleGrid.h` and `ObstacleMap.h`: the core

```cpp
namespace Heron::Perception {

inline constexpr int kGridCells = 80;    // each way
inline constexpr double kCellM = 0.05;

/** Design §6.1; the starting values. */
struct GridConfig {
  std::int8_t hit{3};
  std::int8_t hitMax{12};
  std::int8_t pass{-1};
  std::int8_t passMin{-8};
  std::int8_t blockedAt{3};
  std::int8_t freeAt{-2};
  double recentreM{0.5};
};

enum class Cell : std::uint8_t { Unknown, Free, Blocked };
struct CellIndex { int ix{0}; int iy{0}; };

/** 80 x 80 signed counts, rolling with Heron, in the odom frame. */
class ObstacleGrid {
  public:
    explicit ObstacleGrid(GridConfig config = {});

    /** Forgets everything; the grid is centred on `centre`, its edges on
     *  multiples of 5 cm. */
    void reset(Geometry::Vec2 centre) noexcept;
    /** Shifts by whole cells when `robot` is more than recentreM from the
     *  centre; cells shifted in are unknown (design §6.3). */
    void follow(Geometry::Vec2 robot) noexcept;

    /** Lowers each cell from `from` to `to`, not `to` itself; then raises
     *  `to` if it is a hit. Either end may lie outside the grid. */
    void addRay(Geometry::Vec2 from, Geometry::Vec2 to, bool hit) noexcept;

    [[nodiscard]] Cell classify(Geometry::Vec2 p) const noexcept;  // outside: Unknown
    [[nodiscard]] std::optional<CellIndex> cellOf(Geometry::Vec2 p) const noexcept;
    [[nodiscard]] Geometry::Vec2 origin() const noexcept;  // corner of cell (0, 0)
    [[nodiscard]] const std::array<std::int8_t, kGridCells * kGridCells> &counts() const noexcept;
};

/** One map, as Perception publishes it: never changed once published. */
struct ObstacleMap {
  ObstacleGrid grid;
  std::uint64_t epoch{0};      // the odom frame it is in (I8)
  std::uint64_t scanSeq{0};    // the newest scan in it
  TimePoint scanEnd{};
  Geometry::Pose2 poseAtScanEnd{};
};

enum class ScanOutcome : std::uint8_t { Added, Stale, NotCovered, NoPose, NewFrame };

struct LidarMount { Geometry::Vec2 offset{}; };  // [lidar] x, y; yaw stays in Lidar (P4)

/**
 * Adds one scan to a map, design §6.2: checks it, places every point at
 * its own time, passes and hits. A new epoch resets the map first, and
 * the result says NewFrame (the scan is still added).
 */
ScanOutcome addScan(ObstacleMap &map, const Heron::Scan &scan, const Pose::PoseHistory &history,
                    std::uint64_t epoch, const LidarMount &mount, TimePoint now,
                    Duration maxScanAge = std::chrono::milliseconds(300));

}  // namespace Heron::Perception
```

`addRay()` takes odom-frame points rather than cells, so callers never
handle indices; the line is drawn with Bresenham's algorithm between the
two cells.

### 5.2 `Perception.h`: the wrapper

```cpp
namespace Heron::Perception {

struct PerceptionConfig {
  LidarMount mount{};
  GridConfig grid{};
  Duration maxScanAge{std::chrono::milliseconds(300)};
  Duration pollPeriod{std::chrono::milliseconds(10)};
};

struct PerceptionHealth {
  std::uint64_t scansAdded{0};
  std::uint64_t scansSkipped{0};  // Stale, NotCovered or NoPose
  std::uint64_t mapResets{0};     // NewFrame
  ScanOutcome lastOutcome{ScanOutcome::Added};
  std::string lastError;
};

class Perception {
  public:
    Perception(const Lidar::Lidar &lidar, const Pose::PoseTracker &pose, PerceptionConfig config);
    ~Perception();
    bool start();
    void stop();
    /** The newest map, or nullptr before the first. */
    [[nodiscard]] std::shared_ptr<const ObstacleMap> getLatest() const;
    [[nodiscard]] PerceptionHealth getHealth() const;
};

}  // namespace Heron::Perception
```

Each new scan: copy the last map, `addScan()` with `copyHistory()` and
the steady clock's now, publish the copy. The planner keeps reading the
old one meanwhile (design §6.4).

### 5.3 N3 as built (2026-11-08)

`heron_pose`, `heron_perception` and the simulated OTOS are built; `pose_test`,
`perception_test` and `nav_sim_test` pass on the Mac and the Pi, built with
no warnings by both compilers.
Additions to the interfaces above, made while building; nothing accepted
was changed:

- **`PoseFeed`** (`src/pose/PoseFeed.h`, private): what `PoseTracker`
  does with each reading (the epoch, the gap, freshness), without its
  thread or clock, so `pose_test` checks a restart or a gap in one call.
  The tracker is the thread round it, as P2 has every wrapper.
- **`PoseHistory::oldest()`**, beside `newest()`. **`PoseTracker::start()`
  polls once** before its thread starts, so `getLatest()` has an answer at
  once. A reading the OTOS flags as a sensor error is not stored.
- **`ObstacleGrid` gained** `centreOf()`, `centre()`, `count()` and
  `config()`, for tests and the telemetry. Cells are found by multiplying
  by 20 per metre, not dividing by 0.05: 0.05 is not exact in binary, and
  2.0 / 0.05 floors to 39. Non-finite points are ignored.
- **`addScan()`** takes the map's freshness limit as `kMaxScanAge` (300
  ms), and `toString(ScanOutcome)` names an outcome for the health. A new
  epoch resets the map even when the scan is then skipped.
- **`Perception`** publishes a map only when a scan was added, or when a
  new epoch reset it: a map in a frame that no longer exists is worse
  than an empty one.
- **The simulated OTOS** integrates its drift: each true step is applied
  in the drifted heading, so the position error grows with the distance
  driven, as a real one's does. It starts at the base's true pose.

Measured in `perception_test`: a turning robot (1.5 rad/s, 0.2 m/s) sees
a straight wall 1.2 m away; placed at each point's own time, every blocked
cell is within 25 mm of the wall (one cell); placed at the scan's end, 325
mm off on the Mac, 375 on the Pi (the random numbers differ between
standard libraries, §3).

**One hit blocks.** With the accepted counts (+3, blocked at +3), a
single hit blocks an unknown cell; a cell seen free (−2 or lower) takes
two. Design §6.1's "a point seen in two scans is enough to block" holds
only for the second case, and is corrected there.

## 6. `heron_limiter`: the safety limiter

Headers in `include/heron/limiter/`. Pure, no thread: the control loop
calls it every 20 ms. Links `heron_geometry`, `heron_pose`,
`heron_lidar_types`. It never sees the map (design §7).

```cpp
namespace Heron::Limiter {

struct LimiterConfig {
  Geometry::Outline outline{};
  Geometry::Vec2 lidarOffset{};
  double stopMarginM{0.05};
  double slowMarginM{0.25};
  double brakeMps2{1.0};
  Duration maxScanAge{std::chrono::milliseconds(300)};
  Duration scanPeriod{std::chrono::milliseconds(137)};
  Geometry::SweepStep step{};
};

enum class Reason : std::uint8_t {
  Clear, Cut, Slowed, BackingOff, NoScan, Stale, NotCovered, Unobserved
};

struct LimitInput {
  Geometry::Twist command{};              // m/s, rad/s
  const Heron::Scan *scan{nullptr};       // the newest, or nullptr
  const Pose::PoseHistory *history{nullptr};
  bool poseFresh{false};
  Geometry::Twist measured{};             // newest odometry velocity
  TimePoint now{};
};

struct LimitResult {
  Geometry::Twist command{};              // what may be driven
  Geometry::Twist scale{1.0, 1.0, 1.0};   // per axis, 0..1
  Reason reason{Reason::Clear};
  std::optional<Geometry::Vec2> nearest;  // robot frame, now
};

class Limiter {
  public:
    explicit Limiter(const LimiterConfig &config);
    /** Design §7.3 and §7.4. */
    [[nodiscard]] LimitResult limit(const LimitInput &input) const noexcept;
};

/**
 * The profile-time check of design §7.4: the LiDAR's 13 cm blind circle
 * must lie inside the stop outline.
 *
 * @return An empty string if it does, otherwise why not
 */
[[nodiscard]] std::string checkBlindSpot(const LimiterConfig &config);

}  // namespace Heron::Limiter
```

`limit()` is `const` and holds nothing between calls: the same input
always gives the same answer. Its points are placed in the robot frame at
their own time, then moved by `relative(poseAt(pointTime), poseAt(now))`
to where they are now; with the pose not fresh, that step is skipped and
the reaction time doubles (design §7.4).

### 6.1 The core as built (2026-11-08)

`heron_limiter` is built; `limiter_test` and `nav_scenario_test` pass on
the Mac and the Pi, built with no warnings by both compilers. Against the
interface above:

- **The rule is exact, not searched.** Scaling a twist leaves the path
  the outline sweeps unchanged; only the speed changes. So if a motion
  touches a point at t < T, the same motion scaled by t / T reaches it at
  exactly T, and each axis's cut, and the cut of the whole, is one sweep.
  The slow zone is the distance travelled before contact against
  `slowMarginM`; for a turn, the angle times the outline's half-diagonal.
- **Rule 4 needed more than "motion away"**, and is §6.2, below, in
  full: what is built, what it guarantees, what it costs, and the more
  complex alternative to weigh later.
- **`Unobserved` per axis:** vx and vy look for an unobserved arc within
  45° of their direction; a turn needs none anywhere. With the default
  coverage threshold (10°, the same as the unobserved arcs'), a scan that
  passes the coverage check has none, so this row cannot fire on its own
  today; it is kept for a coverage threshold set wider.
- **`LimitResult` gained `horizonS`**, the T used, and `toString(Reason)`.
  `BackingOff` is reported whenever something is inside the stop margin.
- **`heron_geometry` gained `Sweep`:** `firstContact()` for many points
  and one motion, the step poses worked out once. `firstContact()` is now
  a `Sweep` for one point (no longer `noexcept`: it allocates the steps).
  The first limiter took 4.8 ms a call on the Mac in the worst case (530
  points in reach, all three axes), most of it the sines and cosines of
  the steps, recomputed per point; with `Sweep` and skipping points a
  motion cannot reach, 0.34 ms; on the Pi, 0.68 ms, 3% of the 20 ms cycle
  (0.62 ms after the scenarios' speed-up, below). These Mac figures are the
  Debug build's; an optimised one is about 20 µs.

**The promise, tested:** in 2000 random scenes with random commands, no
limited command, held for T, brings a point within 4 cm of the body
(the 5 cm margin less 1 cm, the sweep's resolution: a 2° step moves a
corner 9 mm, and contact is found at a step's start; checked with steps
ten times finer). On the sweep's own steps, 19 scenes touched the 5 cm
outline up to 8.6 ms before T, which is that resolution.

**The scenarios** (`nav_scenario_test`, design §12.2 scenarios 2, 3 and
5): the simulated Heron driven by a stick through the limiter, in a 20 ms
loop in virtual time, the base braking at 0.6 m/s² and answering 60 ms
late, the limiter told the same brake.

| Scenario | Result (Mac; Pi) |
|---|---|
| 2: driven at a wall, 0.6 to 1.5 m away, at up to 60°, 0.2 to 0.6 m/s with a random sideways part, sensor noise on, 1000 runs | Never touched; nearest 23.5 mm (Mac), 35.6 mm (Pi); median stop 69 mm. With no range noise, nearest 49.2 mm (§6.2) |
| 3: pushed along a wall and into it, 6 s | Closed to 63 mm, kept sliding at 0.30 m/s |
| 5: the LiDAR stops after 2 s at 0.4 m/s | The command stops 302 ms after the last scan ended (Stale); the base is still 740 ms later (lag and braking) |

Scenario 2 first took 40 s on the Mac (the host preset is a Debug build)
and 95 s on the Pi. Its 1000 runs are independent, so their starts are
drawn first and the runs share the cores: 9 s on the Mac, 23 s on the
Pi's four, with the same results. Two changes to the libraries came with
it, worth having on the robot anyway: `wrapAngle()` returns an angle
already in range at once, without `fmod` (the largest single cost of a
limiter call), and the limiter moves its points with a table of Heron's
motion over the revolution, 65 moments interpolated, instead of a
pose lookup and two rotations per point. A worst-case limiter call on
the Pi went from 0.68 to 0.62 ms; the simulated LiDAR is the rest of the
scenario's time.

### 6.2 Rule 4: something already inside the margin (accepted 2026-11-08)

Design §7.3's rule 4 says: a point inside the stop outline now blocks only
motion that brings it closer; motion away is allowed, so Heron can always
back off. This section is how that is built, and why it took three tries.
The user accepted it for the first robots (2026-11-08), and asked for the
alternative's pros and cons to be weighed later (end of this section, and
design §18).

**Why the sweep alone cannot do it.** The sweep (§2.2, design §7.2) asks
when the stop outline (the body grown by `stopMarginM`, 5 cm) first
touches a point. A point already inside it touches at once, for every
motion, so the sweep alone would freeze Heron as soon as anything came
within 5 cm: it could not even back off. So points inside the margin need
a rule of their own, and the points outside it need an outline that the
first ones do not spoil.

**How close is "close": one distance, as the outline grows.** A point's
distance to the body is measured as `grown()` grows the outline: the
larger of its gap along x (`|x| - halfLength`) and its gap along y
(`|y| - halfWidth`). A point is inside the stop outline exactly when this
is 5 cm or less. Off a corner, this is less than the straight-line
distance; it is the distance the body's sides keep from the point when
the body passes it.

**What is built: three parts.**

1. **Every point inside the margin is held.** For each axis alone, a
   point in the margin stops the axis if the motion brings it closer than
   `closest`, the nearest any point is now. A motion that shrinks only one
   of the point's two gaps can bring it no nearer than the other gap,
   which it leaves alone; so it counts only if that other gap is below
   `closest`. A motion that shrinks both counts always. (`closesIn()`.)
2. **Every point outside the margin is swept** against the body grown by
   `closest` less 1 mm, not by the margin. A point coming in from beside,
   as the rest of a wall does when Heron slides along it, may then enter
   the margin as deep as what is already there, and once in, it is held
   by part 1. The 1 mm is so that a point level with the closest is not
   "touched" by the outline's edge (a contact on the edge counts).
3. **The cut motion is checked once more, as a whole.** Part 1 is not
   linear: two axes, each allowed alone, may together shrink both gaps of
   a point at once. If the cut motion brings any held point closer, the
   command is zero.

What each motion does, with a wall 3 cm in front of the body (2 cm inside
the margin), as `limiter_test` checks:

| Motion | What happens | Why |
|---|---|---|
| Forward | Stopped | The wall's points in front shrink their gap along x; their gap along y is below zero, below `closest` |
| Backward | Whole | It grows every gap |
| Sideways, either way | Whole | The wall's points past the side shrink their gap along y, but their gap along x is the wall's own 3 cm: no nearer than `closest` |
| Sideways and edging forward | The slide stays, the edging stops | Per axis: x is stopped, y is not |
| Turning | Stopped | A front corner swings into the wall: a point on the turning side shrinks its gap along x, and its gap along y is below zero |
| In a corner of two walls, backing out diagonally | Whole | It grows both walls' gaps |
| In that corner, sliding into the side wall | Stopped | The side wall's points shrink their gap along y; their gap along x is below zero |

**What it guarantees.** With exact ranges, the nearest anything is to the
body never shrinks while something is in the margin: a held point cannot
come below `closest` (part 1), a point from outside enters no deeper than
1 mm under it and is held from then on (part 2), and the two together are
checked (part 3). Measured in `nav_scenario_test`, scenario 2 (Heron
driven at a wall at 0.2 to 0.6 m/s, at up to 60° to it, 1000 runs) with
the simulator's range noise set to zero: the body came no nearer than
49.2 mm to the wall, against the 50 mm margin.

**How it got here: two versions that failed.**

- *First:* only the closest points (within 1 mm of `closest`) were held,
  and every other point was swept against the body grown by `closest`
  less 1 mm. `limiter_test` passed. Scenario 2 did not: 8 of 1000 runs
  touched the wall, the worst 24 mm into it, all at 51° to 60° and pushed
  fast. The margin ratcheted. Driven at a slanting wall, the next wall
  point along reached the shrunk margin, became the closest, and shrank
  it again; at a slant the closest point sits off the body's corner, and
  moving forward does not bring that one point closer, so nothing held
  Heron back. It crept in at 1 to 9 cm/s.
- *Second:* every point in the margin was held, and held strictly: no
  point might come any closer at all. Scenario 2 then kept 50.7 mm, but
  the slide along a wall in the margin stopped again: the wall's points
  past the side come closer along y as Heron slides, though no closer than
  the wall already is. Part 1's "no nearer than the other gap" is what
  tells those apart.

**What it costs.** For the first robots, accepted:

1. **While anything is in the margin, the margin is that close for
   everything.** An obstacle coming from outside the margin may come as
   near as the thing already in it. Sliding along a wall at 3 cm, a box
   ahead may be approached to 3 cm, not 5. The margin returns to 5 cm as
   soon as nothing is in it.
2. **Range noise erodes the margin.** `closest` is the least of many
   noisy distances, so it reads low, and part 1's "no nearer than
   `closest`" lets some approach through. With the simulator's 5 mm on
   each range, scenario 2's closest was 23.5 mm on the Mac and 35.6 mm on
   the Pi (whose random numbers differ), against 49.2 mm with no noise.
   The 5 mm is a guess: the A1M8's range noise has not been measured
   (design §18). Nothing touched in any run.
3. **It is cautious at corners and when turning.** A motion that shrinks
   both of a point's gaps is stopped however little it does so; turning
   with anything in the margin near a front or back edge is usually
   stopped. Backing off is never stopped.
4. **It keeps nothing between calls,** so it cannot know that `closest`
   was larger a moment ago; every cycle judges afresh. That is what makes
   it testable call by call (P2), and also what lets noise move `closest`.

**The alternative, for later: a margin per side.** Keep four margins, one
for each side of the body (front, back, left, right), each shrunk only by
what is in the margin on that side; the other sides keep the full 5 cm.

- *What it would improve:*
  - A wall in the margin on one side no longer lowers the margin on the
    others: sliding along a wall at 3 cm, a box ahead is still kept 5 cm
    away (cost 1 goes).
  - The report can say which side is too close, for the `limiter`
    channel and for the planner, which could steer away from that side.
  - Noise on one side's points erodes only that side's margin.
- *What it would cost:*
  - More code in the one place where a bug drives Heron into something.
  - The outline is no longer centred on the robot frame's origin. The
    sweep handles that by moving to the outline's centre c and expressing
    the twist there (`v + w × c`), but that is new geometry, to be tested
    as `firstContact` was.
  - A point off a corner is on two sides at once, and needs a rule for
    which side's margin it shrinks: the same corner question part 1
    answers, now per side.
  - Noise still erodes the side that is shrunk (cost 2 stays there).
  - In the common case, one wall, it behaves exactly as the built rule.
- *Noise, whichever is chosen,* could be reduced by: measuring the A1M8's
  range noise first (if it is 1 to 2 mm, the erosion shrinks with it);
  taking `closest` as the k-th nearest point, or a percentile, rather than
  the least; smoothing each range with its neighbours'; or letting the
  limiter keep `closest` between cycles, lowered only by measured motion
  (which gives up "keeps nothing", P2).
- *When to weigh it:* once N1 has measured the range noise; when Heron
  first drives near walls in Assisted; and before a larger, faster Heron,
  where a margin taken by a wall on one side matters more.

## 7. `heron_robot`: what joins them (N4)

`heron_robot`'s control loop is a program, not a library, but its
decisions are kept in small pure functions so they are tested like the
cores. Private headers in `src/robot/`.

### 7.1 The profile sections

`heron_profile` gains four optional sections (P11, design §10), read
with its existing rules; a rig without wheels has no `[geometry]`:

```cpp
namespace Heron::Profile {
struct GeometryProfile   { double length, width, maxSpeed, maxTurnRate, accel, brake; };
struct LidarProfile      { double x, y, yawDeg, height; };
struct OdometryProfile   { double x, y, yawDeg, linearScalar, angularScalar; };
struct NavigationProfile { double stopMargin, slowMargin; };

// added to RigProfile:
std::optional<GeometryProfile> geometry;
std::optional<LidarProfile> lidar;
std::optional<OdometryProfile> odometry;
std::optional<NavigationProfile> navigation;
}
```

`heron_robot` fills each library's config from these (I6), and refuses
to enter Assisted or Navigate on a rig whose profile lacks any of them,
or whose LiDAR fails `checkBlindSpot()`.

### 7.2 Modes and units

```cpp
enum class Mode : std::uint8_t { Manual, Assisted, Navigate };

/** SwC's position as a mode; which end is Manual is set once SwC is
 *  checked (design §18). */
[[nodiscard]] Mode modeFromSwitch(Heron::Remote::Switch3 swc, bool manualIsHigh);

/** Fractions of full drive to m/s and rad/s, and back: the one place
 *  the two meet (I2). */
[[nodiscard]] Geometry::Twist toTwist(const Drive::BodyVelocity &v, const GeometryProfile &g);
[[nodiscard]] Drive::BodyVelocity toBodyVelocity(const Geometry::Twist &t, const GeometryProfile &g);
```

### 7.3 One control cycle, Assisted

```cpp
const TimePoint now = Clock::now();
const RemoteSnapshot r = remote.getLatest();
const bool armed = gate.update(r.isOk(), Drive::readThrottle(r.frame, sticks));
const Drive::BodyVelocity stick = Drive::sticksToVelocity(r.frame, sticks);

Drive::BodyVelocity out{};
switch (mode) {
  case Mode::Manual:
    out = stick;                                   // as heron_teleop, nothing else
    break;
  case Mode::Assisted: {
    const std::shared_ptr<const Scan> scan = lidar.getLatest();
    const auto [history, epoch] = pose.copyHistory();
    const Pose::PoseSnapshot p = pose.getLatest();
    const Limiter::LimitResult limited = limiter.limit(
        {toTwist(stick, geometry), scan.get(), &history, p.fresh, p.newest.velocity, now});
    out = toBodyVelocity(limited.command, geometry);
    break;
  }
  case Mode::Navigate:
    // N5: the planner's twist instead of the stick, then the same limiter.
    break;
}
motors.drive(armed ? Drive::mix(out) : Drive::WheelValues{});
```

Manual touches nothing new: no scan, no pose, no limiter, as D5 requires.
A test drives Manual and `heron_teleop` with the same recorded remote
frames and compares the wheel values.

## 8. `heron_planner`: the local planner (N5)

**Accepted 2026-11-09** (I10 to I16 below, as proposed). Design §8 and P9
are what it implements; where it changes them, it says so. Headers in
`include/heron/planner/`. The core links `heron_geometry` and
`heron_perception` (for `ObstacleMap`); `LocalPlanner` also links
`heron_pose` and `Perception`'s wrapper.

What it does, every 100 ms in Navigate: take the newest map and pose, try
about 121 sideways and forward speeds, imagine Heron holding each for
1.5 s, throw out any that come too near something, score the rest, and
hand on the best. It never chooses the turn rate (D3), and whatever it
chooses still goes through the limiter (P7, P8).

### 8.1 Decisions

| # | Proposal | Why |
|---|---|---|
| I10 | **Collision and clearance come from a distance field,** built once a cycle from the map, not from the limiter's sweep. P7 says the planner uses the same sweep as the limiter; this changes that | Measured on the Mac with throwaway prototypes, release build, 121 candidates against 600 cells: the sweep takes 0.5 ms going straight but **27 ms at full turn** (each candidate is stepped every 2°); the field takes **0.05 ms to build and 0.15 ms** for the same candidates, turning or not. The limiter ran about 30 times slower on the Pi than in the Mac's optimised build (20 µs against 0.62 ms, §6.1), so the sweep would take about 800 ms of the 100 ms cycle while turning, the field about 5 ms [estimate]. The limiter keeps the exact sweep, on raw points |
| I11 | **The planner keeps 8.3 cm more than the limiter's margin** (the cell slack, §8.4) | The map is cells, not points: a blocked cell's hit can be anywhere in its 5 cm square, and the field is read at cell centres. So the planner's choices normally pass the limiter untouched, and the limiter cuts only what the map got wrong |
| I12 | **Already close: a candidate may not bring Heron nearer than it is now.** While the nearest cell is inside the planner's margin, a candidate passes if its path's clearance never drops below today's | The planner's own rule 4. It cannot ratchet the way the first limiter rule 4 did (§6.2): the floor is "now", with no slack, and the limiter's rule 4 stays the safety |
| I13 | **Speed is capped at what the stick asks** (its size × throttle × `max_speed`), not at `max_speed` | Otherwise progress, which scores distance gained, always picks full speed, and a half stick means nothing |
| I14 | **Candidates never move backwards along the course, nor end more than 1 m off its line;** none left is Blocked | A dead end then stops Heron (scenario 4) instead of backing it out and in again, and a long wall cannot draw it off sideways for ever. Backing away is Manual's or Assisted's job. Both numbers are starting values |
| I15 | **The course is kept by `CourseKeeper`,** a pure class in this library, which the control loop updates every 20 ms | Design §8.1's rules (engage, re-anchor past 10°, let go to stop) are tested in one call each, and the 10 Hz thread only reads the course |
| I16 | **The turn rate is the stick's, passed in with the course;** the planner simulates it but never searches it | D3 and design §8.1: heading is yours. Turning still moves the outline, so each candidate is imagined with it |

### 8.2 The course: `CourseKeeper`

```cpp
namespace Heron::Planner {

/** What the stick asks for, design §8.1, in the odom frame. */
struct Course {
  std::uint64_t seq{0};        // raised on every engage and re-anchor
  std::uint64_t epoch{0};      // the odom frame it is in (I8)
  Geometry::Vec2 anchor{};     // the line runs through here...
  double headingRad{0.0};      // ...in this direction
  double speedMps{0.0};        // the stick's size x throttle x max_speed (I13)
  double turnRateRadps{0.0};   // the left stick's, as in Manual (I16)
};

struct CourseConfig {
  double reanchorRad{10.0 * Geometry::kDegToRad};
};

/**
 * Turns the stick into a course and keeps it while the stick is held.
 * Pure: the control loop calls update() every 20 ms in Navigate.
 */
class CourseKeeper {
  public:
    explicit CourseKeeper(CourseConfig config = {});

    /**
     * @param stick  The sticks as a twist, m/s and rad/s (toTwist(), §7.2)
     * @param pose   Heron now, odom frame
     * @param epoch  The pose's epoch
     * @return The course, or no value when the right stick is centred
     */
    std::optional<Course> update(const Geometry::Twist &stick, const Geometry::Pose2 &pose,
                                 std::uint64_t epoch);

    void reset() noexcept;  // leaving Navigate
};

}  // namespace Heron::Planner
```

Each call:

- **Right stick centred** (vx and vy both zero after the stick's dead
  band): no course; Heron stops (D1). The next push engages afresh.
- **Engage:** the stick's direction turned into the odom frame by Heron's
  heading is the heading; the line runs through Heron's position; `seq`
  rises.
- **Held:** the speed and turn rate follow the stick every call. If its
  odom-frame direction drifts more than 10° from the heading (the stick
  moved, or Heron turned under it), re-anchor at the current position.
- **A new epoch** (odometry restarted): re-anchor, since the old anchor
  is in a frame that no longer exists.

### 8.3 The core: `Planner`

```cpp
namespace Heron::Planner {

/** Design §8.2's starting values; tuned in the simulator (design §18, 3). */
struct ScoreWeights {
  double clearance{1.0};
  double progress{2.0};
  double offLine{1.0};
  double change{0.5};
};

struct PlannerConfig {
  Geometry::Outline outline{};          // [geometry] length and width, halved
  double stopMarginM{0.05};             // [navigation] stop_margin
  double maxSpeedMps{0.35};             // [geometry] max_speed
  double accelMps2{0.6};                // [geometry] accel
  double windowS{0.3};                  // speeds reachable in this long
  int samples{11};                      // per axis: 11 x 11 candidates
  double horizonS{1.5};                 // each candidate held this long
  double stepS{0.05};                   // checked this often (§8.4)
  double pathStepS{0.1};                // the published path's spacing
  double unknownRadiusM{0.6};           // unknown is blocked this near (P6)
  double clearanceCapM{0.5};            // more clearance scores no higher
  double maxOffLineM{1.0};              // I14
  Duration maxMapAge{std::chrono::milliseconds(500)};
  ScoreWeights weights{};
};

enum class State : std::uint8_t {
  Following,  // on the course
  Avoiding,   // off it, to get round something
  Blocked,    // no candidate passes: stopped (I14)
  Stopped     // cannot plan: see Reason
};

enum class Reason : std::uint8_t {
  None,          // Following or Avoiding
  NothingSafe,   // Blocked: every candidate came too near or broke I14
  NoCourse,      // the stick is centred
  NoMap,         // no map yet
  StaleMap,      // the map is older than maxMapAge
  PoseNotFresh,
  OtherFrame     // the map, pose and course are not all in one epoch
};

/** One cycle's answer; never changed once published. */
struct Plan {
  Geometry::Twist twist{};                 // vx, vy chosen; wz the course's
  std::vector<Geometry::Pose2> path;       // odom frame, every pathStepS, for telemetry
  State state{State::Stopped};
  Reason reason{Reason::NoCourse};
  std::uint64_t courseSeq{0};              // the course it was made for
  double clearanceM{0.0};                  // the chosen path's, after the cell slack
  int candidatesPassed{0};                 // of those tried
  TimePoint time{};                        // when it was made
};

/** What one cycle reads. Borrowed for the call only. */
struct PlanInput {
  const Perception::ObstacleMap *map{nullptr};
  Geometry::Pose2 pose{};                  // Heron now (PoseHistory::at(now))
  std::uint64_t epoch{0};
  bool poseFresh{false};
  Geometry::Twist measured{};              // newest odometry velocity
  std::optional<Course> course;
  TimePoint now{};
};

/**
 * Thread Safety: a plain value. plan() changes the remembered last
 * choice, so one Planner belongs to one thread.
 */
class Planner {
  public:
    explicit Planner(const PlannerConfig &config);

    /** Design §8.2, with I10 to I14. */
    [[nodiscard]] Plan plan(const PlanInput &input);

    void reset() noexcept;  // forget the last choice

  private:
    PlannerConfig m_config;
    std::optional<Geometry::Twist> m_last;  // for the change score
};

[[nodiscard]] const char *toString(State state) noexcept;
[[nodiscard]] const char *toString(Reason reason) noexcept;

}  // namespace Heron::Planner
```

Each call:

1. **Can it plan?** No course, no map, a map older than `maxMapAge`, a
   pose not fresh, or the map, pose and course in different epochs:
   Stopped, with the reason. A zero twist; the last choice is forgotten.
2. **The field** (§8.4), from the map and Heron's pose now.
3. **Candidates** (design §8.2): `samples` × `samples` values of vx and
   vy, robot frame, evenly over the measured velocity ± `accelMps2` ×
   `windowS`, keeping only those no faster than the course's speed
   (I13); plus the straight candidate, the course's direction at its
   speed, if it is reachable. The turn rate is the course's in all.
4. **Imagine each** for `horizonS`, every `stepS`, with `integrate()`
   from Heron's pose. Reject it if its clearance at any step is below the
   planner's margin (unless I12 lets it through), if it moves Heron
   backwards along the course heading, or if it ends more than
   `maxOffLineM` from the line (I14).
5. **Score the rest** (§8.5) and **choose** the best: Following if it is
   the straight candidate and Heron is within 5 cm of the line, otherwise
   Avoiding. None left: Blocked, zero.

### 8.4 The distance field

Every cell's distance to the nearest cell Heron must keep clear of, in
metres, built fresh each cycle (80 × 80, from the map). The cells to keep
clear of:

- every **Blocked** cell;
- every **Unknown** cell within `unknownRadiusM` of Heron's centre (P6),
  **except** those under Heron and its margin now: the outline grown by
  `stopMarginM` and the cell slack, at the current pose. They are the
  LiDAR's blind circle and the robot's own body; nothing can be seen
  there, and `checkBlindSpot()` has already put the blind circle inside
  the margin.

The distance is the exact Euclidean distance between cell centres, by the
two-pass distance transform of Felzenszwalb and Huttenlocher (a 1-D pass
along each row, then each column). Measured on the Mac: 0.05 ms for the
grid.

**A candidate's clearance at one step** is the least field value at
points 2.5 cm apart round the outline's edge (34 points for 0.24 × 0.20
m), each read from the cell it falls in. Its clearance is the least over
its steps. Reading the edge alone is enough because nothing to keep clear
of starts inside the outline (the exception above), and between steps the
edge moves less than the margin, so nothing can cross it unseen: at 0.35
m/s a 0.05 s step moves it 1.75 cm, and at 2 rad/s a corner 15.6 cm out
moves 1.6 cm. The step is what keeps this true, which is why it is 0.05 s
and not design §8.2's 0.1 s.

**The cell slack, 8.3 cm**, is the most that true clearance can fall short
of the field's: half a cell's diagonal for where the hit lies in its cell
(3.54 cm), the same for reading the field at a cell's centre (3.54 cm),
and half the 2.5 cm spacing round the edge (1.25 cm). A candidate passes
if its clearance less the slack is at least `stopMarginM`; so the planner
keeps 5 cm or more in truth, and usually about 9 to 13 [estimate]. The
narrowest gap it will drive through is about 0.20 + 2 × 0.133 = 0.47 m on
the bench rig, at worst [estimate]; a finer grid would lower it, later.

**Cost.** 121 candidates × 30 steps × 34 points: about 120,000 field reads
a cycle, and 4,000 `integrate()` calls. Measured on the Mac, release:
about 0.2 ms a cycle, the field included (a prototype). On the Pi, about 5 ms by §6.1's ratio [estimate]: to be measured in
N5's first step; budget 20 ms of the 100.

### 8.5 The score

As accepted; built with five terms and progress toward a point ahead,
§8.9. For each candidate that passes, with H = `horizonS` and s the course's
speed, each term scaled to 0..1:

| Term | Value | Weight (start) |
|---|---|---|
| Clearance | min(clearance less the slack, `clearanceCapM`) / `clearanceCapM` | + 1.0 |
| Progress | distance along the course heading in H / (s × H) | + 2.0 |
| Off the line | distance from the course line at the end / `maxOffLineM` | − 1.0 |
| Change | distance in (vx, vy) from the last choice / (2 × `maxSpeedMps`) | − 0.5 |

The best total wins. Progress outweighs the rest at the start, so on an
empty map the straight candidate wins; clearance and the line pull it
round a box and back. All four weights are design §18's item 3, settled
by scenario 1.

### 8.6 The wrapper: `LocalPlanner`

```cpp
namespace Heron::Planner {

struct LocalPlannerConfig {
  PlannerConfig planner{};
  Duration period{std::chrono::milliseconds(100)};
};

struct PlannerHealth {
  std::uint64_t plans{0};
  std::uint64_t blocked{0};
  std::uint64_t stopped{0};
  Reason lastReason{Reason::NoCourse};
  Duration lastCycle{};        // how long plan() took: design §18, item 2
  Duration longestCycle{};
};

class LocalPlanner {
  public:
    LocalPlanner(const Perception::Perception &perception, const Pose::PoseTracker &pose,
                 LocalPlannerConfig config);
    ~LocalPlanner();
    bool start();
    void stop();

    /** From the control loop, every 20 ms; no value stops Heron. */
    void setCourse(const std::optional<Course> &course);

    /** The newest plan, or nullptr before the first. */
    [[nodiscard]] std::shared_ptr<const Plan> getLatest() const;
    [[nodiscard]] PlannerHealth getHealth() const;
};

}  // namespace Heron::Planner
```

Every 100 ms: read the course (under its lock), `perception.getLatest()`,
`pose.copyHistory()` and `getLatest()`, the pose at the steady clock's
now, call `plan()`, publish. It plans only while a course is set, so in
Manual and Assisted it sleeps.

### 8.7 In the control loop

The Navigate case of §7.3:

```cpp
case Mode::Navigate: {
  const std::optional<Planner::Course> course =
      courseKeeper.update(toTwist(stick, geometry), p.newest.pose, p.epoch);
  localPlanner.setCourse(course);
  const std::shared_ptr<const Planner::Plan> plan = localPlanner.getLatest();
  Geometry::Twist want{};                          // stop, unless...
  if (course && plan && plan->courseSeq == course->seq && now - plan->time <= 300ms) {
    want = {plan->twist.vx, plan->twist.vy, course->turnRateRadps};
  }
  const Limiter::LimitResult limited = limiter.limit(
      {want, scan.get(), &history, p.fresh, p.newest.velocity, now});
  out = toBodyVelocity(limited.command, geometry);
  break;
}
```

- **Letting go stops at once,** in the loop, not 100 ms later in the
  planner (D1).
- **A plan made for an older course is not driven** (`courseSeq`), so a
  re-anchored stick never gets 100 ms of the old direction.
- **The turn rate is the stick's now,** not the one the plan was made
  with.
- **Older than 0.3 s is stop,** design §8.2.

### 8.8 Tests

| Test | Checks |
|---|---|
| `planner_test` | `CourseKeeper`: engage, held at 9°, re-anchored at 11°, let go, a new epoch. The field against brute force on random grids. `Planner`: straight on an empty map (Following, the straight candidate); a box ahead (Avoiding, never below the margin); a dead end (Blocked); unknown within 0.6 m avoided, beyond it ignored, under Heron ignored; already close, moving away allowed and nearer refused (I12); each Stopped reason; the window and the speed cap; nothing backwards, nothing past 1 m off the line |
| `nav_scenario_test` | Design §12.2 scenarios 1 and 4, with the planner, the limiter and the simulator together. Also counts how often the limiter changes the planner's choice in scenario 1: I11 expects almost never |
| On the Pi | `plan()`'s time, from `PlannerHealth`, against the 20 ms budget |

### 8.9 As built (2026-11-09)

`heron_planner` is built: `CourseKeeper.h`, `Planner.h`, `LocalPlanner.h`,
and the private `src/planner/DistanceField.h`. `planner_test` and
scenarios 1 and 4 pass on the Mac, built with no warnings by both
compilers (the Pi's build, in the container, also clean); not yet run on
the Pi, which was off. Against the interfaces above:

- **The score has five terms, and progress is scored toward a point
  ahead.** Scenario 1 showed the four of §8.5 cannot pass a box and
  return to the line together: with progress along the course heading,
  angling back gains too little in 1.5 s to beat going straight on, so
  Heron stayed 43 cm off the line; weighting the line more (2.0) made it
  slow head-on into the box instead (111 plans Blocked). As built:
  - **Progress is measured toward the point on the line `lookAheadM`
    (1 m) ahead of Heron's place along it,** as a driver steers for a
    point ahead: on the line, straight on; off it, back at an angle that
    eases as it nears. Weight 2.0.
  - **A fifth term, the free way** (weight 1.0): how far the candidate
    runs, out to 1 m whatever its speed, before it comes nearer anything
    than `preferredClearanceM` (0.15 m), or, already that near, nearer than
    Heron is now. It is the distance term of the dynamic window approach.
    Without it a slow candidate that stops short of a box within the 1.5 s
    outscored going round it, and two ways that both met the box scored
    alike. Measured to 15 cm, not the margin, so Heron keeps 15 cm where
    there is room and uses the margin only where there is not; the "now"
    floor is what stops the pull back to the line cutting a box's corner.
  - Clearance (over the horizon, capped at 0.5 m), the line (at the
    horizon's end) and change keep §8.5's form and weights, 1.0, 1.0 and
    0.5. Clearance at 2.0 and 3.0, and a preferred clearance of 0.20 m,
    were tried and were worse.
- **I14, as built: a candidate must make at least 1 cm of progress along
  the course** (`minProgressM`), not just none backwards. Otherwise the
  zero candidate passes in a dead end and Heron "avoids" standing still.
- **The field is read interpolated** between the four nearest cell
  centres, not at the cell a point falls in. Read by cell, a 4 cm creep
  toward a wall inside one cell reads as no nearer, and I12 let it
  through: the ratchet I12 exists to stop. The distance is 1-Lipschitz,
  so interpolation is out by no more than half a cell's diagonal, as the
  nearest-cell read was; the cell slack is unchanged.
- **The straight candidate is tried first** and wins a tie, so on the
  line it is Following, not Avoiding with the same twist.
- **Added to the config:** `preferredClearanceM` (0.15), `lookAheadM`
  (1.0), `minProgressM` (0.01), `onLineM` (0.05); to `Plan`,
  `candidatesTried`. **`LocalPlanner` plans at once for a new course
  seq**, not at its next tick, since the loop drives no plan for an older
  one.
- **Navigate with the right stick centred is stopped, turning included:**
  no course, no plan. Turning on the spot in Navigate is not possible in
  this version; switch to Assisted or Manual.

Measured, scenario 1 over ten box positions (two distances, five offsets
from the line, dead centre included), Mac: every run passes the box with
157 to 175 mm to spare and is back within 42 to 84 mm of the line 2 m
past it; the limiter cut none of the planner's choices, and only slowed
it in its 25 cm slow zone (7 to 117 cycles of about 830). Thin against
the targets (15 cm, 10 cm): the weights are tuned to these ten runs.
Scenario 4: Blocked and still 209 mm from the dead end, never within the
margin.

`plan()` with a box ahead, turning at 2 rad/s: 6.1 ms on the Mac's Debug
build, 0.45 ms optimised. On the Pi about 14 ms by the limiter's ratio
[estimate], inside the 20 ms budget but not the 5 ms §8.1 estimated: the
free way's look ahead and the interpolation are the difference. To be
measured.

**Found on the way, perception's:** a cell the true line of a ray crosses
can be skipped, since `addRay()` steps from cell to cell (Bresenham). Rays
from one point, as the LiDAR's are, covered every cell in the test; rays
started 0.2 m out left five cells near 0.55 m never passed. Not changed;
noted here for when perception is revisited.

### 8.10 Still to settle

- **I13, I14:** the speed cap, no backing, and 1 m off the line are
  choices about how Navigate should feel, accepted as starting points;
  driving on the floor may change them.
- **The weights** are tuned to scenario 1's ten runs (§8.9), with little
  to spare; the floor will settle them.
- **`plan()`'s time on the Pi,** against the 20 ms budget.

## 9. Tests, by phase

| Phase | Test | Checks |
|---|---|---|
| N2 | `geometry_test` | §2.3's numbers; compose and inverse round trips; `integrate` against hand-worked arcs; `firstContact` front, side, corner, behind, already inside, with rotation; wrap at ±π |
| N2 | `nav_sim_test` | The simulated LiDAR's scans keep every `Scan` invariant; counts, gaps and timing within the measured ranges; a still robot in a square room sees four walls where they are |
| N3 | `pose_test` | Interpolation, the short way round; extrapolation to 50 ms and not past it; a restart raises the epoch |
| N3 | `perception_test` | Design §12.1's list; a skewed scan of a straight wall, from the simulator, comes out straight |
| N4 | `limiter_test` | Each rule of design §7.3 and row of §7.4; the blind-spot check |
| N4 | `nav_scenario_test` | Design §12.2 scenarios 2, 3 and 5, with the simulator and the cores |
| N5 | `planner_test` | §8.8: `CourseKeeper`'s rules; the field against brute force; `Planner` on hand-built maps |
| N5 | `nav_scenario_test` | Design §12.2 scenarios 1 (ten box positions) and 4, with perception, the planner and the limiter |
| N4 | `robot_test` | Manual gives `heron_teleop`'s wheel values for the same frames; the mode switch; the unit conversions |

All run on the Mac and on the Pi, as the existing tests do.
