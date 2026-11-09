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
glue them. The planner (N5) is sketched in §8 and written out when N5
starts.

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
| I9 | **Namespaces follow the libraries:** `Heron::Geometry`, `Heron::Pose`, `Heron::Perception`, `Heron::Limiter`, `Heron::NavSim`, as `Heron::Odometry` and `Heron::Lidar` | One library, one namespace, as now |

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
- **The rotation rate was the plug, not the supply.** 7.10 Hz here, 7.35
  Hz in October. The LiDAR workstream found the LiDAR's micro-USB plug
  poorly seated at its adapter board; re-seated, on the same mains
  adapter, it turned at 7.52 Hz, period 131.9 to 134.1 ms, 519 to 521
  samples a revolution (findings, trap 12) [ours]. Tests take the rate
  from `LidarConfig`, never assume 7.3 Hz.
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
  failed. On the mains adapter it answered 20 of 20 with the motor
  stopped, and recorded cleanly, though only 11 of 20 with the motor
  running and requests 50 ms apart. The 11 of 20 was the poorly seated
  plug: re-seated, it lost none of 1556 (findings, trap 12) [ours]. The
  power block's 4 of 20 has not been re-tested; the plug may have added
  to the under-voltage [unverified].

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

## 8. The planner, in outline (N5)

Written out when N5 starts. Its shape, so N2 to N4 leave room for it:

```cpp
namespace Heron::Planner {
struct Course { Geometry::Vec2 anchor; double heading; double speed; };
struct Plan { Geometry::Twist twist; std::vector<Geometry::Pose2> path;
              State state; std::string reason; TimePoint time; };
class Planner { Plan plan(const Perception::ObstacleMap &map, const Pose::PoseSnapshot &pose,
                          const Course &course, TimePoint now); };  // keeps the last choice
class LocalPlanner { /* the 10 Hz thread, getLatest() -> shared_ptr<const Plan> */ };
}
```

`Planner` keeps one piece of state, the last choice, for the smoothness
score; it is a member, still with no clock or thread inside.

## 9. Tests, by phase

| Phase | Test | Checks |
|---|---|---|
| N2 | `geometry_test` | §2.3's numbers; compose and inverse round trips; `integrate` against hand-worked arcs; `firstContact` front, side, corner, behind, already inside, with rotation; wrap at ±π |
| N2 | `nav_sim_test` | The simulated LiDAR's scans keep every `Scan` invariant; counts, gaps and timing within the measured ranges; a still robot in a square room sees four walls where they are |
| N3 | `pose_test` | Interpolation, the short way round; extrapolation to 50 ms and not past it; a restart raises the epoch |
| N3 | `perception_test` | Design §12.1's list; a skewed scan of a straight wall, from the simulator, comes out straight |
| N4 | `limiter_test` | Each rule of design §7.3 and row of §7.4; the blind-spot check |
| N4 | `nav_scenario_test` | Design §12.2 scenarios 2, 3 and 5, with the simulator and the cores |
| N4 | `robot_test` | Manual gives `heron_teleop`'s wheel values for the same frames; the mode switch; the unit conversions |

All run on the Mac and on the Pi, as the existing tests do.
