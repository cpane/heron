# Odometry C++ — design and intended use

How the OTOS library is layered, and how the robot is meant to use it.
**Built 2026-10-18** on `feature/odometry-library`; the live check on the
sensor is `odometry_adapter`. The sensor itself is in
[`odometry-otos-reference.md`](odometry-otos-reference.md); every behaviour
this design relies on is cited there by section.

Written from the reference, not from SparkFun's code: none of it may be
copied without the user's approval (reference §9).

## 1. Layers

```
heron_otos         pure: OtosCodec -- register numbers, scalings, decoding a
                   snapshot to SI, encoding scalars and the offset. No I/O,
                   no threads.
  ^
heron_otos_device  I2cBus: the interface to the bus. LinuxI2cBus: i2c-dev,
                   one combined write-then-read per transaction.
                   OtosDevice: bus + codec + the start-up sequence,
                   synchronous.
  ^
heron_odometry     Odometry: a reader thread publishing the newest reading.
                   What robot code links.

otos_bus, otos_start,
otos_move, otos_spin   probes, as now: raw registers, kept for diagnosis
odometry_adapter       probe: Odometry live, as a 50 Hz loop sees it
```

As with the remote, each row links the rows above it, publicly.

**Why `I2cBus` is an interface.** The remote's tests drive the real port
code through a pseudo-terminal. There is no host equivalent for I2C, and
`linux/i2c-dev.h` does not exist on macOS. With the bus behind a small
interface, `OtosDevice` and `Odometry` take a fake bus in tests and build
and run on the host; only `LinuxI2cBus` needs the Pi. On a non-Linux host,
opening a `LinuxI2cBus` fails with a clear error, so `Odometry(config)`
still compiles everywhere and `start()` reports the failure through
`lastError`, like a missing device.

**Register numbers and scalings stay private.** Every header below
`Odometry` lives in `src/odometry/` (codec, `I2cBus`, `OtosDevice`), like
the LiDAR's `ScanBuilder.h`: visible to tests and probes, which are given
the path, but not to robot code. The public headers in
`include/heron/odometry/` are `Planar.h`, `OtosConfig.h` and `Odometry.h`,
and speak metres, radians and seconds only. `Odometry.h` names `I2cBus`
only as a forward declaration, for the test constructor that takes a bus
factory.

## 2. The robot's view: `Odometry`

Shaped like `Heron::Lidar::Lidar` and `Heron::Remote::RemoteControl`, so the
robot core treats every sensor the same way:

- `start()` and `stop()` belong to one owning thread.
- One reader thread (`std::jthread`, named `heron-otos` on Linux) polls the
  sensor at a fixed period and publishes the newest reading.
- `getLatest()` and `getHealth()` are callable from any thread and hold a
  mutex only to copy the snapshot.
- **Latest wins.** A consumer polling slower than the reader skips
  readings; `seq` shows how many.

### 2.1 Configuration

```cpp
namespace Heron::Odometry {

/** x and y with a heading: m, m, rad -- or their rates, by context. */
struct Planar {
  double x{0.0};
  double y{0.0};
  double h{0.0};
};

struct OtosConfig {
  std::string device{"/dev/i2c-1"};
  double linearScalar{1.0};    // 0.872 to 1.127; true / reported distance
  double angularScalar{1.0};   // 0.872 to 1.127; true / reported angle
  Planar offset{};             // the sensor's pose in the robot frame, m, m, rad
  Duration pollPeriod{std::chrono::milliseconds(10)};   // 100 Hz
  Duration staleAfter{std::chrono::milliseconds(100)};
};

}
```

The defaults are the uncorrected sensor with no offset. The robot fills in
its measured values, as it does `ReceiverConfig::failsafe`: the scalars
measured so far are both 1.0 (reference §5), and the offset depends on
where the sensor is mounted, which is not decided. `start()` checks the
config and fails with `lastError` if a scalar is out of range.

### 2.2 What a reading holds

```cpp
struct OdometrySnapshot {
  bool fresh{false};          // judged at the call: running, and age <= staleAfter
  std::uint64_t seq{0};       // readings so far; 0 before the first
  Duration age{};             // from the reading to the call
  std::uint8_t status{0};     // register 0x1F, raw
  Planar pose{};              // m, m, rad; heading wrapped to +-pi
  Planar velocity{};          // m/s, m/s, rad/s
  Planar acceleration{};      // m/s^2, m/s^2, rad/s^2
  Planar poseStdDev{};        // the board's filter statistic, not measured error
  double totalHeading{0.0};   // rad, unwrapped since start(): counts whole turns

  bool tiltWarning() const;      // status bit 0
  bool trackingWarning() const;  // status bit 1: x and y may be unreliable
  bool sensorError() const;      // status bit 6 or 7: optical chip or IMU
  bool isOk() const { return fresh && !sensorError(); }
};
```

- **`isOk()` is the one test**, as for the remote: fresh and no chip
  error. The two warnings do not fail it. The tilt bit was set throughout
  every spin test and the soak (reference §5, §3) while the heading stayed
  accurate, so failing on it would stop the robot for nothing. What to do
  on a tracking warning (distrust x and y, keep the heading) is the
  robot's decision.
- **`totalHeading`** is unwrapped in the reader thread by adding the short
  way round between reads, as `otos_spin` does (reference §5). At 100 Hz
  this holds below 18 000°/s; the fastest spin measured was 544°/s.
- **The standard deviations** come free with the read (§4), and are
  published for diagnostics. They are filter output, not error bounds
  (reference §1).
- **Freshness is judged at the call**, from the time of the newest
  reading, exactly as the remote judges its link: a dead reader, a failed
  bus and a stopped instance all read as not fresh, with no special case.
  `getLatest(now)` takes an explicit time, for a loop that stamps each
  cycle once.

### 2.3 Health

```cpp
struct OdometryHealth {
  OdometryState state{OdometryState::Stopped};   // Stopped, Running, Failed
  std::uint8_t hardwareMajor{0};                 // 1.1 on ours
  std::uint8_t hardwareMinor{0};
  std::uint8_t firmwareMajor{0};                 // 1.0 on ours
  std::uint8_t firmwareMinor{0};
  Duration calibrationTime{};                    // the last start()
  std::uint64_t readings{0};
  std::uint64_t failedReads{0};
  std::uint64_t tiltSnapshots{0};
  std::uint64_t trackingWarnings{0};
  std::uint8_t statusSeen{0};                    // every status bit seen, ORed
  std::string lastError;
};
```

## 3. Start-up: `start()` blocks, and the robot must be still

`start()` runs the sequence the reference found necessary (§4: without it,
a tilt warning and a 12°/min drift), then launches the thread:

1. Open the bus. Read 0x00–0x02: the product ID must be 0x5F; keep the
   versions for health.
2. **IMU calibration:** write 0x06 = 255; poll every 10 ms until it reads
   0, failing after 2 s. Measured 609–619 ms (reference §4, §5).
3. Write both scalars in one transaction, and read them back. A mismatch
   fails `start()`.
4. Write the offset, all 6 bytes in one transaction; a partial write is
   discarded silently (reference §8). It cannot be read back (§6).
5. Reset tracking: write 0x07 = 1, poll until 0. Measured 11 ms.
6. One read, to prove the snapshot path, then start the reader thread.

**About 0.65 s, and the sensor must be still and level throughout.** That
is a constraint on the robot, not just the library: the robot must start
odometry with its motors off, before it accepts commands. Restarting after
a failure carries the same requirement.

The library never recalibrates by itself. The drift after a calibration
starts near 0.2°/min and grows (reference §8); recalibrating while stopped,
or correcting the heading from the LiDAR, is robot design (§6).

## 4. The reader thread

Each period it reads **37 bytes from 0x1F** in one combined transaction:
status, then pose, velocity, acceleration and their standard deviations.
That is the exact transaction probe O1 soaked: 150 000 reads with the LiDAR
and remote running, no failures, 3.66–6.28 ms each (reference §3). A
shorter read would be faster but is unproven, and the bus carries nothing
else.

- **Period 10 ms (100 Hz),** set by `pollPeriod`, kept by deadline
  (`sleep_until`), not by sleeping after each read: probe O5's
  sleep-then-read loop managed only 65 a second (reference §4). The board updates every
  2.4 ms (reference §7), so faster polling gains little, and the 50 Hz
  control loop then sees a reading at most about 16 ms old. A read takes up
  to about 6 ms, so the bus is busy up to 60% of the time.
- **Timestamp:** steady_clock when the read completes. The data inside is
  up to one board period (2.4 ms) plus the read time older; the robot
  cannot do better without IO9 (reference §7).
- **A failed read** is counted and skipped; the snapshot is not updated, so
  it goes stale after `staleAfter`.
- **One second of consecutive failures** ends the thread with `state =
  Failed`. A sensor that disappears for that long may have lost power, and
  with it its calibration, scalars and offset (reference §4), so carrying
  on would publish uncalibrated data. `start()` can be called again, with
  the robot still.

## 5. Intended use in the robot

A sketch; the robot core does not exist yet.

```cpp
Heron::Odometry::OtosConfig config;
config.offset = kOtosMount;                 // where it is mounted, robot frame
Heron::Odometry::Odometry odometry(config);
if (!odometry.start()) {                    // motors off: ~0.65 s, must be still
  fail(odometry.getHealth().lastError);
}

// each control cycle, e.g. 50 Hz
const auto now = Heron::Odometry::Clock::now();
const auto o = odometry.getLatest(now);
if (!o.isOk()) {
  // no odometry: remote-only driving can continue; autonomy must not
} else {
  useHeading(o.totalHeading);
  if (!o.trackingWarning()) { usePosition(o.pose); }
}
```

What the robot owns, not this library:

- **The mounting offset** and the robot frame's axes.
- **When to recalibrate or reset**, and what to trust during warnings.
- **Fusing with the LiDAR**, and what to do when odometry is not OK.

## 6. Left out of the first version

Each can be added without changing the shape above.

- **`recalibrate()` and `resetPose()` while running.** Needed once the
  robot stops long enough to recalibrate; until then a stop and a
  `start()` does both.
- **Setting the pose** (writing 0x20–0x25), for localisation.
- **Detecting a sensor restart.** Scalars read back 0 after a power cycle
  (reference §4), but only if they were set to something other than 1.0.
- **The signal-processing register** (0x0E), left at its default 0x0F.
- **IO9** as a data-ready line for timestamps (reference §7).
- **A sensor-neutral interface.** `Odometry` is the OTOS reader; "heron_"
  is the project prefix, not a tie to the robot's body. If a second
  odometry source appears (wheel encoders), split out an interface then;
  robot code would not change. Decided 2026-10-19: keep it simple for v1.

## 7. Open questions this design depends on

- **Offset conventions:** which way the sensor's axes and the heading
  point, and whether an offset write takes effect over `/dev/i2c-1` (one
  user saw writes fail silently; reference §6, §10 question 2). Needs a
  probe before the robot relies on the offset.
- **Beyond ±10 m** the position registers may wrap (reference §8). Not
  handled; a robot that roams further needs `setPose` or its own frame.

## 8. Tests

| Test | What it covers |
|---|---|
| `otos_codec_test` | decoding at the scaling edges and signs; scalar encoding, rounding of negative values (reference §5), range checks; offset encoding |
| `otos_device_test` | `OtosDevice` against a fake bus: wrong ID, calibration counting down and timing out, the order of writes, scalars read back wrong, a failed transaction |
| `odometry_test` | `Odometry` with a fake bus: publication, `seq`, freshness and stale, `totalHeading` across ±π, failures counted, `Failed` after one second, stop and restart |

All run under `ctest` on the host and the Pi. `LinuxI2cBus` is only
exercised against the sensor, by `odometry_adapter` and the probes.
