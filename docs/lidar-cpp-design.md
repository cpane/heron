# Using the RPLIDAR SDK from production C++

How to structure the LiDAR code in `cpp/`, and why. This is a design argument,
not a tutorial — it assumes you have read
[`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md), because almost every
recommendation here is downstream of something that document measured.

It deliberately stops at the sensor boundary. How the central robot process
schedules sensors, fuses them and makes decisions is a later document; this one
only has to leave a clean edge for that one to attach to.

> **Provenance.** The architecture is grounded in this repository's own
> measurements. The SDK specifics were **read from source**: Slamtec/rplidar_sdk
> at `99478e5` (2024-04-09), reporting `RPLIDAR_SDK_VERSION "2.0.0"`,
> BSD 2-clause. (The SDK carries two version macros that disagree:
> `rplidar.h` says 2.0.0, while `sl_lidar.h`'s `SL_LIDAR_SDK_VERSION`, which
> the probes print, says 2.1.0. Same commit.) Section 9 records what that
> reading established, including two findings that changed the design.
>
> **Status, 2026-10-04: built.** The design below is implemented as the
> `heron_lidar` library and verified against the Python decoder and live on
> the Pi. Where the build departed from the original plan, the text says so.
> Progress and open items are in `RESUME-lidar.md`.

---

## 1. The short answer

**Wrap it. Do not let SDK types reach robot code.**

Three layers, one rule each:

| Layer | Directory | Rule |
|---|---|---|
| Vendor SDK | `third_party/rplidar_sdk/` | Unmodified, version-pinned |
| Driver / adapter | `cpp/src/lidar/` | The **only** code that includes SDK headers |
| Robot core | everything else | Sees `Heron::Scan`, never `sl::` |

The interesting question is not *whether* to wrap — it is what the wrapper is
allowed to be. A wrapper that mirrors the SDK call-for-call buys you nothing
but indirection and a second place for bugs to live. The wrapper earns its
keep only if it **makes decisions the SDK does not make**, and this sensor
hands you four of them.

---

## 2. Why wrapping is not optional here

Not on general principle. Four specific measured facts.

### 2.1 The coordinate frame is left-handed

The single strongest argument, and it is not stylistic.

0° points out the front of the housing and **the bearing increases clockwise**
viewed from above. REP 103 — x forward, y left, z up — is anticlockwise-
positive. So the conversion is not an offset, it is a *negation*:

```
bearing_robot = -bearing_sensor + mounting_offset
```

Get the sign wrong and every obstacle mirrors across the forward axis. There
is no compile error. There is no exception. The robot simply steers toward
things instead of away from them, and it does it consistently enough to look
like a tuning problem rather than a sign error.

A transform like that must happen **exactly once, in exactly one place**. If
`sl_lidar_response_measurement_node_hq_t` is visible to path planning, then
somewhere in the next year a second consumer will read `.angle_z_q14` directly
and either skip the transform or apply it twice. The wrapper exists so that
"is this bearing in robot frame?" has one answer: *if it is a `Heron::Point`,
yes, always.*

### 2.2 Most returns are not returns

61–84% of samples are invalid (`dist_q2 == 0`) in the recorded scenes, and
fewer in a closer one. That is the common case, not an edge case. And `quality == 0` means exactly "no return"
— measured, zero valid samples out of 15,619. That is a legacy-mode fact; in
express the SDK *derives* quality from distance, so the adapter's test for a
no-return is `dist == 0`, in every mode.

So somebody has to decide what an invalid return *is* to a consumer. Leak the
raw array and every consumer re-implements that filter, slightly differently,
and one of them forgets. Decide once at the boundary: **drop invalid returns,
keep the count.**

Keeping the count matters more than it looks. "Nothing at 90°" and "did not
look at 90°" are different facts, and the difference is the one that gets a
robot into trouble — an unobserved sector is not a clear sector.

### 2.3 Buffer overrun is completely silent

Measured: ~128 consecutive samples vanished with no error, no status flag, and
byte alignment intact afterwards. Nothing in the SDK will tell you this
happened, because nothing on the wire says it happened.

The only evidence is geometric: an angular gap much wider than the expected
spacing. So the wrapper runs a gap detector and publishes
`droppedEstimate`.

**Measured 2026-10-04 in express mode: a gap is not always the symptom.** A
small loss reached the SDK's output as a 45° gap; larger ones made the SDK
publish two revolutions merged into one scan (827 and 929 samples against a
normal ~530), which has no gap at all. So the adapter needs **two checks**:
the angular gap, and the sample count against the count expected for one
revolution. See the findings, protocol trap 10. That is not a convenience — it is the only integrity signal
this sensor's data stream has, and it does not exist unless you build it.

### 2.4 A revolution is not an instant

~137 ms start to finish at ~7.3 Hz. Once the robot moves, the points in one
revolution come from different poses. Treating a revolution as a single
timestamp smears the map.

So the boundary type carries `t_start` and `t_end`, not a timestamp. Whether
you deskew on day one is a separate decision — but if the *type* cannot express
the span, you have made that decision permanently and by accident.

The span alone is not enough, though. Sorting by bearing discards time order,
the sweep starts wherever the sync flag fires (351–359.5° sensor frame), and
time runs toward *decreasing* robot-frame bearing because the sensor turns
clockwise. So the type also carries `sweepStartRad`, read off the sync
sample before `ascendScanData()` sorts it away, and `timeOf(bearing)` turns
the three into a per-point time.

Both times are estimated *measurement* times. `t_start` is the SDK's scan
timestamp — the sync sample's arrival back-dated by the mode's per-sample
delay (`sl_lidar_driver.cpp`, `pushScanNodeData()`). `t_end` is when the beam
returns to `sweepStartRad`, a full turn; the SDK gives no end time, so the
adapter estimates it. The Python server's `t_start_ns`/`t_end_ns` are host
*arrival* times, so replay diffs against it are offset by the sensor latency.
The SDK's `CLOCK_MONOTONIC` is `steady_clock`'s clock on the Pi build —
measured 2026-10-01 from `lidar_record` captures, see §9 row 4.

### 2.5 The honest counter-argument

Wrapping costs something. A thin wrapper that only forwards calls is pure
liability: more code, an extra hop when debugging, and a strong tendency to
grow `getHealth()`, `getFirmware()`, `setMotorSpeed()` passthroughs until it is
a worse copy of the SDK.

The discipline that prevents that: **the wrapper's public surface is defined by
what the robot needs, never by what the SDK offers.** If robot code never asks
for the firmware version, the wrapper does not expose it. Diagnostics that need
deeper access belong in `cpp/probes/`, which is allowed to include SDK headers
directly and is explicitly not production.

---

## 3. Keep using the SDK

The decision in `RESUME-lidar.md` stands, and the strongest reason is worth
restating: **this unit's recommended mode is one you cannot decode yourself
cheaply.**

| Mode | Streams | Hand-rolled? |
|---|---|---|
| 0 Standard | legacy 0x81 | already done in Python |
| 1 Express | express 0x82 | already done in Python |
| 2 Boost | ultra 0x84 | days of work |
| 3 Sensitivity ★ | ultra 0x84 | days of work |
| 4 Stability | ultra 0x84 | days of work |

★ is what `SCAN_MODE_TYPICAL` reports. The ultra format uses a
varbit/predictive encoding, and its failure mode is not a crash — it is
plausible-looking wrong distances. That is the worst possible bug class for a
sensor, and it is exactly the class this project has already been bitten by
twice.

**Default scan mode: 1, Express** (decided 2026-10-04). Its decode is proven:
SDK output matches the Python decoder node for node (§8). Mode 3 stays
selectable through `LidarConfig::scanMode` once something independent has
checked the ultra decode. The SDK is still the right choice with Express as the
default: it keeps mode 3 one config change away, and it supplies the per-sample
timestamps and the angle sort either way. Note that `-1` ("ask the device") would
select mode 3 on this unit, which is why the default is explicit.

Three things the API gives you directly (all confirmed in source):

- `startScanExpress(force, scanMode, ...)` takes the scan-mode id, so mode
  selection is a parameter, not a protocol exercise. `getAllSupportedScanModes()`
  and `getTypicalScanMode()` enumerate them.
- `grabScanDataHqWithTimeStamp(nodes, count, timestamp_uS, timeout)` exists
  alongside the plain variant — a device-side timestamp in microseconds, which
  is the real input to deskewing and which the Python work never implemented.
- `ascendScanData()` sorts by angle, which this sensor needs because ~12% of
  samples arrive behind the previous angle. **Read §6 before calling it** — it
  does something to invalid samples that you must account for.

The SDK's own header documents the ordering problem in the same terms this
project measured it: *"the angle data in one scan may not be ascending."*

Hand-rolling legacy+express only remains legitimate — `docs/lidar-references.md`
indexes the whole spec — but it costs 2× angular resolution to remove one
dependency. Not recommended.

---

## 4. The boundary type

As built in `cpp/include/heron/lidar/Scan.h` (abridged), with the reasoning attached:

```cpp
// cpp/include/heron/lidar/Scan.h -- abridged; the header is the authority
#ifndef HERON_LIDAR_SCAN_H
#define HERON_LIDAR_SCAN_H

namespace Heron {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

float wrapBearing(float rad);  // -> [-pi, +pi); the only wrap anyone uses

struct Point {
  float bearingRad{0.0f};  // ROBOT frame, [-pi, +pi). Sensor frame is
                           // left-handed; the negation happened once.
  float rangeM{0.0f};      // > 0: no-returns never reach this vector.
};                         // No quality -- see below.

struct Scan {
  std::uint64_t seq{0};   // monotonic; gaps mean the consumer fell behind
  TimePoint tStart{};     // ~137 ms apart. A revolution is not an instant,
  TimePoint tEnd{};       // and deskewing needs the span, not a moment.
  float sweepStartRad{0.0f};  // where the sweep began, robot frame

  std::vector<Point> points;  // ascending bearing, invalid returns removed

  std::uint32_t rawSampleCount{0};   // before dropping invalids
  std::uint32_t droppedEstimate{0};  // from the angular-gap detector
  float worstSamplingGapRad{0.0f};   // over ALL samples, no-returns too
  std::vector<Arc> unobserved;       // sampling gaps wider than 10 deg

  // "Did I actually look all the way round?" Not the same question as
  // "how many points are there?", and the one that matters for safety.
  bool isCoverageOk(float maxGapRad = kUnobservedGapRad) const;
  bool isObserved(float bearingRad) const;
  TimePoint timeOf(float bearingRad) const;  // per-point measurement time
};

}  // namespace Heron

#endif  // HERON_LIDAR_SCAN_H
```

Names, layout and documentation follow
[`CPP_CODING_STANDARD.md`](../CPP_CODING_STANDARD.md). The aggregates'
public data members are camelCase without `m_`, following the standard's own
aggregate example; `m_` is for the private members of classes.

The reasoning behind the choices, settled in two reviews (2026-10-01):

- **`seq` as well as timestamps.** Timestamps tell you *when*; `seq` tells you
  whether you missed one. A consumer that processes every third scan because
  it is too slow should be able to know that.
- **No `quality` — settled 2026-10-01.** The strawman kept it as a "0–15
  reflectivity proxy". Checked against the captures and the SDK, it means
  something different in every mode: legacy is 6 bits, 90–97% of valid
  returns saturated at 15, and the SDK multiplies it by 4; **express has no
  quality field at all** and the SDK substitutes the constant 188
  (`handler_capsules.cpp`); ultra is real but its resolution changes with
  distance. Reinstate it when a consumer needs it *and* someone has measured
  what it means in the mode in use.
- **No scan mode on `Scan` — settled 2026-10-01.** Quality was the main thing
  a consumer could misread without the mode. Expected density is a
  diagnostic, for a health/status channel.
- **`float` not `double`.** Range is ≤12 m at ¼ mm native resolution; float has
  ~7 significant digits. Ample, and `Point` is 8 bytes — about 4 KB for a
  530-point revolution.
- **The half-open bearing range needs one wrap function.** Measured over
  ~1.9e9 inputs: `std::remainder`, the obvious library call, returns exactly
  +π for +π; the `fmod` form, `atan2(sin, cos)` and while-loops did not
  break the range. `wrapBearing()` is the one implementation.
- **Mind the units at the seam.** The wire format is Q6 degrees, but the SDK's
  `sl_lidar_response_measurement_node_hq_t` renormalises to **Q14**:

  ```c
  sl_u16 angle_z_q14;   // degrees = angle_z_q14 * 90.f / 16384.f
  sl_u32 dist_mm_q2;    // mm      = dist_mm_q2 / 4.f
  sl_u8  quality;
  sl_u8  flag;          // bit 0 = start of scan
  ```

  So the Python library's `angle_q6` and the SDK's `angle_z_q14` are *not* the
  same number. Anything cross-checking C++ output against a `.jsonl` capture
  has to convert, and a Q6/Q14 mix-up is a factor-of-256 error that looks like
  a wildly wrong bearing rather than a subtle one — which, for once, is the
  merciful failure mode.
- **No `std::optional<Pose>` yet.** Deskewing needs pose history, which is the
  *robot's* knowledge, not the sensor's. The sensor publishes when it saw
  things; the core decides where the robot was. Keep that split.

---

## 5. Ownership, threading and lifetime

### 5.1 One thread, owned by the driver

**The SDK already spawns two threads of its own.** `AsyncTransceiver` starts
`_rxThread` and `_decoderThread` on connect; they read the channel and decode
frames in the background. (`_cachethread` is declared in
`sl_lidar_driver_impl.h` but never referenced in `sdk/src/` — vestigial from
the older design.)

`grabScanDataHq()` blocks the *calling* thread until a complete 360° scan is
available or the timeout expires; passing `timeout = 0` makes it return
immediately, giving you a non-blocking poll if you prefer.

So your adapter thread is a third thread, and its job is to block in
`grabScanDataHq()` and then: gap detect over every sample → drop invalids →
transform → sort → publish. As built (2026-10-04) it sorts its own robot-frame
copy and never calls `ascendScanData()`, which removes §6's ordering trap
rather than working around it. It also applies two timing rules before
publishing (§6): merged revolutions and post-stall backlog are dropped. That is still the right structure — something has to own the blocking
call — but budget three threads per LiDAR, not one, when you size the robot
process.

```cpp
// cpp/include/heron/lidar/Lidar.h -- built 2026-10-04; abridged
namespace Heron::Lidar {

struct LidarConfig {
  std::string port{"/dev/rplidar"};  // never ttyUSB0
  std::uint32_t baud{115200};
  std::int32_t scanMode{1};          // Express by default; see §3
  float mountingOffsetRad{0.0f};
  std::int32_t discardRevolutions{3};  // spin-up transient
  std::string replayFile;            // testing: a lidar_record .rpraw instead of the port
};

class Lidar {
  public:
    explicit Lidar(const LidarConfig &config);
    ~Lidar();  // stops the motor. Always.

    Lidar(const Lidar &) = delete;
    Lidar &operator=(const Lidar &) = delete;

    bool start();
    void stop();

    // Latest complete revolution, or nullptr if none yet. Cheap: a
    // mutex-guarded shared_ptr copy, no copy of the point vector.
    std::shared_ptr<const Heron::Scan> getLatest() const;

    LidarHealth getHealth() const;  // state and counters; a first version

  private:
    struct Impl;  // SDK types live here and nowhere else
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Heron::Lidar
```

The `Impl` pimpl is what keeps `sl_lidar_driver.h` out of every translation
unit that wants a distance reading. It is also what stops the SDK's include
paths and macros leaking into the rest of the build.

### 5.2 Latest-wins, not a queue

For a control loop, a queue of scans is the wrong shape. If the consumer falls
behind, you do not want a backlog of stale geometry — you want the newest one
and a way to know you skipped some. Hence `getLatest()` plus `seq`.

A queue becomes right later, if something needs *every* scan (a mapper
accumulating a point cloud). Add that as a second, explicit subscription then;
do not make the common path pay for it now.

### 5.3 RAII is not enough on its own

The destructor stops the motor. That covers normal exit and exceptions. It does
**not** cover `SIGINT`, `SIGTERM`, or an abort — and a half-second of Ctrl-C
leaving the head spinning is how this project ended up chasing an under-voltage
problem that was really a power supply problem.

So: destructor **and** a signal handler that stops the motor, as `RESUME-lidar.md`
§4 already requires. In a multi-sensor process this is an argument for a single
shutdown path that every sensor registers with, rather than each sensor
installing its own handler — worth settling in the robot-core design.

---

## 6. Failure modes to design for

Not hypothetical. These are the ones this sensor demonstrably has.

| Failure | Evidence | Design response |
|---|---|---|
| Silent buffer overrun | measured, ~128 samples gone | angular-gap detector **and** a sample-count check → `droppedEstimate` |
| Overrun merges two revolutions | measured 2026-10-04: 827- and 929-sample scans, no gap | sample-count check; such a scan must not be published as covered (open: drop it, or flag it) |
| SDK frees queued buffers with the wrong `delete` | measured abort in `disconnect()` | one-line local patch at build time, see `cpp-build.md` |
| Post-stall backlog stamped on arrival | measured 2026-10-04: intervals of 6 ms, even −4 ms, against 137 ms | after an interval over 1.5x the period, drop scans until one arrives at a normal interval |
| Stream not angle-sorted | ~12% arrive backwards | `ascendScanData()` or your own sort — never assume order |
| Invalid returns dominate | 61–84% | drop at the boundary, keep the count |
| Spin-up transient | probes discard 3 revolutions | `discard_revolutions` in config |
| USB re-enumeration | **open question** in findings | reconnect state machine; do not assume the fd stays valid |
| Health states 1 and 2 | **never observed** | cannot be tested here. Log loudly, fail safe, do not pretend you handled it |
| `ascendScanData()` fabricates angles | **read from SDK source** | run the gap detector on *every* sample, *before* `ascendScanData()` — see below |

### The `ascendScanData()` trap

This one is specific, verified, and would have silently defeated §2.3.

`ascendScanData()` does not drop invalid samples. Before sorting, it **invents
an angle for every one of them**, assuming uniform angular spacing:

```c
// sdk/src/sl_lidar_driver.cpp, ascendScanData_()
float inc_origin_angle = 360.f / count;
...
// Fill invalid angle in the scan
float expect_angle = frontAngle + i * inc_origin_angle;
```

Two problems, both measured in this repository:

1. **Spacing is not uniform on this sensor.** Legacy: median 1.40°, p95 3.3°,
   p99 5.5°, worst gap 8–9° per revolution. The `360/count` assumption is
   simply false here, so the synthesised angles are wrong by degrees, not by
   rounding.
2. **It masks the silent buffer overrun.** When ~128 samples vanish, the
   survivors leave a wide angular hole — that hole is the *only* evidence the
   loss happened. The lost samples never arrived, so they occupy no slots;
   but the *no-return* samples that did arrive have their real wire angles
   **overwritten in place** with `frontAngle + i * 360/count`. Index `i` keeps
   counting straight across the hole, so those guesses land inside it. Run
   the gap detector over the ascended buffer and the hole is paved over.
   (An earlier version of this section said the lost samples sit in the
   buffer with `dist == 0`. They don't; the mechanism is the overwrite.)

### And the obvious fix is also wrong

An earlier version of this document said: filter `dist == 0` first, then
detect gaps on the surviving valid points. That avoids the fabricated angles
but measures the wrong thing. A no-return still proves the beam looked
there; only a sample that never arrived leaves a hole. With 61–84% of samples
returning nothing, gaps between *valid* points are 30–115° on perfectly
healthy revolutions. Measured 2026-10-01 across all 11 captures (981
revolutions): the 8° coverage threshold failed **every one** on valid
points, and on all samples every one read 8.0–8.9°.

So the gap detector runs over **every** sample, with its **real** wire angle,
which means before `ascendScanData()` gets to it:

```
grabScanDataHq()  ->  gap detect  ->  ascendScanData()  ->  DROP dist==0  ->  transform
                      ^^^^^^^^^^
                      all samples, real angles, before anything rewrites them
```

`grabScanDataHq()` returns nodes in arrival order, which is not angular
order, so the detector sorts a copy of the angles itself. The Python
reference is `Revolution.sampling_gap_q6()` in `python/heron/lidar/scan.py`:
integer Q6, sorted, closing the 360° seam. Same revolution in, same number
out.

**Unverified:** that the angles `grabScanDataHq()` gives no-return samples
are real ones in express and ultra modes. There they are interpolated from
capsule start angles anyway — see §9, "Still open".

This is exactly the class of bug this project keeps meeting: no crash, no error
code, just plausible-looking data that quietly asserts the sensor saw more than
it did.

That last row deserves its own note. You cannot produce a degraded-health
sensor on demand, so that code path will ship untested. Write it so that the
untested branch does the *safest* thing — stop and report — rather than the
cleverest one.

### The pattern that matters most

**Never let a bad scan look like an empty room.** A scan with 400 dropped
samples and a scan of an empty corridor can both arrive with few points. If
the consumer cannot distinguish them, the failure mode is a robot confidently
driving into something it did not look at. `isCoverageOk()` exists for this, and
whatever consumes `Scan` should be required to check it.

---

## 7. Build integration

As built (details and commands in [`cpp-build.md`](cpp-build.md)):

```
CMakeLists.txt, CMakePresets.json   host and pi configurations
cpp/
  include/heron/lidar/      Lidar.h, Scan.h, SectorCheck.h     (no SDK headers)
  src/lidar/                Lidar.cpp, ScanBuilder.*, Rpraw.*, TapChannel.*, ReplayChannel.*
  probes/                   on-target programs; MAY include SDK headers
  tools/                    lidar_gui_server
  tests/                    unit tests (ctest) and recording checks
third_party/
  rplidar_sdk/              pinned submodule at 99478e5, unmodified
  CMakeLists.txt            our build of it, outside the submodule
```

Decisions made along the way, each recorded where it was settled:

- **The SDK's sources compile into our own CMake target**, not its Makefile:
  one build graph, one set of flags, one toolchain. Its headers are `SYSTEM`
  and its warnings are off, so third-party noise never tempts us into
  lowering our own warning level.
- **Pinned as a git submodule**, with our build description outside it, so an
  update is a checkout and nothing else.
- **One upstream bug is patched at build time**, from a copy, leaving the
  submodule untouched; configure fails if the patched line ever changes.
- **Built on the host, in a Debian trixie arm64 container**, never on the Pi,
  as the development model requires. The Pi 3B's 905 MiB and throttling cores
  made on-target builds the wrong choice anyway.

---

## 8. Testing — this is where the Python work pays off

The most valuable asset this repository has for the C++ work is not the Python
driver. It is the **`.rpraw` capture format and its `--verify` acceptance
test**:

```
python3 python/tools/lidar_rawcat.py captures/<name>.rpraw --verify
```

Byte-exact wire recordings, plus a tool that re-decodes them and checks the
result reproduces the recorded output exactly. That is a golden-file test suite
for a decoder — and it works for validating *the SDK* just as well as your own
code.

### The seam that unlocks it — confirmed

`IChannel` is a pure abstract interface in the **public** header
`sl_lidar_driver.h`, and `connect()` takes an `IChannel*`. Nine methods, all
trivial over a byte buffer:

```cpp
bool      open();
void      close();
void      flush();
bool      waitForData(size_t size, sl_u32 timeoutInMs, size_t* actualReady);
sl_result waitForDataExt(size_t& size_hint, sl_u32 timeoutInMs);
int       write(const void* data, size_t size);
int       read(void* buffer, size_t size);
void      clearReadCache();
int       getChannelType();
```

An earlier version of this section proposed a channel that "serves recorded
bytes and swallows writes", fed from the existing Python captures. **That
cannot work**, read from source: the SDK is not a passive reader.
`connect()` sends `GET_DEVICE_INFO` and waits for the answer
(`sl_lidar_driver.cpp`, `checkMotorCtrlSupport()`), and `startScanExpress()`
sends four `GET_LIDAR_CONF` queries before the scan command, failing if any
goes unanswered. The Python captures hold only `SCAN`/`EXPRESS_SCAN` and
`STOP`, so those answers do not exist as bytes anywhere.

What was built instead (2026-10-01) — **record through the SDK, replay in
lockstep**:

| Piece | Where | What it does |
|---|---|---|
| `.rpraw` reader/writer | `cpp/src/lidar/Rpraw.*` | the Python format, byte for byte; no SDK |
| `TapChannel` | `cpp/src/lidar/TapChannel.*` | wraps the real serial channel; records every `write()` and non-empty `read()` as one chunk; forwards `setDTR` |
| `ReplayChannel` | `cpp/src/lidar/ReplayChannel.*` | serves a received chunk only once every write recorded before it has been matched **byte for byte** by the SDK; reports divergence otherwise; paces at recorded speed |
| `lidar_record` | `cpp/probes/` | one session function for live and replay, so the SDK makes identical calls |
| `lidar_sdkcheck.py` | `python/tools/` | live vs replay, and SDK vs Python decode of the same bytes |

Measured on the Pi and replayed on the host, 40 revolutions per mode:

| Mode | Writes matched | Replay = live | SDK = Python |
|---|---|---|---|
| 0 Standard (legacy, 0x81) | 17 of 17 | 40/40 identical | 40/40 identical, node for node |
| 1 Express (0x82) | 11 of 11 | 40/40 | 40/40 |
| 3 Sensitivity (ultra, 0x84) | 11 of 11 | 40/40 | — no Python ultra decoder |

Each replay was repeated three times; all nine matched the live run exactly.
A one-node, 1 mm change to a dump fails both checks and is located to the
node.

Why pacing defaults to real time rather than "as fast as possible": the SDK
publishes scans latest-wins from a double buffer, so a stream served faster
than the consumer grabs it silently drops whole revolutions.

**The ultra row is replay fidelity only.** It proves the SDK reproduces its
own ultra output from the recording, not that the output is right: nothing
independent decodes ultra capsules. The capture
`desk-sdk3_20261002T032343Z.rpraw` is the first ultra fixture this
repository has.

The Python captures remain usable for the Python decoder, but cannot be
replayed through the SDK. A device emulator that answers the queries itself
would change that; deliberately not built, since an emulator can drift from
the real device without anyone noticing.

**One gotcha, and it is a real one.** For a device reporting
`MotorCtrlSupportNone` — which the A1 is, since it uses DTR rather than PWM —
`setMotorSpeed()` does this:

```cpp
if (_transeiver->getBindedChannel()->getChannelType() == CHANNEL_TYPE_SERIALPORT) {
    ISerialPortChannel* serialChanel = (ISerialPortChannel*)_transeiver->getBindedChannel();
    ...  serialChanel->setDTR(...);
```

An unchecked downcast gated only on `getChannelType()`. So a replay channel
must either derive from `ISerialPortChannel` (implementing `setDTR` as a
no-op) **or** report a channel type that is not `CHANNEL_TYPE_SERIALPORT`.
Return `CHANNEL_TYPE_SERIALPORT` from a plain `IChannel` subclass and that
cast is undefined behaviour.

Reporting a non-serial type is the cleaner choice for replay: motor control is
meaningless against a recording anyway. `ReplayChannel` reports
`CHANNEL_TYPE_TCP`; the only other branch on the type is baud-rate detection,
which a replay never runs. `TapChannel` *is* an `ISerialPortChannel`, so motor
control works through it live.

### What to test at each layer

| Layer | How | Where, as built |
|---|---|---|
| Decode | `.rpraw` replay, diffed against the Python decoder | `lidar_record --replay` + `lidar_sdkcheck.py` |
| Transform | Pure function, table-driven. Feed known bearings, assert robot-frame results. The left-handedness makes this the highest-value unit test in the system | `ScanBuilderTest` |
| Gap detector | Synthetic sample lists with deliberate holes, *including no-return samples* — a fixture of valid points only cannot catch the mistake §6 describes. Then `.rpraw` replay against `Revolution.sampling_gap_q6()`: healthy captures must read 8–9°, never over 10° | `ScanBuilderTest`; `scan_builder_check` + `lidar_gapcheck.py` |
| Overrun detection | Replay the starved-reader recordings (`starve-stall-sdk1_*`, `starve-stall2-sdk1_*`): every loss `lidar_overrun.py` finds on the wire must be flagged, by gap or by sample count. They replay with the same anomalies | `lidar_overrun.py`; `lidar_replay_check` |
| Whole adapter | Every published scan matches the Python decoder point for point | `lidar_replay_check` + `lidar_adaptercheck.py` |
| Consumer rules | Freshness, coverage, sector observed, nearest | `SectorCheckTest` |
| Lifecycle | Start/stop/restart against hardware, in `cpp/probes/` | start, stop, Ctrl-C and a stall verified with `lidar_adapter`; restart and unplug **not yet** |

Keep a few fixtures: an open room, a close wall, a cluttered scene. One
ultra-capsule fixture exists, `desk-sdk3_20261002T032343Z` (git-ignored, so
local to the host and the Pi). Keep it: mode 3 is the mode whose decode you most
need to trust before enabling it, and the least able to verify by eye.

---

## 9. What reading the SDK established

All answered against `99478e5`, SDK 2.0.0. Two of these changed the design.

| # | Question | Answer |
|---|---|---|
| 1 | Custom channel for `.rpraw` replay? | **Yes.** `IChannel` is public and pure virtual; `connect()` takes `IChannel*`. Mind the `ISerialPortChannel` downcast — §8 |
| 2 | What does the grab call return? | `grabScanDataHq(nodes, count, timeout)` → one complete 360° scan, `nodes[0]` has `start_bit == 1`. Blocks until ready or timeout; `timeout = 0` polls |
| 3 | Does `ascendScanData()` sort in place, and drop invalids? | Sorts in place with `std::sort`. **Does not drop invalids — it fabricates their angles.** See §6. Returns `OPERATION_FAIL` if every sample is invalid |
| 4 | Per-sample timestamps? | `grabScanDataHqWithTimeStamp()` gives a timestamp in µs per scan: the sync sample's arrival on `CLOCK_MONOTONIC`, back-dated. Per-*sample* back-dating happens inside the unpacker, not on the public API. **Measured 2026-10-01:** on the Pi build this is the same clock as `steady_clock` — same epoch, and every scan stamp lands 0.3–17 ms before the received chunk that carried it. **On macOS it is not:** `steady_clock` is `CLOCK_MONOTONIC_RAW`, 2.9 s from the SDK's `CLOCK_MONOTONIC` (measured 2026-10-04), so the adapter converts stamps by age. After a reader stall the stamps are arrival times, not measurement times (§6) |
| 5 | Threading model | **Two SDK threads** — `_rxThread`, `_decoderThread`, from `AsyncTransceiver` on connect. Not opt-out. `_cachethread` is vestigial |
| 6 | Error taxonomy | `sl_result` codes, no exceptions. `SL_RESULT_OK`, `_OPERATION_TIMEOUT`, `_OPERATION_FAIL`, `_INVALID_DATA`, `_OPERATION_NOT_SUPPORT`, … failure marked by `SL_RESULT_FAIL_BIT` (0x80000000) |
| 7 | USB re-enumeration | Not addressed by the SDK. `connect()`/`disconnect()`/`isConnected()` are the only tools; the reconnect state machine is yours to write |
| 8 | License | **BSD 2-clause.** Vendoring is fine; retain the copyright notice |

Two more worth knowing:

- **Motor control on the A1 goes through DTR**, via `setMotorSpeed()` →
  `ISerialPortChannel::setDTR()`. Note the polarity: speed 0 sets DTR *true*,
  which is motor off — the same inversion `python/heron/lidar/transport.py`
  already handles.
- **`getFrequency(scanMode, nodes, count, frequency)`** computes the rotation
  rate from a scan, so you do not need to time revolutions yourself to sanity
  check against the measured ~7.3 Hz.

### Answered 2026-10-04: does an overrun hole survive the SDK?

**Not reliably.** Starved-reader recordings in express mode (findings, trap
10) show a lost capsule can reach `grabScanDataHq()` as a visible gap (45°),
or, after a longer stall, as two revolutions merged into one over-full scan
with no gap at all. The adapter therefore checks the sample count as well as
the gap (§2.3, §6).

Still open from the same recordings:

- ~~**What the adapter publishes for a merged revolution.**~~ Decided
  2026-10-04: it is **dropped**, and so is the backlog drained after a stall,
  whose SDK stamps are arrival times. Both advance `seq` and are counted in
  `LidarHealth` (§6).
- **The SDK disagrees with itself at a discontinuity.** The two scans touching
  each overrun differ between live and replay, and between the SDK and the
  Python decoder; every other scan matches exactly. Cause not established.

## 10. How this fits the multi-sensor robot

The LiDAR is the first sensor, which makes it the one that accidentally sets
the pattern for the Arduino motor controller, the I²C odometry and the
GPIO/UART remote. Worth being deliberate about which parts are LiDAR-specific
and which are the pattern.

### Make uniform

- **One clock.** `steady_clock` everywhere, captured as close to the hardware
  event as possible. Sensors that disagree about time cannot be fused, and this
  is very hard to retrofit.
- **Lifecycle.** Every sensor: construct → `start()` → running → `stop()`,
  with the destructor safe at any point, and one process-wide shutdown path.
- **Health reporting.** A common shape for "I am fine / degraded / gone", so
  the core can reason about sensors it does not understand in detail.
- **Reconnection as normal.** USB and I²C devices disappear. A sensor that
  cannot come back without a process restart will define your uptime.

### Let differ

- **Payload types.** `Scan` has nothing useful in common with a wheel-tick
  count. Do not invent a `SensorReading` base class to force them together;
  you will spend the next year casting out of it.
- **Rates.** ~7.3 Hz here, likely kHz for odometry. A single polling rate for
  all sensors is a mistake that is easy to make early.

### Do not build yet

Resist a generic sensor framework until there are at least two real sensors
running. Right now you have one, and a framework designed around one example
is just that example with extra indirection. The three uniform things above are
enough to keep options open; everything else can wait for the second sensor to
tell you what is actually common.

---

## 11. Suggested order

1. ~~**Settle the `Scan` type**~~ done (§4). It is the contract; everything else is an
   implementation detail behind it. Argue about it now, cheaply.
2. ~~**Decide host cross-compile vs on-target build**~~ done (§7). Shapes the build
   system, so it comes before the build system.
3. ~~**Vendor and pin the SDK, get it compiling**~~ done in a CMake target. No wrapper
   yet — just prove it builds and links for your chosen target.
4. ~~**Write the replay channel**~~ done (§8). Confirmed feasible, ~an afternoon, and
   it is the difference between a decoder you trust and one you hope about.
   Do it before the adapter, so the adapter is testable from its first commit.
5. ~~**Build the adapter**~~ done 2026-10-04: thread, **gap detect over every sample, then** sort,
   filter, transform, publish. That order is load-bearing — see §6.
6. **Next: one boring end-to-end behaviour** — "nearest obstacle in a forward 60°
   arc", per `RESUME-lidar.md` §5. Proves the whole chain before SLAM or costmaps
   make failures ambiguous.

The transform unit test in step 5 is the one to write first and keep forever.
A sign error there is invisible in code review, invisible at compile time, and
perfectly disguised as a tuning problem in the field.

---

## Related

- [`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md) — the measurements every argument here rests on
- [`lidar-references.md`](lidar-references.md) — protocol spec by page, and which SDK file settles which question
- [`RESUME-lidar.md`](RESUME-lidar.md) §3–§5 — the prior decisions this document expands
- [`lidar-gui.md`](lidar-gui.md) — the live viewer, useful for sanity-checking C++ output against something visual
