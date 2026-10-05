# LiDAR workstream — resume point

Last updated 2026-10-04. Branch **`develop`**, everything pushed
(`feature/pi-provisioning` was merged to `main` via PR #1).

## Read this first when resuming

**Paused 2026-10-04** to work on something else. The LiDAR work is at a clean
stopping point: a reusable C++ library, a test program and a GUI backend, all
verified on recordings and live on the Pi.

**Scope (decided 2026-10-04): a reusable LiDAR, not a robot.** Odometry and
the remote control have not been explored yet, so robot-level design (motion
arbiter, motor link, shutdown path, deskewing, mounting offset) waits for them.

What exists, and where it is written up:

| Piece | Run it | Section |
|---|---|---|
| `Lidar` adapter (`Lidar.h`, `Scan.h`, `SectorCheck.h`) | linked as `heron_lidar` | 6 |
| `lidar_nearest`: nearest obstacle in a sector | `scripts/deploy_cpp.sh cpane@<pi> lidar_nearest --arc 90` | 7 |
| `lidar_gui_server` + the viewer | `scripts/deploy_cpp.sh cpane@<pi> lidar_gui_server`, then `.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect` | 8, `lidar-gui.md` |
| `lidar_adapter`: live health, once a second | `scripts/deploy_cpp.sh cpane@<pi> lidar_adapter 20` | 6 |
| Record a session / replay it | `lidar_record --mode 1 --scans 40 --tag x` on the Pi; `--replay` on the host | 4b |

Build first with `scripts/build_cpp.sh` (Pi) or `cmake --build --preset host`.
Recordings are git-ignored and live in `captures/` on this host and the Pi.

**Picking up: lifecycle hardening** (section 9): restart the same `Lidar`
repeatedly; unplug and replug with reconnection (needs someone at the Pi to
pull the cable); measure and shorten the ~1 s `stop()`. Then mode 3, once
something independent can check its ultra decode.

History, in order: the 0-degree question (1), the live viewer (2), the C++
build chain (3), the `Scan` type (4), SDK record and replay (4b),
starved-reader recordings (4c), the adapter (6), the test program (7), the
GUI backend (8).

---

## 1. ANSWERED — where is 0 degrees?

**0 degrees points out the front of the housing, along its symmetry axis, and
the bearing increases clockwise viewed from above.**

Measured 2026-09-26, target ~30 cm out on the symmetry axis:

| Position | Bearing | Distance |
|---|---|---|
| Rear face | 175.3 deg (range 174.1-176.6) | 309 mm |
| 90 deg clockwise from rear | 262.3 deg (range 259.6-263.8) | 337 mm |

Delta +87.0 deg for a nominal +90 deg move. Anticlockwise would have given
~85 deg, so the direction is not a close call. The 3 deg shortfall is hand
placement, as is the 28 mm change in radius.

**The consequence that matters:** clockwise-positive makes the sensor frame
left-handed, while REP 103 (x forward, y left, z up) is anticlockwise-positive.
The transform must negate, not merely offset:

```
bearing_robot = -bearing_sensor + mounting_offset
```

Wrong sign mirrors every obstacle across the forward axis, with no compile
error.

Two things to carry forward rather than treat as settled:

- The 175.3 deg is the bearing to the *nearest return* on the target surface,
  not to the target's centre. The direction is exact; the offset is good to a
  few degrees. Re-derive it from mounting geometry once the sensor is bolted
  down.
- Earlier notes suggested using "the cable exit" as the landmark. There isn't
  a usable one on this unit. The landmark is the housing's **symmetry plane**,
  which is also what spec figure 4-6 (protocol PDF p.17) references.

## 2. DONE — the live viewer

Built and **verified against the sensor 2026-09-27**. Two terminals:

```
scripts/deploy_pi.sh cpane@<pi> server/lidar_server.py
.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
```

Full usage in `docs/lidar-gui.md`. What it is:

| | |
|---|---|
| `python/server/lidar_server.py` | Pi-side, owns the sensor, one client |
| `python/tools/lidar_gui.py` | host-side Tkinter polar view |
| `python/heron/link/wire.py` | newline-JSON over TCP, shared by both |

Motor, scan-mode selection, scan toggle, and a range filter that both drops
distant returns and rescales the plot so the kept region fills the canvas.

**What is verified:** the wire format, the GUI's polar transform and range
filter, mode handling, and the server's accept loop, single-client lock,
command queue and teardown — 28 checks, all against a *stub* sensor on the
host.

**Since exercised on hardware (2026-10-01):** `describe()`, the mode dispatch
in `start_stream()` for all five modes, both point-streaming loops and the
raw-statistics loop, driven by a scripted client rather than the GUI.

### Hardware run 2026-10-01

**The gap fix (section 4) holds live.** Over 279 revolutions the server's
`max_gap_q6` never exceeded 10°; the old valid-point figure, recomputed from
the same messages, is shown for contrast:

| mode | revs | no-return (median) | `max_gap_q6` now | valid-point gap (old) |
|---|---|---|---|---|
| 0 Standard | 141 | 53% | median 8.5–8.6°, max 8.8° | median 21–36°, max 37° |
| 1 Express | 138 | 38% | median 7.4°, max 8.2° | median 16°, max 18° |

**Found and fixed: a silent stream could not be stopped.** Mode 4 (Stability)
sometimes sends no bytes at all after `EXPRESS_SCAN` — 2 of 11 attempts, both
times entered from another ultra mode (3 and 2). The stream generators in
`driver.py` loop on read timeouts without yielding, and the server only
checked for Stop between yielded items, so the worker hung, could not idle the
sensor on disconnect, and needed SIGINT. All three generators now take a
`stop` predicate polled on every timeout, and the server passes its own.
Verified on host against a transport that never returns data, then on the Pi:
the second silent mode 4 stopped on request, motor off, no warning.

Still not understood: **why** mode 4 goes silent. Not chased. Worth knowing
before the C++ relies on mode switching, since the SDK will meet the same
device behaviour.

### The protocol question this run will answer

`start_raw_express()` deliberately trusts the descriptor the device returns
rather than the mode table, because the two are not known to agree:
`GET_LIDAR_CONF` calls mode 0 "Standard" (streams 0x81), yet `EXPRESS_SCAN`
with `working_mode` 0 yields express capsules (0x82). Those cannot both be a
plain identity mapping.

The server prints this when they disagree:

```
NOTE: mode N advertised 0xAA but the descriptor says 0xBB; trusting the descriptor.
```

Cycling through all five modes in the GUI and recording those lines settles
the mapping, which the C++ driver will need regardless of whether it uses the
SDK. Cheap, and it comes free with testing the viewer.

**Partly answered 2026-10-01:** across three full five-mode cycles and five
more runs of the ultra modes, the server printed **no** NOTE line. So `EXPRESS_SCAN`
with `working_mode` 1–4 answered with the type the mode table advertises for
modes 1–4 (0x82, then 0x84 three times). Two limits:

- Mode 0 goes through legacy `SCAN`, not `EXPRESS_SCAN`, so the reported
  contradiction — `working_mode` 0 yielding 0x82 — was not exercised.
- Modes 2–4 all answer 0x84, so the descriptor cannot show whether those
  three are permuted among themselves.

## 3. DONE — the C++ build chain

Full detail in [`cpp-build.md`](cpp-build.md). Summary:

```
git submodule update --init --recursive     # once after cloning
scripts/build_cpp.sh                        # aarch64, in a container
scripts/deploy_cpp.sh cpane@<pi> lidar_info
```

- Vendor SDK is a **pinned git submodule** at `third_party/rplidar_sdk`
  (`99478e5`). Our build description is in `third_party/CMakeLists.txt`,
  deliberately OUTSIDE the submodule.
- Two configurations: `host` (native, for tests) and `pi` (container,
  deployable). `CMakePresets.json` names them; the banner reports which one
  you actually built, derived from the compiler rather than from a flag.
- `cpp/probes/LidarInfo.cpp` runs on the Pi and its output matches the
  findings document independently.
- CLion: open the **repository root**, not `cpp/`. A Docker toolchain using
  `heron-build:trixie` builds the Pi target in-IDE; `gdb` is in the image
  for that reason.

## 4. DONE — the Scan type

`cpp/include/heron/lidar/Scan.h`, tested by `cpp/tests/ScanTest.cpp`
(35 checks, passing on the host build and on the Pi).

### Reviewed 2026-10-01: coverage was measured on the wrong samples

The strawman computed `worst_gap_rad` between **valid** returns, against an
8° threshold taken from the findings — but the findings' 8–9° was measured
over **all** samples. Checked against every capture (981 revolutions):

| | widest gap per revolution | `isCoverageOk()` |
|---|---|---|
| all samples, seam closed | 8.0–8.9° | — |
| valid returns only (what the strawman did) | 30–115° median | failed 981/981 |

A no-return still proves the beam looked there. What changed:

- `worst_gap_rad` → **`worstSamplingGapRad`**, over every sample.
- **`unobserved`**: a list of `Arc`s wider than `kUnobservedGapRad` (10°),
  plus **`isObserved(bearing)`**. This closes the old "which sectors?" question.
- Default threshold **10°**, not 8°: healthy legacy revolutions reach 8.9°.
- `isCoverageOk()` no longer requires points. All no-return = looked
  everywhere, saw nothing in range.
- The **design doc's pipeline was the source of the bug** and is rewritten
  (§6): gap detection now runs on `grabScanDataHq()` output *before*
  `ascendScanData()`, which overwrites no-return angles in place (read from
  SDK source).
- The **viewer had the same bug.** `lidar_server.py` now uses
  `Revolution.sampling_gap_q6()`, which is also the reference the C++ must
  match. Checked against the captures, then **verified on the Pi** — see
  section 2.

### Second round 2026-10-01: quality, timing, the bearing range

- **`quality` removed from `Point`.** It means something different in every
  mode: legacy is 6 bits with 90–97% of valid returns saturated at 15;
  express has **no quality field** and the SDK substitutes the constant 188;
  ultra's resolution changes with distance. `Point` is now 8 bytes.
- **Per-point time is recoverable.** Sorting by bearing discards time order,
  the sweep starts wherever the sync flag fires, and time runs toward
  *decreasing* robot-frame bearing. New `sweepStartRad` plus
  `timeOf(bearing)`. `tStart` is defined as the SDK's back-dated scan
  timestamp, and `t_end` as one full turn later (estimated).
- **`wrapBearing()`** is the one wrap. Measured over ~1.9e9 inputs:
  `std::remainder` returns exactly +π for +π; the `fmod`, `atan2` and loop
  forms held. The guard in `wrapBearing()` has never been seen to fire.
- **Kept:** bearing in [-π, +π); no pose field; **no scan mode** on `Scan`,
  since quality was the main reason a consumer would need it.

### Still unverified

- ~~Whether an overrun hole survives the SDK.~~ Answered: not reliably
  (section 4c).
- Rotation speed *within* one turn. `timeOf()` assumes it is constant;
  only the period between turns is measured (stdev 1–2 ms).

Settled since: the SDK's `CLOCK_MONOTONIC` **is** `steady_clock`'s clock on
the Pi build (section 4b).

## 4b. DONE — SDK record and replay (2026-10-01)

The design doc's original plan — replay the Python captures through the SDK
— cannot work: the SDK asks the device questions (`GET_DEVICE_INFO`,
`GET_LIDAR_CONF` ×4) before every scan, and the Python captures hold no
answers. So sessions are **recorded through the SDK and replayed in
lockstep**: an answer is served only after the SDK has repeated, byte for
byte, every write that preceded it. Full detail and design in
`lidar-cpp-design.md` §8.

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh cpane@<pi> lidar_record --mode 1 --scans 40 --tag desk
scripts/fetch_captures.sh cpane@<pi>
cmake --build --preset host
build-host/bin/lidar_record --replay captures/desk-sdk1_<stamp>.rpraw
python3 python/tools/lidar_sdkcheck.py captures/desk-sdk1_<stamp>.rpraw
```

Measured, 40 revolutions per mode:

| Mode | Writes matched | Replay = live | SDK = Python decode |
|---|---|---|---|
| 0 Standard | 17 of 17 | 40/40, ×3 runs | 40/40, node for node |
| 1 Express | 11 of 11 | 40/40, ×3 runs | 40/40 |
| 3 Sensitivity (ultra) | 11 of 11 | 40/40, ×3 runs | not checkable — no Python ultra decoder |

A one-node, 1 mm edit fails both checks and is located to the node.
`cpp/tests/ReplayTest.cpp` (17 checks, host and Pi) covers the lockstep gate,
divergence and pacing without hardware.

Two things the recordings settled on the way:

- **Clock:** SDK scan stamps and `steady_clock` share an epoch on the Pi, and
  every scan stamp lands 0.3–17 ms before the bytes that carried it.
- **SDK conversions, confirmed by the exact match:** legacy quality is
  multiplied by 4; express quality is the constant 188; express `flag` is 2,
  not 0, on a non-sync node.

Not established: that the SDK's **ultra** decode is right. The replay proves
only that it is reproducible.

**Fixtures are git-ignored.** The first ultra fixture,
`desk-sdk3_20261002T032343Z`, plus `desk-sdk0_…032334Z` and
`desk-sdk1_…032250Z`, exist in `captures/` on this host and on the Pi only.

## 4c. DONE — starved-reader recordings (2026-10-04)

Express mode, recorded with `lidar_record`, analysed with
`python/tools/lidar_overrun.py`, which checks for loss three ways: read
timing, capsule start-angle jumps on the wire (the ground truth), and what the
SDK delivered. Full table in the findings, protocol trap 10.

- **Four CPU hogs: no loss.** The native SDK reader kept up; the old Python
  overrun probably came from the interpreter lock (not demonstrated).
- **Freezing the reader 0.8–1.5 s did lose capsules**, and the SDK showed it
  two ways: once as a 45° gap, and twice as a **merged scan** (929 samples in
  run 1, 827 in run 2, against a normal ~530) with no gap at all. **The adapter must check samples per scan**
  as well as the gap; the gap check alone missed every loss in the second run.
- **An SDK bug, now patched.** `unbindAndClose()` frees queued buffers with
  `delete []` though they were made with `new`. Replaying the overrun
  recording aborted in `disconnect()`. Fixed by a one-line build-time patch in
  `third_party/CMakeLists.txt`; the submodule is untouched, and configure
  fails if the patched line ever changes (`cpp-build.md`).
- **Fixtures** (git-ignored, on this host and the Pi):
  `starve-stall-sdk1_20261004T191338Z`, `starve-stall2-sdk1_20261004T192626Z`,
  and the no-loss control `starve-cpu-sdk1_20261004T191125Z`. Both stall
  recordings replay with the same anomalies.

Open from this work: what the adapter publishes for a merged scan (drop it, or
flag it not covered), and why the SDK decodes the two scans at a
discontinuity differently between live, replay and Python.

## 5. Superseded — the original Scan interface sketch

Replaced by `cpp/include/heron/lidar/Scan.h`, whose comment block now
carries every decision and the measurement behind it. The original sketch,
and its rationale, are in this file as of `18552e4`.

## 6. DONE — the adapter (2026-10-04)

Decision already made and still standing: **use the vendor SDK, wrapped behind
the interface above.** The default scan mode is 1, Express (decided
2026-10-04): its decode is proven against Python; mode 3 is selectable through
`LidarConfig::scanMode` once its ultra decode is checked. `-1` would select
mode 3 on this unit, so the default is explicit.

### What was built

| Piece | Where | What it does |
|---|---|---|
| Public API | `cpp/include/heron/lidar/Lidar.h` | `Lidar`: `start()`, `stop()`, `getLatest()`, `getHealth()`. No SDK types |
| Implementation | `cpp/src/lidar/Lidar.cpp` | owns the driver and a `std::jthread` reader; the only production file with SDK headers |
| Scan builder | `cpp/src/lidar/ScanBuilder.*` | SDK-free: gap detection, sensor-to-robot transform, filter, sort, sample-count check |
| Library | `heron_lidar` | links the SDK **privately**: code that links it gets no SDK include path |
| Tests | `ScanBuilderTest` (33 checks) | transform table, gaps, seam, order, merge/short counts |
| Checks | `scan_builder_check`, `lidar_replay_check` + `lidar_gapcheck.py`, `lidar_adaptercheck.py` | builder and adapter against the Python reference, on recordings |
| Probe | `cpp/probes/LidarAdapter.cpp` (`lidar_adapter`) | the adapter live, reported once a second |

Design changes made while building, each measured first:

- **`ascendScanData()` is not used at all.** It only sorts, and it overwrites
  no-return angles. The adapter measures gaps on the raw samples, then filters,
  transforms and sorts its own copy, which removes the ordering trap.
- **SDK stamps are converted, not trusted as `steady_clock`.** On macOS
  `steady_clock` is `CLOCK_MONOTONIC_RAW`, the SDK's is `CLOCK_MONOTONIC`, 2.9 s
  apart. The adapter converts by age at each scan; exact on the Pi.
- **The period is a median of normal intervals**, and `seq` advances by the
  revolutions elapsed, so skips made by the SDK's own buffer show up too.

Two decisions (2026-10-04), same principle: never publish times or geometry
that cannot be trusted. Both still advance `seq` and are counted in health.

- **Merged scans are dropped** (sample count over 1.5x expected).
- **Backlog scans after a stall are dropped.** The SDK stamps a post-stall
  burst on arrival (intervals of 6 ms, even −4 ms), so those scans claimed to be
  fresh with data up to 1.5 s old. After an interval over 1.5x the period, scans
  are dropped until one arrives at a normal interval. The last revolution before
  a stall is still published, truthfully stamped and stale on arrival, so a
  consumer's freshness check rejects it.

### Verified

- **On recordings:** every published scan matches the Python decoder point for
  point in all 7 legacy and express recordings (healthy, CPU-load and both
  stalls); ultra runs clean; no scan ever arrives from the future.
- **Builder gap vs Python `sampling_gap_q6()`:** 516 scans, worst difference
  0.0052° (Q6-to-Q14 rounding).
- **Live on the Pi:** 7.5 Hz, ~330 points per scan, coverage ok, worst age
  40 ms, 1.2% CPU, 3.5 MiB. A 1.5 s stall: 12 `seq` skips reported, 3 backlog
  scans dropped, normal the next second. Ctrl-C: clean stop. Four CPU hogs: full
  rate, no skips, no drops.

### Open from the adapter

- **`stop()` takes ~1 s.** Likely the SDK's receive thread waiting out its 1 s
  `waitForDataExt` during `disconnect()`; motor off is sent before that. Not
  measured phase by phase.
- **Robot-wide health shape and reconnection** are not designed yet; `LidarHealth`
  is a first version.
- **Rotation was 7.33 Hz under CPU load against 7.51 Hz idle.** Possibly supply
  sag slowing the motor; unverified. The period tracking follows it.

## 7. DONE — first real behaviour, as a test program (2026-10-04)

**Scope for now (decided 2026-10-04): a reusable LiDAR, not a robot.**
Odometry and the remote control have not been explored yet, so robot-level
design (arbiter, motor link, shutdown path, deskewing, mounting offset) waits.
LiDAR work in order: this test program, a C++ backend for the GUI, then
lifecycle hardening (restart, unplug/replug, the slow `stop()`), then mode 3.

- **`SectorCheck.h`** (public, header-only, SDK-free): `checkSector()` applies
  the four consumer rules (fresh, covered, sector observed, nearest) to any
  sector; `isSectorObserved()` tests unobserved arcs against the sector exactly,
  so an arc that only clips the edge is caught (a 5-degree stepping check would
  miss it). `SectorCheckTest`: 14 checks, host and Pi.
- **`lidar_nearest`** (`cpp/probes/LidarNearest.cpp`): reports CLEAR / STOP /
  UNKNOWN-with-reason ten times a second, live or `--replay`; `--arc`,
  `--center`, `--stop` to play with. Live on the Pi: a steady 2.05 m at -18 deg,
  scan age 23-123 ms. On the stall recording, each stall shows as UNKNOWN
  (stale scan) and recovers.

    scripts/deploy_cpp.sh cpane@<pi> lidar_nearest --arc 90 --stop 0.4
    build-host/bin/lidar_nearest --replay captures/<stem>.rpraw

## 8. DONE — a C++ backend for the GUI (2026-10-04)

`cpp/tools/LidarGuiServer.cpp` (`lidar_gui_server`) serves the adapter's scans
to the existing viewer over the same protocol, version 1, with additive
optional fields for robot-frame points, coverage, unobserved arcs and dropped
samples (documented in `wire.py`). The GUI draws robot frame when those fields
are present, and the Python server still works unchanged. Usage in
`docs/lidar-gui.md`. `Lidar::getInfo()` was added for the viewer's hello:
device identity and scan mode, captured at start with no extra SDK calls.

Verified: protocol and GUI parsing, on a replay and live from the Pi (46 scans
in 8 s, no seq gaps, coverage ok, ~4.3 KB per message); start, stop, unknown
commands, disconnect leaving the sensor idle. **Confirmed by eye
(2026-10-04):** an object on the sensor's left draws on the plot's left, and
the frame caption reads correctly. That check found the caption had always sat
on top of the "180" label; it is now two short lines in the bottom-left
corner, and the stats panel names REP 103 too. **Still unexercised:** the
shaded unobserved arcs; every lossy scan in the recordings arrived in a
post-stall burst and was dropped.

## 9. NEXT — lifecycle hardening

Restart, unplug and replug with reconnection, and the ~1 s `stop()`. Then
mode 3, once something independent can check its ultra decode.

---

## Validating the C++ against the Python

Now built: `lidar_sdkcheck.py` decodes an SDK-recorded `.rpraw` with the
Python decoder and requires every SDK scan to match it node for node (section
4b). Same bytes in, same numbers out. Python-recorded captures are still
checked with:

```
python3 python/tools/lidar_rawcat.py captures/<name>.rpraw --verify
```

**Note:** captures are git-ignored, so the local `captures/` directory may be
empty after a fresh clone. Record SDK sessions with `lidar_record`, Python
ones with `probes/lidar_04_capture.py`. Worth keeping a few fixtures: an open
room, a close wall, a cluttered scene. One ultra fixture now exists, locally
only (section 4b).

---

## State of the hardware

- Pi 3B at `<pi>` (wired link), user `cpane`. SSH key installed, no password
  needed. `sudo` still prompts.
- LiDAR at `/dev/rplidar` (udev rule installed). Firmware 1.29, HW 7, health 0.
- **Power issue resolved.** New supply, `throttled=0x0` idle, while scanning,
  and under CPU load. The old `0x50000` readings are historical.

## What is still genuinely unverified

- **Decoding at an overrun discontinuity.** The two scans touching each
  overrun differ between live and replay, and from the Python decoder; all
  other scans match exactly (section 4c).
- **Where the extra serial buffering lives.** A 1.5 s stall lost ~250 bytes,
  far less than a 4 KB buffer predicts.

- **The `working_mode` vs scan-mode-id mapping — partly.** Modes 1–4 agree
  with the table as far as the descriptor can show (section 2). Still open:
  `working_mode` 0 under `EXPRESS_SCAN`, and whether 2–4 are permuted. The
  latter needs the ultra decoder, or the SDK's own mode names.
- **Why mode 4 sometimes streams nothing** — 2 of 11 attempts, 2026-10-01.
  The server now survives it; the cause is unknown.
- **Whether the SDK's ultra decode is correct** (modes 2, 3, 4). Ultra
  streams genuine 132-byte 0x84 frames on this unit (2026-10-01), and the SDK
  decodes mode 3 live and replays it identically (section 4b), but nothing
  independent checks the numbers. Needed before mode 3 can be the default.
- **Restart and reconnection.** Stopping and starting the same `Lidar`
  repeatedly, and recovering when the USB device disappears and returns, are
  not tested or designed (section 9).
- **Why `stop()` takes ~1 s.** Likely the SDK's receive thread waiting out a
  1 s timeout in `disconnect()`; not measured phase by phase (section 6).
- **Rotation speed within one turn**, which `timeOf()` assumes is constant;
  and why rotation dropped to 7.33 Hz under CPU load from 7.51 Hz idle
  (section 6).
- **The viewer's shaded unobserved arcs** have never been drawn from real
  data (section 8).
- **The serial number byte order.** The SDK demo prints it forward, this
  repo's Python reverses it, and the findings record the reversed form.
  `lidar_info` prints both; only the housing label settles it.
- **Invalid fraction below 61%.** One express revolution read 33% on
  2026-09-29; on 2026-10-01 the live server read a median 38% express and 53%
  legacy over ~140 revolutions each. So the 61–84% range is scene-specific,
  not a property of the sensor. Still wanted: a proper *capture* of the
  closer scene with the scene described, before the findings' range is
  widened — the live runs were not recorded.

- Health states 1 and 2 — never observed, cannot be produced without a fault.
- FORCE_SCAN returns nulls on a stationary head in 10 of 12 runs, real data in
  2. Settle time and motor coasting are both ruled out. Likely just what the
  parked head faces; test by putting an object ~30 cm away and re-running
  `probes/lidar_06_capabilities.py`.
- The ultra capsule format (modes 2, 3, 4) is decoded only by the SDK; the
  Python library has no ultra decoder. Spec at pages 25-27 and 31 of the
  protocol PDF, indexed in `docs/lidar-references.md`.

## Key measured facts, so they are not re-derived

| | |
|---|---|
| Legacy rate | 1954 samples/s (device claims 1969) |
| Express rate | 3911 samples/s (claims 3937) |
| Rotation | ~7.3-7.5 Hz, period stdev ~1-2 ms |
| Samples/rev | ~262-267 legacy, ~530 express |
| Invalid returns | 61-84% in the recorded scenes; 38-53% seen live in a closer one (not yet captured) |
| Stream order | **Not angular** — ~12% arrive behind the previous angle |
| S flag | Fires at ~358.6 deg, never at 0 |
| Zero bearing | Out the **front**, on the housing symmetry axis |
| Angle direction | **Clockwise** viewed from above (left-handed vs REP 103) |
| quality == 0 | Means exactly "no return" (0 valid out of 15,619) |
| Scan modes | 5; device recommends mode 3 (ultra). The SDK decodes it, but nothing independent has checked that; the default here is mode 1 (Express) |
| Worst gap between samples | 8.0-8.9 deg per revolution when healthy, both modes; 10 deg is the coverage threshold |
| Overrun symptoms | a wide gap, or two revolutions merged into one over-full scan with no gap at all |
| SDK stamps after a stall | arrival times, not measurement times (intervals of 6 ms, even negative) |
| Clocks | the SDK's `CLOCK_MONOTONIC` is `steady_clock` on the Pi, but 2.9 s from it on macOS |
| Adapter cost on the Pi | 1.2% CPU, 3.5 MiB, 4 threads (2 of them the SDK's) |

Full detail in `docs/rplidar-a1m8-findings.md`.
