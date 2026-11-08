# LiDAR workstream — resume point

**2026-11-08: resumed for one Inbox item,** the missed requests (section
10), on `feature/lidar-missed-requests`. The rest of the Inbox entry is not
folded in yet: its facts wait to be confirmed.

Last updated 2026-10-04. Branch **`develop`**, everything pushed. The Pi
was re-provisioned on 2026-10-04; the tests, recording checks and viewer were
re-verified on it, but `lidar_nearest` and `lidar_adapter` were not re-run
live (section 6, "Re-verified").

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
| `lidar_nearest`: nearest obstacle in a sector | `scripts/deploy_cpp.sh <user>@<pi> lidar_nearest --arc 90` | 7 |
| `lidar_gui_server` + the viewer | `scripts/deploy_cpp.sh <user>@<pi> lidar_gui_server`, then `.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect` | 8, `lidar-gui.md` |
| `lidar_adapter`: live health, once a second | `scripts/deploy_cpp.sh <user>@<pi> lidar_adapter 20` | 6 |
| Record a session / replay it | `lidar_record --mode 1 --scans 40 --tag x` on the Pi; `--replay` on the host | 4b |

Build first with `scripts/build_cpp.sh` (Pi) or `cmake --build --preset host`.
The viewer needs a `.venv` built on a Python that has Tkinter; the README's
host setup says how, and why plain `python3` can pick one without it.
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
scripts/deploy_pi.sh <user>@<pi> server/lidar_server.py
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
scripts/deploy_cpp.sh <user>@<pi> lidar_info
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
scripts/deploy_cpp.sh <user>@<pi> lidar_record --mode 1 --scans 40 --tag desk
scripts/fetch_captures.sh <user>@<pi>
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

### Re-verified on the re-provisioned Pi (2026-10-04)

After re-provisioning (new hostname, udev rule reinstalled, work tree now
`~/heron` on the Pi), with the existing `build-pi` binaries redeployed:

- `/dev/rplidar` resolves through the reinstalled rule. Probe 1 and
  `lidar_info` both read firmware 1.29, HW 7, health Good, and the same serial.
- **Unit tests on the Pi:** `scan_test` 35, `replay_test` 17,
  `scan_builder_test` 33, `sector_check_test` 14 — 99 checks, 0 failures.
- **Recording checks run on the Pi itself**, the Python tools included (they
  need only the standard library), against the 9 recordings that have an
  `.sdkdump`: gap check PASS on all 7 legacy/express, worst 0.0052°;
  `lidar_replay_check` OK on all 9; every published scan matches the Python
  decoder in all 7 (406 scans).
- **The Pi build publishes exactly what the host build does:** its
  `.adapter.dump` files are identical to the host's in every scan and point.
  Only the last field of each `scan` line differs, `ageAtSight`, which is
  wall-clock timing.
- `lidar_gui_server` with the viewer, live: works, and SIGINT stops it with
  the sensor idle.

To run the checks on the Pi, copy each recording's `.rpraw` and `.sdkdump` to
a directory outside `~/heron/captures` (`/tmp/heron-checks` was used), so that
`fetch_captures.sh` cannot bring the Pi's dumps back over the host's.

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

    scripts/deploy_cpp.sh <user>@<pi> lidar_nearest --arc 90 --stop 0.4
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

## 10. IN PROGRESS — missed requests with the motor running (2026-11-08)

From navigation's Inbox entry: on the mains adapter the LiDAR answered 20 of
20 GET_INFO stopped but 11 of 20 running, requests 50 ms apart. How that
test counted a miss was not recorded, so it was repeated with a probe that
classifies each one: `python/probes/lidar_07_requests.py` (ok, none,
garbled, partial; every byte timed; bytes waiting before a request logged
as stray; `get_throttled` polled every 0.25 s).

Conditions for every run below: the bench rig, the Pi on its mains adapter
(`throttled=0x50000` throughout, the boot event only; bit 0 never set in
any sample), `heron_teleop` running with the Arduino attached, robot still,
the Pi reached over Wi-Fi. Captures `captures/{smoke,e1,e1-spinup,e1-window,
e2-gaps,e3-health,e3-rate}_20261108T19*` on the host.

Measured [ours]:

- **11 of 20 did not reproduce.** With the motor running steadily (3 s
  after motor-on), GET_INFO was answered 200 of 200 at gaps of 10, 50, 200
  and 1000 ms. No reply was ever late, garbled or partial, and no stray
  bytes were ever found before a request.
- **Motor stopped: 810 of 810 answered.**
- **Spin-up loses at most one request.** In 8 of 14 motor starts, exactly
  one request got no reply at all (nothing within 20 to 500 ms), at 20, 21,
  21, 41, 60, 60, 167 and 177 ms after motor-on [the 14 runs: e1, smoke,
  e3-health, e3-rate, five in e1-spinup, five in e1-window]. Requests 0.6
  ms after motor-on were always answered; nothing was missed later than 177
  ms. The miss is not at a fixed time: with requests every ~12 ms
  (e1-window) only 2 of 5 starts lost one, both late in the window.
- **Not GET_INFO-specific:** GET_SAMPLERATE lost one at spin-up too
  (e3-rate); GET_HEALTH's one spin-up lost none.
- **Reply latency does not change with the motor:** 3.2 to 4.7 ms typically,
  with occasional 7 to 16 ms replies both stopped and running.
- **The SDK's start-up is clear of the window** [ours, from the code]:
  `startScanExpress()` sends EXPRESS_SCAN right after `startMotor()`
  (`sl_lidar_driver.cpp`, 10 ms after only), and a request 0.6 ms after
  motor-on was always answered here. Consistent with "the SDK started every
  time" in the Inbox entry; not measured on the SDK path itself.

E4, the Arduino unplugged (`captures/e4-noarduino_20261108T*`, 15 starts,
requests every ~10 ms, 100 ms timeout, otherwise as above), measured [ours]:

- **Single misses as before:** 13 of 15 starts lost a request (one lost
  two), at 20 to 272 ms, one at 0.7 ms. The baseline at the same settings
  (e1-spinup) lost one in 4 of 5, so no change shown.
- **A second failure: the LiDAR stayed silent while the motor ran.** In 2
  of 15 starts it stopped answering at 27 and 172 ms after motor-on and
  sent not one byte for the rest of the phase (39 and 32 requests, about 4
  s). It answered at once when the motor was stopped (DTR asserted), and the
  next starts were normal. The kernel logged nothing for its port
  (ttyUSB1) in that time: the USB link stayed up. This could be what
  navigation saw as 11 of 20 [inference].
E4 control, the Arduino back in and `heron_teleop` restarted so it talks to
it again (`captures/e4-arduino_20261108T*`, same settings), measured [ours]:

- **The Arduino makes no difference.** Single misses in 15 of 15 starts
  (some lost two or three, at 20 to 576 ms), and **2 of 15 silent starts
  again** (from 163 and 465 ms after motor-on). The same as unplugged.
- **One silence outlasted the motor:** after one silent start the next
  stopped phase got no reply for its first 6 requests, 0.56 s after motor
  stop, then answered normally. The other silence ended at motor stop.
- **One reply was another command's:** a GET_INFO at 131 ms after motor-on
  was answered `a5 5a 05 00 00 40 81`, the descriptor of a legacy SCAN
  (length 5, multi, type 0x81), with no scan nodes after it; the next
  GET_INFO was answered normally. A well-formed descriptor for a different
  command points to the request being corrupted on its way **into** the
  LiDAR, not the reply on its way out [inference]; the probe cannot see the
  LiDAR's side of the cable.
- **Side finding for the motor workstream:** `heron_teleop` keeps the
  unplugged Arduino's deleted tty open and reports `motors NOT OK` after a
  replug; it does not reconnect. (For motor's Inbox at pause.)

By ear during the E4 control [ours, heard, not measured]: the motor sounded
steady through every start, with no stutter or stop noticed, and possibly a
short delay before it began to turn. So the silent starts are probably not
the LiDAR stalling or restarting (that would stop the motor) [inference].
When listening, note that each phase is followed by `--pause` (2 s) with the
motor as it was: a normal start spins ~2.6 s, a silent one ~5.5 to 6 s.

E7, soft starts (`--phases ...,soft`: DTR switched at 100 Hz, on 10% to
100% over 400 ms; each switch took 0.2 to 1.9 ms, median 0.3, so the ramp
ran as asked), 2026-11-08 ~15:45, measured [ours]:

- **Not comparable with the runs before:** in the same runs the motor-off
  phases now lost requests too (15 + 1 garbled, then 1, then 2 of 40 at
  10 ms; the first ended silent with the motor never on), where 1400
  motor-off requests at 10 ms were clean before 15:25. A motor-off run at
  50 ms between them was clean (100 of 100, `e7-stoponly`).
- Soft starts lost 34, 6 and 9 of 40, misses spread through and past the
  ramp. No conclusion from them until the baseline is stable again.
- **What changed:** between the E4 control (15:25) and E7, the adapter board
  was handled to look for a second micro-USB. A disturbed connector between
  board and sensor would fit everything so far [hypothesis, untested]: a
  request corrupted on its way in, worse while the motor vibrates, and now
  intermittent with it off. Next: reseat both connectors with the motor
  stopped, then repeat the motor-off check and the E4 baseline.

Unit A again, ~16:05, nothing touched since (`e8-a-stopped`, `e8-a-starts`,
E4's settings) [ours]: motor off 700 of 700; starts lost a request in 14 of
15, one silent (29 misses). Back to the pattern before 15:25: the motor-off
misses of ~15:45 were transient, cause unknown. The board was not handled
then after all (the user looked at it without touching the LiDAR), so the
connector hypothesis is weakened.

**E8, the second A1M8 (unit B), ~16:10:** same cable, Pi port, settings
and conditions, minutes after unit A's check (`e8-b-stopped`,
`e8-b-starts`) [ours]: motor off 700 of 700; **starts 0 of 15 with a miss,
0 silent, 600 of 600 answered while running.** Unit A at ~16:05: 14 of 15
and 1 silent. **The spin-up failures belong to unit A**, not the Pi, its
supply, the cable or the port. Both report firmware 1.29, hardware 7
(GET_INFO `1d01 07`), so not a firmware version. GET_INFO serial fields
(payload order): unit A `c887ed93c0ea98c9a5e698f25d594669`, unit B
`d6b3fa92c3e49ec8a5eb9bf8406e5463`. Which part of unit A (its adapter
board, the cable to the head, the head itself) is not known.

**Unit A again after the swap back, ~16:12** (its micro-USB pulled and
re-inserted at its adapter board; same cable, port and conditions;
`e8-a2-stopped`, two `e8-a2-starts`) [ours]: motor off 100 of 100; **starts
2 of 30 with a miss, both at ~20 ms, none silent**, against 42 of 45 with a
miss and 5 silent for unit A earlier in the day. Serial confirmed as unit A.

**Conclusion so far:** the failures came mainly from the micro-USB
connection at unit A's adapter board. Re-seating that plug is the only thing
that changed, and it took unit A from 42 of 45 to 2 of 30 [ours]. How a
poor contact loses requests is an inference: it adds resistance in the 5 V
and ground path, so the motor's start-up current makes the board's supply
and ground bounce, corrupting the UART between the board's CP2102 and the
head (USB itself is differential and stayed up). That fits the misread SCAN
and the silences ending when the motor stopped. The robot is driven around,
which could work a plug loose; navigation's 11 of 20 likely came from this
[inference]. Whether 2 of 30 is a residue of the contact or normal for the
A1 is not known (unit B: 0 of 15; too few to tell).

**Wiggle test, 16:24, unit A** (`wiggle_20261108T*`; motor running
steadily, requests every 50 ms for 80 s; plug pressed side to side at
25-45 s and 65-85 s) [ours]: **379 of 749 unanswered and 2 garbled, in the
hands-off windows as much as the wiggle ones**, from the first request:
silences of 4 to 6 s alternating with 1 to 6 s of replies that lost about
every fifth request. Steady running had been 200 of 200 earlier. No
under-voltage; the motor sounded as before (user). The user may have
touched or moved the plug before 16:24 while getting ready, and the robot
shakes continuously from the LiDAR's own spin. So the contact was probably
disturbed before the first window; the test could not separate wiggling
from not [ours: the data; the handling is the user's recollection]. Unit
A's state has followed handling three times (as installed: bad; re-seated:
2 of 30; handled: bad), consistent with the contact, but not yet a
controlled test. Next: re-seat firmly, confirm a clean steady baseline, then
wiggle in a run that starts clean.

**Second wiggle test, 16:34, unit A after a firm re-seat** (the plug felt
snug, user) [ours]: a hands-off baseline first, motor running steadily,
491 of 491 (`reseat2-steady`); then the clocked run (`wiggle2`), 1065 of
1065, no miss at any time. The user wiggled from 16:34:30 to 16:34:40 by
their clock, but reported it done when the Pi's NTP clock read 16:34:12,
so their clock runs at least ~20 s ahead and the wiggle may have fallen
before or at the very start of the requests (they began 16:34:05): the run
does not show whether a gentle wiggle breaks a snug plug. What it does show:
seated firmly, unit A answered 1556 of 1556 with the motor running.

**Unit A, firmly seated, 16:36-16:40** [ours]: 15 starts, 0 with a miss,
1200 of 1200 answered (`e8-a3-starts`), as unit B. Then an express
recording, 220 revolutions, robot still, Pi on its mains adapter, the
same room as navigation's (`unitA-reseat-sdk1_20261108T213830Z`; replay
check 217 scans, invariants held, nothing dropped). Both recordings run
through one script (scratch, not kept: the SDK dump's every sample, first
two scans dropped; it reproduces navigation's figures from theirs exactly):

| | navsim 18:52 (plug as installed) | unit A re-seated 21:38 UTC |
|---|---|---|
| Rotation | 7.10 Hz, period 139.6-142.2 ms | **7.52 Hz**, 131.9-134.1 ms |
| Samples a revolution | 549-554, mean 550.8 | 519-521, mean 519.8 |
| Sample rate | 3908/s | 3908/s |
| Count-implied period sd / stamped sd / correlation | 0.19 / 0.51 ms / 0.40 | 0.14 / 0.59 ms / 0.00 |
| No-returns | 36-40%, runs mean 5.2, max 70 | 33-39%, runs mean 4.4, max 25 |
| Widest gap | 6.5-8.3°, mean 7.50°, at ~220° | 7.7-8.2°, mean 7.97°, at ~188° |
| First (sync) node | median 355.2° | median 359.6° (359.3-360.0) |

- **The 7.10 Hz was the plug** [ours: same unit, same supply, before and
  after re-seating; mechanism inference: the contact's resistance lowered
  the motor's voltage]. 7.52 Hz is the findings' range on this supply
  (7.45-7.52 Hz).
- **The stamped period spread is timestamp jitter**, now from two
  recordings: the count puts the true spread at 0.14-0.19 ms, stamps show
  0.5-0.6 ms, and they barely correlate (0.40, 0.00).
- **The gap's bearing is not a property:** 220° then 188° in the same room;
  one wide gap a revolution in both.
- **No-returns 33-40%** in both, same room; the room is not yet described,
  which the findings' open item asks for.

A miss with the motor stopping: the first request 0.44 s after the port's
open stopped a free-running motor got no reply (`dur-check`); e7-check's
first phase began the same way.

Untested predictions: unit A's 7.10 Hz rotation (navigation's Inbox, the
plug as it was) was slowed by the same contact; a recording now would show
it. A loose plug could also have added to the power block's 4 of 20.

**The motor spins whenever no program holds the port** [ours, seen by the
user: it spins all the time the robot is driven with the remote; no process
had the port open, and `heron_teleop` does not use the LiDAR]. With the port
closed, DTR is not held and the motor runs (findings, protocol trap 1), so it
has free-run between every run here and through every drive. To fix later
(wear, the 5 V budget); for navigation's and motor's Inboxes at pause.

Up to 15:25: the failures all start in the first 0.6 s after motor-on, the
Arduino is not involved, and the Pi's rail shows no under-voltage.

Open:

- **What navigation's test did differently** [unverified]. A test that
  restarted the motor, or reopened the port (which stops it), for each
  request would put every request in the spin-up window; a reply timeout
  under ~16 ms would count slow replies as misses. Neither is known.
- **Why spin-up loses a request:** a supply dip from the motor's inrush, or
  the firmware busy starting up. `get_throttled` saw no under-voltage, but
  it samples the Pi's rail every 0.25 s, not the LiDAR's. Whether the
  request or the reply was lost is not known: the probe sees only the Pi's
  side.
- **Next experiments** (the plan of 2026-11-08; E4 done, above): E5, 15
  starts with the LiDAR powered from its own supply, through a powered USB
  hub (its adapter board has only the one micro-USB [ours, looked], so it
  cannot be fed separately there), same
  settings as E4; E6, the adapter's 5 V on a multimeter at spin-up. The
  baseline at E4's settings is 28 of 30 starts with a miss and 4 of 30
  silent, so none in 15 would be telling.

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

- Pi 3B, hostname `heron`, reached at `<pi>` over its wired link. Access
  details (user, SSH key, `sudo`) are per-developer: see `CLAUDE.local.md`.
- LiDAR at `/dev/rplidar` (udev rule `99-heron-rplidar.rules`, installed by
  provisioning). Firmware 1.29, HW 7, health 0. Deployed code and binaries
  live under `~/heron` on the Pi.
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

## Inbox

Facts other workstreams found that belong here, waiting to be checked and
folded into this workstream's docs when it resumes (`CLAUDE.md`,
"Workstreams"). Newest last; delete an entry once folded in.

### From navigation, 2026-11-08: a 220-revolution express recording

Recorded for the simulator's comparison: `lidar_record --mode 1 --scans 220
--tag navsim` on the bench rig, robot still, the Pi on its mains adapter;
`captures/navsim-sdk1_20261108T185202Z.*` on the host (git-ignored).
Replayed by `lidar_replay_check`: 217 scans, invariants held, nothing
dropped. The full write-up is `navigation-cpp-design.md` §3 (branch
`feature/navigation-geometry` until merged). Against the findings:

- **Rotation 7.10 Hz** (stamped period 139.6 to 142.2 ms, mean 140.9),
  not the findings' 7.35 Hz, on the same adapter as October. Why is not
  known [unverified].
- **Samples a revolution 549 to 554**, mean 550.8, stdev 0.75. The sample
  rate held: 3908 a second, against 3904 from the findings' 531 at 136.0
  ms. So the count follows the period, as the findings say.
- **The findings' period stdev of 0.8 ms is probably timestamp jitter**
  [ours, inference from one recording]: the count's 0.75-sample spread
  at 254 us a sample puts the true spread near 0.2 ms, the stamped spread
  here was 0.51 ms, and count and stamped period correlate only 0.40.
  Worth a second recording before it goes into the findings as measured.
- **Widest gap 6.5 to 8.3°, mean 7.50°** (findings: medians 8.0 to 8.1°),
  at one bearing, 210 to 221° in 204 of 219 revolutions; the second-widest
  gap median 4.7°.
- **No-returns 36 to 40%** in this room (findings: 61 to 84%), in runs of
  5.2 samples on average, median 2, longest 70.
- **Sync** at a median of 355.2° (findings 351.4 to 359.5°).
- **Supply.** On the robot's power block the Pi was under-voltage and the
  LiDAR answered 4 of 20 GET_INFO requests, so `lidar_info` and
  `lidar_record` failed at start-up: the SDK's own GET_INFO in `connect()`
  was answered, the probe's second one was not (strace). On the mains
  adapter: 20 of 20 with the motor stopped, but 11 of 20 with it running
  and requests 50 ms apart [cause unverified]; the SDK then started every
  time. The supply table is `rig-profiles.md` §1, "Powering the Pi"
  (branch `chore/rig-pi-power` until merged).
