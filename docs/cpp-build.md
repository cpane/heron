# Building the C++ for the Pi

Two build configurations, one CMake tree.

| Config | Where it runs | What it is for |
|---|---|---|
| `build-pi/` | Debian trixie arm64 container | What you deploy. aarch64 ELF |
| `build-host/` | your machine, natively | Tests and the adapter, no hardware, no container |

Both are git-ignored.

## Quick start

```
git submodule update --init --recursive      # once, after cloning
scripts/build_cpp.sh                         # builds build-pi/
scripts/deploy_cpp.sh <user>@<pi> lidar_info
```

Host-native, for anything that does not need the target:

```
cmake --preset host && cmake --build --preset host
ctest --test-dir build-host          # the four unit-test suites
./build-host/bin/lidar_nearest --replay captures/<stem>.rpraw
```

## Which target am I building?

The two configurations share target names -- `lidar_info` and `rplidar_sdk`
appear identically in both -- so an IDE cannot tell you which one you are
producing. `CMakePresets.json` exists to make it explicit:

| Preset | Shown as | Output |
|---|---|---|
| `host` | `host-native  --  runs HERE, not deployable` | `build-host/` |
| `pi` | `pi-aarch64  --  DEPLOYABLE (needs the container)` | `build-pi/` |

Every configure prints which one it is:

```
--    TARGET:     host-native  --  runs on THIS machine, NOT deployable to the Pi
--    system      Darwin arm64
--    compiler    AppleClang 21.0.0
```

Selecting `pi` without an aarch64 Linux toolchain is **refused**, not silently
mis-built:

```
HERON_TARGET=pi needs an aarch64 Linux toolchain, but this is Darwin arm64.

  From a terminal:  scripts/build_cpp.sh
  In CLion:         add a Docker toolchain using the image
                    heron-build:trixie, then attach it to the
                    'pi' CMake profile
```

Without that guard the `pi` preset on macOS would produce a Mach-O binary in
`build-pi/bin` with an entirely reassuring name, and the mistake would only
surface as `cannot execute binary file` on the target.

## CLion

**Open the repository root**, not `cpp/`. `cpp/CMakeLists.txt` uses targets
defined at the root, and opening it directly *configures cleanly* and then
fails at compile with `'sl_lidar.h' file not found` -- a misleading error,
because CMake treats the unknown `rplidar_sdk` target as a plain library name.

CLion picks up `CMakePresets.json` and offers both profiles by name. Use
**`host`** for editing, indexing and debugging; it is the one that works with
no extra setup. The Run button then runs the binary on your machine, where
there is no `/dev/rplidar`, so `lidar_info` reports no device. That is the
correct result, not a broken build.

For deployable builds, either use `scripts/build_cpp.sh` from a terminal
(simplest), or add a Docker toolchain in CLion -- Settings, Build Execution
Deployment, Toolchains, `+`, Docker, image `heron-build:trixie` -- and
attach it to the `pi` profile.

**If CLion reports no configurations**, it is probably restoring a cached
project from before this CMake tree existed. Check `.idea/`: a
`<module type="PYTHON_MODULE">` in the `.iml` means it was set up as a Python
project and CLion will never look for CMake. Right-click the root
`CMakeLists.txt` and choose *Load CMake Project*, or move `.idea/` aside and
reopen. `.idea/` is git-ignored, so it is local-only either way.

**`build-pi/` belongs to the container.** Its CMake cache records paths as
`/src/...` because that is where the repository is mounted. Running
`cmake --preset pi` from the host against it gives a cache-path mismatch
rather than the friendly guard message above -- the mistake is still blocked,
just less clearly. Let the script own that directory.

## Why a container

The Pi and the development host were measured, not assumed:

| | Host (macOS) | Pi |
|---|---|---|
| Arch | arm64 | **aarch64** |
| OS | Darwin 27 | **Debian 13 trixie** |
| Compiler | Apple clang 21 | **gcc 14.2.0** |
| glibc | — | **2.41** |
| CMake | 4.1.1 | 3.31.6 |

A `debian:trixie` arm64 container gives the same distro, the same glibc and
the same gcc major as the target, so the binary runs on the Pi by construction
rather than by luck. On an arm64 host that container is native — no emulation,
full speed. On x86_64 it works through qemu, correctly but slowly;
`build_cpp.sh` says so rather than leaving you to wonder.

Building *on* the Pi would also work — it has gcc, cmake 3.31.6 and ninja
installed — but it contradicts the repository's development model, and 905 MiB
of RAM across four cores means `-j4` on C++ will swap. Keep it as an escape
hatch with `-j2`, not a habit.

Cross-compiling from the host was rejected: no aarch64-linux-gnu toolchain is
installed, and a cross toolchain built against a glibc newer than 2.41
produces binaries that link cleanly and refuse to start on the target.

## Why the SDK is a submodule

`third_party/rplidar_sdk`, pinned. Updating it is:

```
git -C third_party/rplidar_sdk fetch
git -C third_party/rplidar_sdk checkout <tag-or-commit>
git add third_party/rplidar_sdk && git commit
```

The one cost is that a plain `git clone` gets an empty directory. Both
`build_cpp.sh` and the CMake check for it and print the fix, because the
default error otherwise fires deep inside the container where it reads as a
missing header.

**Our build description lives in `third_party/CMakeLists.txt`, outside the
submodule.** Anything written inside `third_party/rplidar_sdk/` would be a
local modification to upstream: permanently dirty in `git status`, in conflict
with every `git submodule update`, and gone on the first version bump.

## What gets built, and what does not

The `rplidar_sdk` static library compiles 15 sources, mirroring
`sdk/Makefile` rather than globbing — a glob would silently absorb whatever
upstream adds on the next bump.

- `src/rplidar_driver.cpp` is **excluded**. It is the deprecated pre-`sl::`
  API, and upstream's own Makefile does not build it either.
- The platform layer is chosen by `CMAKE_SYSTEM_NAME`: `arch/linux` or
  `arch/macOS`. Selecting macOS is what makes the host-native build possible.
- **`-D_MACOS` is required on Darwin**, not optional. `sdk/src/hal/` guards
  `pthread_mutex_timedlock` and `pthread_condattr_setclock` behind it because
  macOS has neither, and without it the SDK does not compile at all. Upstream
  sets it the same way (`mak_def.inc`).
- The SDK builds at `-std=c++11` with warnings off and `SYSTEM` includes. Our
  code builds at C++20 with `-Wall -Wextra -Wpedantic -Wconversion` and more.
  Third-party warnings we cannot action must never be a reason to lower our
  own bar.

## Layering

```
rplidar_sdk               vendor, static, plus one build-time patch (below)
  ^
heron_lidar_channels      TapChannel, ReplayChannel: record and replay sessions
heron_rpraw               the .rpraw format; no SDK
heron_scan_builder        gap detection, transform, assembly; no SDK
heron_lidar_types         public headers: Lidar.h, Scan.h, SectorCheck.h
  ^ (types PUBLIC; the other three PRIVATE)
heron_lidar               the adapter, Lidar.cpp: the only production code with SDK headers
  ^
robot code, programs      see Heron::Scan, never sl::  (robot code: later)

Each row uses the rows above it.

cpp/probes/               may use the SDK directly; not production
```

`heron_lidar` links the SDK and the channels **privately**, so anything
that links only it, like `lidar_nearest` or `lidar_gui_server`, gets no SDK
include path. The boundary is enforced by the build. Rationale in
[`lidar-cpp-design.md`](lidar-cpp-design.md).

### What each executable is

| Target | Source | Kind |
|---|---|---|
| `scan_test`, `replay_test`, `scan_builder_test`, `sector_check_test` | `cpp/tests/` | unit tests, run by `ctest`; no hardware, no recordings |
| `scan_builder_check`, `lidar_replay_check` | `cpp/tests/` | checks against recordings; driven by `python/tools/lidar_gapcheck.py` and `lidar_adaptercheck.py`. Not in `ctest`, since recordings are git-ignored |
| `lidar_info`, `lidar_record`, `lidar_adapter`, `lidar_nearest` | `cpp/probes/` | on-target programs |
| `lidar_gui_server` | `cpp/tools/` | serves the library to the viewer |

## Two things the first hardware run established

**`connect()` does not tell you the device is missing.** In
`sl_async_transceiver.cpp`, `openChannelAndBind()` declares `u_result ans` and
then shadows it with `Result<nullptr_t> ans` inside its `do` block. The
`open()` failure is written to the shadowing inner variable, so the function
returns the outer `RESULT_OK` regardless — and `connect()` then sets
`_isConnected = true`, so `isConnected()` agrees.

Observed, not just read: running `lidar_info` on a machine with no
`/dev/rplidar` produced a successful connect followed by three unexplained
query timeouts. **The adapter must verify with a real round trip** —
`getDeviceInfo()` — and must not treat `connect()` as a liveness check.

**The serial number byte order is unsettled.** The SDK's own demo prints
`serialnum[0..15]` forward; this repository's Python reverses it, and the
reversed form is what `rplidar-a1m8-findings.md` records:

```
serial (fwd)    C887ED93C0EA98C9A5E698F25D594669   <- as the SDK demo prints it
serial (rev)    6946595DF298E6A5C998EAC093ED87C8   <- as this repo's Python prints it
```

`lidar_info` prints both. Only the label on the housing decides, and that
check has not been done.

## The one local patch to the SDK

The submodule is never edited, but one upstream bug is fixed at build time.
`AsyncTransceiver` allocates each receive buffer with `new Buffer()`
(`sl_async_transceiver.cpp:331`), and `unbindAndClose()` frees any still
queued with `delete []` (`:255`): undefined behaviour. It only fires when
`disconnect()` runs with data still queued, which an overrun backlog
produces. Measured 2026-10-04: replaying the starved-reader recording aborted
in `disconnect()` on macOS with "pointer being freed was not allocated";
Linux may corrupt the heap silently instead.

`third_party/CMakeLists.txt` copies that one file into the build tree,
replaces the statement with `delete *itr;`, and compiles the copy instead of
the original. The configure output says so:

```
--   rplidar_sdk  1 local patch: unbindAndClose() delete/new mismatch
```

The replacement must match exactly once, so an SDK update that changes or
fixes the line stops configure with an explanation, and the patch is reviewed
rather than silently skipped. That check was tested by breaking it on purpose.
After the patch the same replay exits cleanly, three runs out of three, and a
live stall on the Pi shut down cleanly.

## Verified

`lidar_info` built in the container via the `pi` preset, deployed, and run on
the Pi against the real sensor. Its output matches `rplidar-a1m8-findings.md` independently —
model `0x18`, firmware 1.29, hardware 7, health Good, and the same five-mode
table with the same µs/sample and data types. Two implementations that share
no code agreeing on the device's self-description is a genuine cross-check.

Since then (2026-10-04), every target builds warning-free under both
AppleClang and GCC 14, and all four unit-test suites pass on the host and on
the Pi. What the library itself was verified against is in
`RESUME-lidar.md`, sections 6 to 8.

## Related

- [`lidar-cpp-design.md`](lidar-cpp-design.md) — why the layering is what it is
- [`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md) — the measurements
- [`lidar-references.md`](lidar-references.md) — protocol spec and SDK file index
