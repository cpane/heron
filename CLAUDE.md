# Heron — project notes for Claude

Robot platform on a Raspberry Pi 3, in hardware bring-up. Each sensor or
subsystem is developed as its own **workstream**: explored, built as a
reusable piece, and verified on the hardware, before the robot design that
ties them together. The LiDAR workstream has a working C++ library and is
paused; odometry and the remote control are next to be explored.

## Workstreams

Workstreams pause and resume independently, so their context must not mix:

- **One resume file each:** `docs/RESUME-<area>.md`. `docs/RESUME.md` is only
  an index of them. If the user says "resume" or "continue" without naming an
  area, read the index; if more than one workstream is active, ask which.
- **Update only the active workstream's files.** Do not record one
  workstream's progress, decisions or open items in another's resume file or
  docs. Shared facts (hardware, build chain, conventions) go in this file or
  `docs/cpp-build.md`.
- **Name its documents `docs/<area>-*.md`**, and its memory entries with the
  area as a prefix (`lidar-...`). General memories carry no prefix.
- **A new workstream** gets a resume file, a row in the index, and a section
  below. `docs/RESUME.md` lists the steps.

### LiDAR — RPLIDAR A1M8 (paused 2026-10-04)

Read before doing LiDAR work:

- **`docs/RESUME-lidar.md`** — where the LiDAR work stopped and what is next.
- `docs/rplidar-a1m8-findings.md` — what this sensor actually does, measured.
  Several behaviours are not what the spec implies. Check here before
  asserting anything about the sensor. Do not re-derive facts that are in it,
  and do not contradict it from general knowledge about RPLIDARs: it was
  measured on this unit.
- `docs/lidar-references.md` — SLAMTEC's official protocol spec, indexed by
  page, plus which SDK file settles which question.
- `docs/lidar-cpp-design.md` — how the C++ is structured around the vendor
  SDK, and why. Read before changing anything in the LiDAR code under `cpp/`.
- `docs/lidar-gui.md` — the live viewer: the Python server and the C++ one
  (`lidar_gui_server`), and the GUI on the host.
- `docs/lidar-start-here.md` — the guided walkthrough of the Python probes
  (for the user, not for you).

LiDAR-only facts:

- **Use `/dev/rplidar`, never `/dev/ttyUSB0`** — `ttyUSB` numbering is
  enumeration order and will move once the Arduino is attached. The udev rule
  is in `target/udev/`.
- `python/heron/lidar/protocol.py` and `express.py` are written as C++
  transliteration targets: no I/O, integer fixed point, explicit shifts. Keep
  them that way.
- Scope until odometry and the remote are understood: a reusable LiDAR, not a
  robot. Robot-level design (motion arbiter, motor link, shutdown path,
  deskewing, mounting offset) waits.

## Development model

Code is written on the host (macOS) and stored in git. **The Pi is a
provisioned execution target, not a dev workstation.** Never edit or build on
the Pi; never `git clone` there. Target configuration must be reproducible
from scripts in this repository.

- `python/heron/` — library, one package per subsystem (`lidar/`, `link/`).
- `python/probes/` — on-target diagnostics, deliberately disposable.
- `python/tools/` — host-side analysis (needs `requirements-host.txt`).
- `cpp/` — production C++: `include/` public headers, `src/` implementation,
  `probes/` on-target diagnostics, `tools/` host-facing programs, `tests/`.
  Builds via the top-level `CMakeLists.txt`; see `docs/cpp-build.md`.
- `third_party/` — vendor code as pinned git submodules. Our build
  descriptions live in `third_party/CMakeLists.txt`, outside the submodules.
- `scripts/` — host-side drivers. `target/` — things that run on the Pi.

## Hardware

Pi 3B, user `cpane`, hostname `heron`. Commands write its address as `<pi>`:
use the wired link (`eth0`), not the Wi-Fi one, which is what its mDNS name
(`heron.local`) resolves to. The address itself is kept out of the
repository. Provisioning sets the hostname and **disables cloud-init**: on
later boots it rebuilt `/etc/hosts` from its cached first-boot user-data,
undoing a rename, and editing `/boot/firmware/user-data` does nothing after
first boot. The network profiles are NetworkManager keyfiles and do not
depend on it. Deployed code and binaries live under `~/heron` on the Pi. SSH key installed, so no
password; `sudo` still prompts. Attached: the RPLIDAR A1M8 (USB). Planned, not yet
attached or explored: an Arduino motor controller, I2C odometry, and a
GPIO/UART remote control.

## Common commands

```
scripts/provision_pi.sh    cpane@<pi>          # rarely
scripts/deploy_pi.sh       cpane@<pi>          # rsync python/
scripts/deploy_pi.sh       cpane@<pi> probes/lidar_01_port.py
scripts/build_cpp.sh                                    # C++ for the Pi, in a container
scripts/deploy_cpp.sh      cpane@<pi> <binary> [args]
scripts/fetch_captures.sh  cpane@<pi>          # pull captures back
```

`deploy_pi.sh` and `deploy_cpp.sh` with extra arguments deploy *and* run, so
stale code cannot be run by accident. Workstream-specific commands are in each
resume file.

## Conventions

**C++** — our code in `cpp/` follows `CPP_CODING_STANDARD.md` (it does not
apply to `third_party/`). Every `.h`/`.cpp` starts with its MIT file header
(the template is in the standard; the license text is in `LICENSE`). Standard
library only — there is no foundation library here. One local reading: plain
aggregates (`Point`, `Scan`, ...) have camelCase public members without `m_`,
per the standard's own aggregate example. No `.clang-format`: it cannot
express the standard's indentation (members of a struct with no access
modifier at +2, members under one at +4).

**Shell** — `#!/usr/bin/env bash`, `set -Eeuo pipefail`, 4-space indent,
`${VAR}` braces always, banner `echo "===="` blocks, `ERROR:`-prefixed
messages, comments explaining *why*. Host scripts take one `user@host`
argument and resolve `REPO_ROOT` via `BASH_SOURCE`.

**Host constraints** — macOS ships `openrsync` (no `--info=`, no `--mkpath`;
use `-rlptvz`, not `-a`) and bash 3.2 (no associative arrays, no `mapfile`,
and `"${ARR[@]}"` on an empty array breaks under `set -u`).

**Python on the target** — dependencies come from apt, declared in the install
list in `target/provision/base.sh`. Debian trixie marks the system interpreter
EXTERNALLY-MANAGED (PEP 668), so `pip3 install` into it is blocked by design.
If a dependency is ever needed that Debian does not package, use
`python3 -m venv --system-site-packages`, do not replace the apt packages.
Host-only dependencies belong in `requirements-host.txt` at the repository
root, never under `python/`, so `deploy_pi.sh` cannot leak them onto the Pi.

**Git** — gitflow, per `CONTRIBUTING.md`. Work on `feature/<name>`,
`bugfix/<issue>` or `chore/<name>` (docs, tooling, upkeep; no issue needed)
branched from `develop`, merged by pull request; never commit
directly to `develop` or `main` (releases only). Short imperative subjects,
capitalized, no ticket refs. Commit in logical groups rather than one lump.
Author is `cpane <cpane@icloud.com>`.

**Captures and reports are git-ignored.** So is `docs/vendor/`.

## Working style

Measure rather than assume, and say which one you did. Several conclusions in
this project were wrong on the first attempt and were corrected by testing
against the hardware — that is the expected workflow, not a failure. When a
claim has not been verified, label it as unverified rather than stating it
confidently.
