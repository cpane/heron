# Heron — project notes for Claude

Robot platform on a Raspberry Pi 3, in hardware bring-up. Each sensor or
subsystem is developed as its own **workstream**: explored, built as a
reusable piece, and verified on the hardware, before the robot design that
ties them together. The LiDAR, remote control and odometry workstreams
each have a working C++ library and are paused. The motor controller
workstream is active: everything up to driving by hand with the remote
(`heron_teleop`) is built and checked on the bench rig; the mini robot on
the floor is next (`docs/RESUME-motor.md`).

## Workstreams

Workstreams pause and resume independently, so their context must not mix:

- **One resume file each:** `docs/RESUME-<area>.md`. `docs/RESUME.md` is only
  an index of them. If the user says "resume" or "continue" without naming an
  area, read the index; if more than one workstream is active, ask which.
- **Update only the active workstream's files.** Do not record one
  workstream's progress, decisions or open items in another's resume file or
  docs. Shared facts (hardware, build chain, conventions) go in this file,
  `docs/cpp-build.md`, or `docs/rig-profiles.md` (how each physical rig is
  described) and `docs/rig-calibration.md` (how to measure one).
- **Telemetry:** external clients will read the LiDAR and odometry live
  through a server inside the robot process, read-only, never slowing the
  control loop. Every library keeps the `getLatest()` shape that allows it;
  the rules are `docs/robot-telemetry.md`.
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
  enumeration order. The LiDAR and the Arduino use identical CP2102 bridges
  (10c4:ea60, serial 0001), so the udev rules in `target/udev/` match the
  **USB port**: LiDAR in 1-1.4, Arduino (`/dev/arduino`) in 1-1.2. A device
  in another port gets no name.
- `python/heron/lidar/protocol.py` and `express.py` are written as C++
  transliteration targets: no I/O, integer fixed point, explicit shifts. Keep
  them that way.
- Scope until odometry and the remote are understood: a reusable LiDAR, not a
  robot. Robot-level design (motion arbiter, motor link, shutdown path,
  deskewing, mounting offset) waits.

### Remote control — FlySky FS-i6X / FS-iA6B over iBUS (paused 2026-10-12)

Read before doing remote-control work:

- **`docs/RESUME-remote.md`** — where the work stands and what is next.
- `docs/remote-ibus-protocol.md` — the protocol reference. FlySky publishes
  no spec; every fact there is tagged [ours], [community] or [unverified].
  Do not state community facts as measured, and do not fill gaps from
  general knowledge. §5 (failsafe) is measured: the receiver never stops
  sending on radio loss, so the robot detects it through the arm switch.
- `docs/remote-cpp-design.md` — the library layers and how the robot uses them.
- `docs/remote-ibus-reader.md` — the `ibus_reader` tool and its subcommands.

Remote-only facts:

- The receiver's iBUS SERVO output is on GPIO15 (RXD), `/dev/serial0`
  (-> `ttyAMA0`, the PL011, freed from Bluetooth by provisioning).
  **Receive-only:** nothing writes to that port.
- Robot code links **`heron_remote`** (`RemoteControl`: reader thread,
  `getLatest()`, shaped like `Lidar`). Below it, `heron_ibus_port` (port,
  `IBusReceiver`) and the pure `heron_ibus`. Read `docs/remote-cpp-design.md`
  before changing them.
- No code from the iBUS sources may be copied; IBusBM and Betaflight are
  copyleft. The license table is in the protocol doc, §8.

### Odometry — SparkFun OTOS (PAA5160E1) over I2C (paused 2026-10-18)

Read before doing odometry work:

- **`docs/RESUME-odometry.md`** — where the work stands and what is next.
- `docs/odometry-otos-reference.md` — the sensor reference: wiring,
  register map, start-up, calibration, mounting. Every fact is tagged
  [ours], [community] or [unverified], as in the remote's protocol doc.
- `docs/odometry-cpp-design.md` — the library design (proposed): layers,
  `Odometry`, start-up, the reader thread.

Odometry-only facts:

- The sensor is on the Pi's I2C bus 1 (`/dev/i2c-1`, SDA GPIO2, SCL
  GPIO3), enabled at 100 kHz by provisioning; address 0x17. **3.3 V
  only.** Its behaviour is measured in the reference; check there, and
  do not state it from general knowledge.
- Robot code links **`heron_odometry`** (`Odometry`: reader thread,
  `getLatest()`, shaped like `Lidar`). Below it, `heron_otos_device` (bus,
  start-up) and the pure `heron_otos`, with private headers in
  `src/odometry/`. Read `docs/odometry-cpp-design.md` before changing them.
- No code from the SparkFun sources may be copied without the user's
  approval, even the MIT libraries; the schematic is CC BY-SA. The license
  table is in the reference, §9.

### Motor controller — Arduino, 4 motors, over USB serial (active)

Read before doing motor work:

- **`docs/RESUME-motor.md`** — what is decided, what is open, and the
  first steps.
- `docs/motor-protocol.md` — the Pi-to-Arduino wire format and the
  watchdog. The contract between the two sides: change it only with a new
  protocol version.
- `docs/motor-design.md` — hardware, firmware layers and toolchain, the
  holonomic mixing on the Pi, the three test platforms.
- `docs/motor-cpp-design.md` — `heron_motor`, the Pi's motor library:
  `Motors`, its start-up handshake, and how the robot drives through it.

Motor-only facts:

- The Arduino is `/dev/arduino`, named by USB port 1-1.2 because its CP2102
  is identical to the LiDAR's (`target/udev/`).
- Motor order 1 front-left, 2 front-right, 3 back-right, 4 back-left;
  positive drives the robot forward. A wrong-way motor is fixed in wiring
  or firmware, never on the wire.
- The bench shield's PWR jumper must be off, or the Pi's USB 5 V back-feeds
  the motor supply.
- The firmware is built in a container (`scripts/build_firmware.sh`) and
  flashed through the Pi; nothing for it is installed on the host. Its
  logic (`firmware/motor/lib/`) has no Arduino code and is tested by
  `ctest`, like the Pi's code.
- Robot code links **`heron_motor`** (`Motors`: `start()` blocks until the
  Arduino is configured; the robot's loop calls `drive()` every 20 ms, the
  heartbeat). Read `docs/motor-cpp-design.md` before changing it.
- The firmware holds no rig-specific numbers: after every reset it refuses
  Drive until the Pi sends Configure with the rig's profile
  (`profiles/<rig>.conf`, `docs/rig-profiles.md`). One build per platform
  (`uno-l293d` covers the bench rig and the mini robot).
- The wire format is code in `protocol/motor/`, header-only and compiled
  by both sides. It uses no C++ standard library: the board's compiler has
  none (`CPP_CODING_STANDARD.md`, "Shared code for the AVR").

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
- `profiles/` — one file per physical rig: its measured numbers
  (`docs/rig-profiles.md`), deployed with the C++ binaries.
- `scripts/` — host-side drivers. `target/` — things that run on the Pi.

## Hardware

A Raspberry Pi 3B per rig (the bench rig, the mini robot, later the
full-size robot). Provisioning names each one `heron-` and the last three
bytes of its wired MAC address (`heron-9d9324`), so several can share a
network. Commands write its address as `<pi>`: use the wired link (`eth0`),
not the Wi-Fi one, which is what its mDNS name (`heron-<suffix>.local`)
resolves to. Addresses, user names and access details are
per-developer and stay out of the repository (see "Personal setup" below).
Provisioning sets the hostname and **disables cloud-init**: on later boots it
rebuilt `/etc/hosts` from its cached first-boot user-data, undoing a rename,
and editing `/boot/firmware/user-data` does nothing after first boot. The
network profiles are NetworkManager keyfiles and do not depend on it.
Deployed code and binaries live under `~/heron` on the Pi; provisioning and
deploy scripts need SSH to it, and provisioning needs `sudo` there.

Attached: the RPLIDAR A1M8 (USB), a FlySky FS-iA6B receiver (iBUS on the
GPIO UART) for the FS-i6X remote, and a SparkFun OTOS optical odometry
sensor on I2C. An Arduino motor controller on USB
(`/dev/arduino`), driving 4 motors for a holonomic drive: the bench rig
has four motors and no wheels; the mini robot has mecanum wheels.

GPIO header use, through a HAT that passes the Pi's pins through and adds
spare 5 V and GND connections (the full build guide is `docs/wiring.md`;
keep the two in step):

- **iBUS receiver:** signal to pin 10 (GPIO15, RXD); power from any 5 V,
  ground to any GND (pins 4 and 6 at present). Pin 8 (TXD) stays empty.
- **OTOS:** 3.3 V, SDA (pin 3), SCL (pin 5), GND. **3.3 V only:** the 5 V
  pins 2 and 4 sit beside pins 1 and 3.

## Common commands

```
scripts/provision_pi.sh    <user>@<pi>         # rarely
scripts/deploy_pi.sh       <user>@<pi>         # rsync python/
scripts/deploy_pi.sh       <user>@<pi> probes/lidar_01_port.py
scripts/build_cpp.sh                                    # C++ for the Pi, in a container
scripts/deploy_cpp.sh      <user>@<pi> <binary> [args]
scripts/build_firmware.sh                               # Arduino firmware, in a container
scripts/flash_firmware.sh  <user>@<pi>         # avrdude on the Pi
scripts/set_rig.sh         <user>@<pi> <rig>   # once per Pi: which profile
scripts/fetch_captures.sh  <user>@<pi>         # pull captures back
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
Commit as the developer you are working for, never as anyone else.

**Captures and reports are git-ignored.** So is `docs/vendor/`.

## Personal setup

This file is shared and public. Anything specific to one developer — git
identity, the Pi's address and user, host quirks, how they open pull
requests — goes in `CLAUDE.local.md` at the repository root, which Claude
Code also loads and which is git-ignored. Never put addresses or credentials
in this file.

## Working style

Measure rather than assume, and say which one you did. Several conclusions in
this project were wrong on the first attempt and were corrected by testing
against the hardware — that is the expected workflow, not a failure. When a
claim has not been verified, label it as unverified rather than stating it
confidently.
