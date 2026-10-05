# Heron

Software and target provisioning for the Heron Raspberry Pi platform.

## Development Model

Source code is developed on a host development machine and stored in Git.

The Raspberry Pi is treated as a provisioned execution target, not as a
development workstation.

Target configuration and dependencies must be reproducible from scripts
contained in this repository.

## Languages

- Python — hardware bring-up, diagnostics, experimentation
- C++ — production robot software

## Hardware

- Raspberry Pi 3 Model B
- SLAMTEC RPLIDAR A1M8 over USB — attached, with a working C++ library
- Arduino motor controller over USB serial — planned
- Odometry sensor over I2C — planned, not yet explored
- Remote control interface over GPIO/UART — planned, not yet explored

## Where work stands

**Picking up? Start at [`docs/RESUME.md`](docs/RESUME.md)**, the index of
workstreams. Each sensor or subsystem is explored and built as its own
workstream, with its own resume file, before the robot design that ties them
together.

| Workstream | State |
|---|---|
| LiDAR | reusable C++ library, a test program and a GUI backend, verified on recordings and live; paused 2026-10-04 ([`docs/RESUME-lidar.md`](docs/RESUME-lidar.md)) |
| Odometry, remote control | next to be explored |

C++ in this repository follows [`CPP_CODING_STANDARD.md`](CPP_CODING_STANDARD.md).
The code is MIT-licensed; see [`LICENSE`](LICENSE).

## Target Workflow

The Pi is an execution target. Code is edited on the host, pushed with rsync,
and run over SSH. Nothing is authored on the Pi.

### One-time host setup

Fetch the vendored SDK (a plain clone leaves it empty):

    git submodule update --init --recursive

Install an SSH key on the target so deployment does not prompt for a password
on every connection (each deploy opens two or three):

    ssh-keygen -t ed25519        # skip if you already have a key
    ssh-copy-id cpane@<pi>

Install host-side analysis dependencies:

    python3 -m venv .venv
    .venv/bin/pip install -r requirements-host.txt

### Provision the target (rarely)

    scripts/provision_pi.sh cpane@<pi>

Installs packages, network policy, and the udev rules in `target/udev/`.
Log out and back in afterwards for `dialout` membership to take effect.

### Deploy and run (constantly)

    scripts/deploy_pi.sh cpane@<pi>
    scripts/deploy_pi.sh cpane@<pi> probes/lidar_01_port.py

`python/` is mirrored to `~/heron/python` on the target. The mirror is
exact: anything deleted from the repository is deleted from the target.

### Build and run the C++ (see `docs/cpp-build.md`)

    scripts/build_cpp.sh                                    # aarch64, in a container
    scripts/deploy_cpp.sh cpane@<pi> lidar_info

`build_cpp.sh` builds inside a Debian trixie arm64 container so the binary
matches the Pi's glibc and gcc. Requires Docker. For tests that need no
hardware, build natively and run the test suites:

    cmake --preset host && cmake --build --preset host
    ctest --test-dir build-host

### Retrieve captured data

    scripts/fetch_captures.sh cpane@<pi>

Probes write to `~/heron/captures` on the target (exported as
`HERON_CAPTURE_DIR`); this pulls them into `captures/`, which Git ignores.

### Devices

Refer to `/dev/rplidar`, never `/dev/ttyUSB0`. `ttyUSB` numbering is assigned
in enumeration order and changes as soon as a second USB serial device — such
as the Arduino motor controller — is attached.

### Target dependencies

Python dependencies on the target come from apt, declared in the install list
in `target/provision/base.sh`. Debian marks the system interpreter
EXTERNALLY-MANAGED (PEP 668), so `pip3 install` into it is blocked by design.

If a dependency is ever needed that Debian does not package, create a venv
that still inherits the apt packages rather than replacing them:

    python3 -m venv --system-site-packages ~/heron/.venv

## LiDAR (RPLIDAR A1M8)

### The C++ library and programs

A reusable library, `heron_lidar`: robot code includes `Lidar.h`,
`Scan.h` and `SectorCheck.h` and never sees the vendor SDK. It publishes one
robot-frame scan per revolution, with coverage, unobserved arcs and overrun
detection, and is verified point for point against the Python decoder.

| Program | What it does | Run it |
|---|---|---|
| `lidar_nearest` | nearest obstacle in a sector: CLEAR, STOP or UNKNOWN with the reason | `scripts/deploy_cpp.sh cpane@<pi> lidar_nearest --arc 90` |
| `lidar_gui_server` | serves the library's scans to the viewer below | `scripts/deploy_cpp.sh cpane@<pi> lidar_gui_server` |
| `lidar_adapter` | the library's health, once a second | `scripts/deploy_cpp.sh cpane@<pi> lidar_adapter 20` |
| `lidar_record` | records an SDK session to `.rpraw`, or replays one with `--replay` | `scripts/deploy_cpp.sh cpane@<pi> lidar_record --mode 1 --scans 40 --tag desk` |
| `lidar_info` | device identity, health and scan modes, motor off | `scripts/deploy_cpp.sh cpane@<pi> lidar_info` |

`lidar_nearest`, `lidar_gui_server` and `lidar_record` also run on the host against a
recording, with `--replay` (build with `cmake --preset host`, binaries in `build-host/bin`).

- [`docs/lidar-cpp-design.md`](docs/lidar-cpp-design.md) — how the library is
  structured around the vendor SDK, and why.
- [`docs/cpp-build.md`](docs/cpp-build.md) — the two build configurations, the
  submodule and its one local patch.
- [`docs/lidar-gui.md`](docs/lidar-gui.md) — the live viewer, with either the
  Python server (raw sensor frame) or the C++ one (robot frame).
- [`docs/rplidar-a1m8-findings.md`](docs/rplidar-a1m8-findings.md) — what the
  sensor actually does, measured on this unit.

### The Python evaluation

**New here? Read [`docs/lidar-start-here.md`](docs/lidar-start-here.md)** — a
40-minute guided walkthrough of the probes, in order, with what to look for in
each. [`docs/lidar-references.md`](docs/lidar-references.md) indexes SLAMTEC's
official protocol specification by page number.

Exploratory probes for the RPLIDAR A1M8, under `python/probes/`. They taught
the sensor's behaviour before the C++ library was written, and remain the
quickest way to look at the wire. Measured results, and the protocol traps they
uncovered, are in [`docs/rplidar-a1m8-findings.md`](docs/rplidar-a1m8-findings.md).

Run each with `scripts/deploy_pi.sh cpane@<pi> probes/<name>.py`.

| Probe | What it shows |
|---|---|
| `lidar_01_port.py` | Port, DTR/motor control (audible), device identity, health, sample rate. `--reset` shows RESET's ASCII banner. |
| `lidar_02_wire.py` | The scan stream as annotated bytes: descriptor bit fields, five-byte nodes with the arithmetic spelled out. `--desync N` forces a resync. |
| `lidar_03_revolution.py` | Revolution assembly, angular spacing, and that the stream is not angle-sorted. `--bearing-test` locates 0°. |
| `lidar_04_capture.py` | Sustained capture to `.rpraw` + `.jsonl`, with full statistics. `--express` for express mode. |
| `lidar_05_express.py` | Legacy vs express A/B on the same scene, with annotated capsules. |
| `lidar_06_capabilities.py` | `GET_LIDAR_CONF` scan-mode table (this unit reports 5 modes) and `FORCE_SCAN` with the motor stopped. |

Host-side, under `python/tools/` (needs `requirements-host.txt`):

    .venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
    .venv/bin/python python/tools/lidar_report.py captures/<name>.jsonl
    python3 python/tools/lidar_rawcat.py captures/<name>.rpraw --verify

`lidar_gui.py` is the live viewer. Start a server on the Pi first, either
`server/lidar_server.py` or the C++ `lidar_gui_server` — see
[`docs/lidar-gui.md`](docs/lidar-gui.md). It needs no host
dependencies beyond the standard library; Tkinter ships with Python.

`lidar_report.py` writes polar, coverage, rate, quality and range figures to
`reports/<capture_id>/`. `lidar_rawcat.py --verify` re-decodes the raw wire log
and checks it reproduces the recorded output exactly. The C++ is held to the
same standard: `lidar_sdkcheck.py`, `lidar_gapcheck.py` and
`lidar_adaptercheck.py` compare the SDK, the gap detector and the whole
library against the Python decoder on the same recordings.

### Capture format

Each run writes two sibling files. `.rpraw` is the byte-exact wire log (a JSON
header line, then 20-byte-header chunk records); it is ground truth and can be
replayed with `--replay` by any probe, with no hardware attached. `.jsonl` is
one JSON object per line — header, command, descriptor, sample, revolution,
event, footer. Nothing is filtered: invalid returns, resyncs and throttling
events are all recorded. `lidar_record` writes the same `.rpraw` format from
inside the SDK, which is how the C++ is tested offline.
