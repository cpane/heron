# Dioptra workstream — resume point

Last updated 2026-11-19. Started 2026-11-19; nothing built. Gathering
requirements: the build starts after the Maker Faire (2026-11-21/22).

## What it is

Dioptra is a host application with a GUI, and the builder's main tool for
configuring and controlling Heron robots. A builder builds a robot,
images its SD card and knows its address. From there Dioptra does the
rest, without the builder running any scripts by hand:

- **Set up** a new robot: provision it, set its rig, deploy.
- **Many robots:** each one is listed with its state. Later Dioptra
  finds them on the network itself, and the builder picks which robot to
  work with.
- **Monitor** a robot's telemetry live.
- **Configure** it: which floor it is on, the boot service, its profile.
- **Calibrate** it: add a new surface and measure its motion table,
  guided step by step, plus the other measurements in
  `docs/rig-calibration.md`.

**The name.** Heron is named after Hero (Greek *Hērōn*) of Alexandria, the
1st-century engineer whose book on automata describes a cart programmed
with cord wound on its axle. His *dioptra* was a surveying instrument
for measuring angles and distances and lining things up. This app does
the same job for the robots.

## Decided

- **2026-11-19 (Chris):** a workstream of its own, named Dioptra. It is
  where builder work is going: when Dioptra covers a task, the builder
  scripts are the terminal fallback for it, not the way builders work.
  Builder has an Inbox entry about this.
- **2026-11-19 (recommended; accepted when Chris said to set up the
  workstream):** nothing is built before the Faire. The Faire
  runs on the scripts and `docs/navigation-faire-checklist.md`. Notes
  taken there ("I wish I could just click this") are the first
  requirements.

## Rules (from builder's, `RESUME-builder.md`)

- **The robot side belongs to the other workstreams.** Dioptra uses what
  the robot already offers. A missing capability, such as a status query
  or a telemetry channel, is a change in the owning workstream first, on
  its own branch. Dioptra records here that it is waiting on it.
- **The scripts stay the source of truth** for each task until Dioptra
  has its own tested way of doing it: nothing is written twice before it
  has to be. Calling a script from Dioptra counts.
- **Plain words.** Name things as a builder sees them (front-left wheel,
  SwA). Every failure says what to do next. Expect slips: offer a retry
  before blaming the hardware.
- **Leave a robot as Dioptra found it:** a service stopped for a check is
  offered back.

## What each part depends on

| Part | Depends on |
|---|---|
| Provision, rig, deploy | `scripts/provision_pi.sh`, `set_rig.sh`, `build_cpp.sh`, `deploy_cpp.sh`: exist. Provisioning needs the builder's sudo password on the Pi |
| Floor, boot service, logs, status | `set_surface.sh`, `robot_autostart.sh`, `fetch_captures.sh`: exist. No status query exists yet |
| A new surface and its motion table | the Faire checklist's steps 2 to 5; `python/tools/motion_points.py` turns a log into table lines: exist, run by hand |
| Live telemetry | the telemetry workstream: design written, not reviewed, nothing built (`RESUME-telemetry.md`) |
| Finding robots | each Pi announces `heron-<suffix>.local` over mDNS [unverified: whether that is enough on an unfamiliar network, such as a phone hotspot] |

## Still open

- **Technology:** Tk (the host `.venv` has it), a local web app, or
  something else. Which platforms builders use (only macOS so far).
- **Where it lives** in the repository, and how it is tested without a
  robot.
- **The build:** today the C++ is built in a container on the host. Does
  each builder build, or does Dioptra deploy binaries built for them?
- **First contact:** Raspberry Pi Imager's settings (user, SSH, Wi-Fi)
  come before Dioptra. Can Dioptra take over more of that step?
- **Access:** SSH keys and sudo prompts, from a GUI.
- **Several robots at once:** what Dioptra does in parallel and what it
  does one robot at a time.

## Picking up

1. **At the Faire (the user):** note every point where a click would have
   beaten a command, and every step that was confusing.
2. **Requirements** (`docs/dioptra-requirements.md`): the scope above,
   the Faire notes, and the builder's tasks in order, from an imaged SD
   card to a calibrated robot on a new floor.
3. **Design** (`docs/dioptra-design.md`): the technology, the
   structure, and the first version's scope. Settle "Still open" here.

## Inbox

Facts other workstreams found that belong here, waiting to be checked and
folded into this workstream's docs when it resumes (`CLAUDE.md`,
"Workstreams"). Newest last; delete an entry once folded in.
