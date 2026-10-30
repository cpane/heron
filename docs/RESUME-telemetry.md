# Telemetry workstream — resume point

Last updated 2026-10-28. Started 2026-10-28; nothing built yet.

## Read this first when resuming

**Active, at step 1: the design, for review.** The robot program will
stream what it knows to programs on other computers, live and read-only,
through one telemetry server with a channel per kind of data. The LiDAR is
the first channel; odometry and later sensors follow the same recipe.

| Document | What it holds |
|---|---|
| [`telemetry-design.md`](telemetry-design.md) | The design: decisions, protocol, server, channels, the recipe for a new sensor, tests, steps |
| [`robot-telemetry.md`](robot-telemetry.md) | The rules every part of the robot keeps so telemetry is possible (shared doc) |
| [`lidar-gui.md`](lidar-gui.md) | The existing viewer, which must keep working unchanged (LiDAR workstream's doc) |
| `python/heron/link/wire.py` | The wire format, version 1; every new message is added there |

**Requirements from the user (2026-10-28):** production code, not a bench
tool; a standard pattern for every later sensor, LiDAR first; the existing
viewer (`python/tools/lidar_gui.py`) works as it is; the `robot` channel
must be viewable (`telemetry_watch.py`); both the wired link and Wi-Fi
work, Wi-Fi for demos.

## Decided

D1 to D12 in `telemetry-design.md` §2, as recommended and accepted when
the user said to start: one general server with channels; `wire.py` v1,
additions only; read-only; every channel by default, `subscribe` for
fewer; latest wins per client and channel; encode once; up to 4 clients;
the robot program owns the LiDAR, Express, `--no-lidar`; a `robot` channel
from the start; telemetry failing never stops the robot; our own JSON
writer; SI units for new messages.

## Steps

1. **The design** (`telemetry-design.md`), reviewed. <- here
2. `heron_telemetry_wire`: `JsonWriter`, the command reader,
   `LidarMessages`; `lidar_gui_server` encodes through it.
3. `heron_telemetry`: `Channel`, `Published<T>`, `TelemetryServer`, tests.
4. The robot program (`heron_teleop`): LiDAR, server, `lidar` and `robot`
   channels; `telemetry_watch.py`; the bench check.
5. Odometry, by the recipe, when its workstream resumes.

## Still open

- Authentication and encryption: anyone on the network can watch.
- Recording a session for replay.
- Discovery over mDNS.
- Mounting offsets in the hello, from the profile's `[lidar]` and
  `[odometry]` sections once those exist.

## Things to know at the bench

- The bench Pi runs `heron_teleop` at boot (`scripts/teleop_autostart.sh`,
  `RESUME-motor.md`). Once it serves telemetry it holds port 5555, so
  `lidar_gui_server` and the Python server cannot listen there while it
  runs: turn the service off first, as for `motor_link`.
- Check the LiDAR is still on USB port 1-1.4 (`/dev/rplidar`) before step 4;
  it has not been used since the LiDAR workstream paused.
