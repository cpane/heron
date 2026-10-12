# Remote control workstream — resume point

Last updated 2026-10-12. Everything is merged into `develop`.

## Read this first when resuming

**Paused 2026-10-12** at a clean stopping point, to start odometry. The
remote is understood, and there is a reusable C++ library for it, verified on
the hardware, including how the robot detects a lost radio.

The remote is a FlySky **FS-i6X** transmitter and an **FS-iA6B** receiver.
The receiver's iBUS SERVO output is wired to the Pi's GPIO UART
(`/dev/serial0`). Scope was the link and a reusable reader, not robot
control: control assignments and the arming rule wait for the robot design.

| Piece | Where | State |
|---|---|---|
| Protocol reference | [`remote-ibus-protocol.md`](remote-ibus-protocol.md) | sourced and tagged; timing, channel map (§4), failsafe (§5) and deadband (§7) measured |
| `heron_ibus`: parser, stats, `LinkMonitor`, normalization, `isFailsafe()` | `cpp/include/heron/remote/`, `cpp/src/remote/` | unit tested; decodes live frames on the Pi |
| `heron_ibus_port`: receive-only termios port, `IBusReceiver` | same | verified on the Pi |
| **`heron_remote`: `RemoteControl`**, what robot code links | same; [`remote-cpp-design.md`](remote-cpp-design.md) (examples in §2) | pty test on the host; live on the Pi, including failsafe |
| `ibus_reader`: monitor, dump, identify, record, failsafe-test, stats | `cpp/tools/IBusReader.cpp`; [`remote-ibus-reader.md`](remote-ibus-reader.md) | all six run live on the Pi |
| `remote_adapter`: `RemoteControl` as a 50 Hz loop sees it | `cpp/probes/RemoteAdapter.cpp` | live on the Pi |
| `ibus_fake.py`: fake receiver on a pty | `python/tools/` | host testing |

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh <user>@<pi> ibus_reader monitor
scripts/deploy_cpp.sh <user>@<pi> remote_adapter 30
```

## What the robot must know

- **Stop rule:** `RemoteSnapshot::isOk()` is false when the link is stale
  or the failsafe channel reads its failsafe value. Configure
  `FailsafeConfig{6, 1000, 50}` (CH7 = SwA, 1000 = disarmed).
- **The receiver never stops sending on radio loss.** Failsafe values arrive
  about 1 s later (1027–1028 ms, three runs). Until then it repeats the last
  sticks: allow for that blind second.
- **The link comes back disarmed:** the i6X will not transmit until SwA is
  up and the throttle down.
- **Still the robot's:** which sticks drive and turn, a speed ceiling, a
  mode switch, and the arming rule (see disarmed, then armed, before
  driving).

## Transmitter configuration (on the i6X, not in this repository)

Needed for the above. If the transmitter is reset or a new model is set up,
redo it and re-run `identify` and `failsafe-test`:

- **10 channels:** System setup › Aux switches, switches on, Ch = 10.
- **Switches on channels:** Functions setup › Aux channels, CH7 SwA, CH8
  SwB, CH9 SwC, CH10 SwD.
- **Failsafe:** System setup › RX setup › Failsafe, CH7 −100% (SwA up) and
  CH3 −100%; every other channel Off. UP/DOWN turns a channel on; *hold*
  CANCEL to save. Run 3 found these settings lost once, so check them.

## Picking up: optional, none blocks the robot

1. **Run the test suites on the Pi.** `failsafe_test` and the failsafe
   checks in `remote_control_test` have run on the host and cross-built
   for the Pi, but not run there.
2. **Out of range:** `failsafe-test` with `r`, outdoors with a laptop. Off
   was tested; out of range probably behaves the same.
3. **Stop bits on the wire** (protocol §2): needs a scope. One stop bit
   works with zero errors.
4. **Frame timing under load:** `ibus_reader stats` with the LiDAR and the
   robot code running.
5. **The high nibble** has never been seen set, at rest or in failsafe.

## Verified so far

On the Pi with the live receiver, 2026-10-10 to 2026-10-12 (details in the
protocol doc):

- `stats --seconds 30`: 129.9 Hz, interval 7.70 ms (std dev 0.01 ms), 0
  checksum errors.
- `identify`: sticks CH1–CH4, knobs CH5–CH6, switches CH7–CH10. SwA 1000
  up, 2000 down; SwC 1000/1500/2000.
- `failsafe-test`, five runs, transmitter off: frames never stop; failsafe
  Off freezes the values; set values arrive after about 1 s, with no flag.
- `remote_adapter 40`: SwA up and transmitter off each read as not OK while
  the link stayed OK; 0 stale events, 0 checksum errors.
- Deadband: sticks at rest do not move by one count in 60 s and return to
  within 1 count after release; recommended 10.
- The four remote test suites pass on the host. `ibus_parser_test`,
  `normalize_test` and the pre-failsafe `remote_control_test` also passed
  on the Pi.

**Not verified:** failsafe out of range, 8N1 vs 8N2 on the wire.
