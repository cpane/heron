# Remote control workstream — resume point

Last updated 2026-10-11. Branch **`feature/ibus-reader`**.

## Read this first when resuming

The remote is a FlySky **FS-i6X** transmitter and an **FS-iA6B** receiver.
The receiver's iBUS SERVO output is wired to the Pi's GPIO UART
(`/dev/serial0`). The aim of this stage is to understand the link and build a
reusable reader, as the LiDAR workstream did. It is not robot control yet.

| Piece | Where | State |
|---|---|---|
| Protocol reference | [`remote-ibus-protocol.md`](remote-ibus-protocol.md) | sourced and cross-checked; timing measured; **§4 map and §5 failsafe await hardware runs** |
| `heron_ibus`: parser, stats, `LinkMonitor`, normalization | `cpp/include/heron/remote/`, `cpp/src/remote/` | unit tested; decodes live frames on the Pi |
| `heron_ibus_port`: receive-only termios port, `IBusReceiver` | same | verified on the Pi |
| **`heron_remote`: `RemoteControl`**, what robot code links | same; [`remote-cpp-design.md`](remote-cpp-design.md) | pty test on host and Pi; `remote_adapter` live on the Pi |
| `ibus_reader`: monitor, dump, identify, record, failsafe-test, stats | `cpp/tools/IBusReader.cpp`; [`remote-ibus-reader.md`](remote-ibus-reader.md) | all six tested against the fake; monitor, dump, record and stats also run live on the Pi |
| `ibus_fake.py`: fake receiver on a pty | `python/tools/` | host testing |

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh <user>@<pi> ibus_reader monitor
```

## Picking up: two hardware runs, with someone at the transmitter

1. **`failsafe-test`**: run 1 is done (2026-10-11, every channel Off,
   transmitter off): **FROZEN**. Frames kept coming at 129.9 Hz with the
   last values, so `LinkMonitor` alone **cannot** detect radio loss. Still
   to run: a failsafe value set on a channel (FS-i6X: System setup › RX
   setup › Failsafe; set the value, then *hold* CANCEL to save, a short
   press discards it), and out of range (`r`). Paste each block into
   protocol §5.
2. **`identify`**, every control on the i6X (both sticks on both axes, the
   switches, VrA and VrB). Paste the table into protocol §4. Decide whether
   to set the i6X to 10 channels.

Then add failsafe detection to the library as `remote-cpp-design.md` §5
describes, shaped by what the test showed. After that, settle §7's control
assignment (drive, turn, speed ceiling, arm, mode) from the map, and tune the
deadband from `monitor`'s resting jitter.

## Verified so far

All on 2026-10-10, on the Pi with the live receiver:

- The refactored reader decodes live frames. `ibus_parser_test` and
  `normalize_test` pass on the host and on the Pi.
- `dump` captured the frame used as protocol §3's worked example. The parser
  test decodes it.
- `stats --seconds 30`: 129.9 Hz, interval 7.70 ms (7.66–7.73, std dev
  0.01), 0 checksum errors. Every byte is accounted for by channel frames
  (protocol §2).
- `monitor` and `record` run against the live receiver.
- `RemoteControl` (`remote_adapter 5`): 250 polls at 50 Hz, link OK at every
  one, 130 frames/s, no checksum errors. `remote_control_test` passes on the
  Pi.

- `failsafe-test` run 1, failsafe all Off, transmitter off: frames and
  values continued unchanged (protocol §5).

**Not verified:** failsafe with a value set and out of range (§5), the channel map (§4), 8N1 vs 8N2
on the wire (§2, needs a scope), and the high nibble outside rest.
