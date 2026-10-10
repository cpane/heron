# Remote control workstream — resume point

Last updated 2026-10-10. Branch **`feature/ibus-reader`**.

## Read this first when resuming

The remote is a FlySky **FS-i6X** transmitter and an **FS-iA6B** receiver.
The receiver's iBUS SERVO output is wired to the Pi's GPIO UART
(`/dev/serial0`). The aim of this stage is to understand the link and build a
reusable reader, as the LiDAR workstream did. It is not robot control yet.

| Piece | Where | State |
|---|---|---|
| Protocol reference | [`remote-ibus-protocol.md`](remote-ibus-protocol.md) | sourced and cross-checked; timing measured; **§4 map and §5 failsafe await hardware runs** |
| `heron_ibus`: parser, stats, `LinkMonitor`, normalization | `cpp/include/heron/remote/`, `cpp/src/remote/` | unit tested; decodes live frames on the Pi |
| `heron_ibus_port`: receive-only termios port | same | verified on the Pi |
| `ibus_reader`: monitor, dump, identify, record, failsafe-test, stats | `cpp/tools/IBusReader.cpp`; [`remote-ibus-reader.md`](remote-ibus-reader.md) | all six tested against the fake; monitor, dump, record and stats also run live on the Pi |
| `ibus_fake.py`: fake receiver on a pty | `python/tools/` | host testing |

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh <user>@<pi> ibus_reader monitor
```

## Picking up: two hardware runs, with someone at the transmitter

1. **`failsafe-test`**, at least twice: once with every channel's failsafe
   Off, and once with a failsafe value set. Use both `o` (switch off) and
   `r` (out of range) if practical. Paste each result block into protocol §5
   and replace the NOT YET VERIFIED callout with what was measured.
   **Every source says the iA6B keeps sending frames on radio loss**, which
   would make the stale rule blind to it. This run decides whether the
   robot's stop logic can rely on `LinkMonitor` alone.
2. **`identify`**, every control on the i6X (both sticks on both axes, the
   switches, VrA and VrB). Paste the table into protocol §4. Decide whether
   to set the i6X to 10 channels.

After those: settle §7's control assignment (drive, turn, speed ceiling, arm,
mode) from the map, and tune the deadband from `monitor`'s resting jitter.

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

**Not verified:** failsafe behaviour (§5), the channel map (§4), 8N1 vs 8N2
on the wire (§2, needs a scope), and the high nibble outside rest.
