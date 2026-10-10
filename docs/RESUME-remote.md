# Remote control workstream — resume point

Last updated 2026-10-10. Branch **`feature/ibus-reader`**.

## Read this first when resuming

The remote is a FlySky **FS-i6X** transmitter and an **FS-iA6B** receiver.
The receiver's iBUS SERVO output is wired to the Pi's GPIO UART
(`/dev/serial0`). The aim of this stage is to understand the link and build a
reusable reader, as the LiDAR workstream did. It is not robot control yet.

| Piece | Where | State |
|---|---|---|
| Protocol reference | [`remote-ibus-protocol.md`](remote-ibus-protocol.md) | written from sources; §4 map and §5 failsafe await hardware runs |
| `heron_ibus`: parser, stats, `LinkMonitor`, normalization | `cpp/include/heron/remote/`, `cpp/src/remote/` | unit tested; decodes live frames on the Pi |
| `heron_ibus_port`: receive-only termios port | same | verified on the Pi |
| `ibus_reader` | `cpp/tools/IBusReader.cpp`; [`remote-ibus-reader.md`](remote-ibus-reader.md) | being extended with subcommands |

Build and deploy as usual:

```
scripts/build_cpp.sh
scripts/deploy_cpp.sh <user>@<pi> ibus_reader
```

## Picking up

Being built now: the `ibus_reader` subcommands (monitor, dump, identify,
record, failsafe-test, stats) and a fake receiver on a pty for testing them
on the host.

## Verified so far

- 2026-10-10: the refactored `ibus_reader` decodes live frames from the
  receiver on the Pi (CH1-6 read 1502 1500 1039 1500 2000 1917 at rest).
  `ibus_parser_test` and `normalize_test` pass on the host and on the Pi.
