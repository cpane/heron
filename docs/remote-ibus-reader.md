# `ibus_reader` — the iBUS learning and debugging tool

A bench tool for the remote control link: the FS-iA6B receiver's iBUS SERVO
output on the Pi's GPIO UART. It prints, so it is a tool, not a library. The
reusable parts (`heron_ibus`, `heron_ibus_port`) do not print, and are what
robot code will link. The protocol is in
[`remote-ibus-protocol.md`](remote-ibus-protocol.md).

Source: `cpp/tools/IBusReader.cpp`. It never writes to the serial port: the
device is opened read-only.

## Running it on the Pi

The normal flow, nothing special:

```
scripts/build_cpp.sh                                   # build for the Pi, in the container
scripts/deploy_cpp.sh <user>@<pi> ibus_reader monitor  # deploy, then run
scripts/deploy_cpp.sh <user>@<pi> ibus_reader stats --seconds 30
```

`deploy_cpp.sh` runs it under `ssh -t`, so `monitor`'s keys and the
interactive prompts of `identify` and `failsafe-test` work. Captures from
`record` land in `~/heron/captures/` on the Pi; bring them back with
`scripts/fetch_captures.sh <user>@<pi>`.

## Common options

| Option | Default | Meaning |
|---|---|---|
| `--device PATH` | `/dev/serial0` | serial device |
| `--stale-ms N` | 100 | no valid frame for N ms means the link is lost. One rule, `LinkMonitor`, in every command |
| `--stop-bits N` | 1 | 1 or 2; see protocol §2 for why 1 |
| `--baud N` | 115200 | |
| `--seconds S` | until stopped | stop after S seconds (`stats` defaults to 10) |

Ctrl-C stops any command cleanly: the terminal is restored and the port
closed.

## Subcommands

### `monitor`

A live display of all 14 channels, redrawn 10 times a second. Each row shows
the raw value, a bar from 1000 to 2000 filled from the 1500 midpoint, and the
min/max since start (or since `r`). A non-zero high nibble is flagged as
`!0x2` and so on. The header shows the link state (OK, STALE with the time
since the last frame, or NO SIGNAL with a hint), the frame rate over the last
second, and the error counters. **`r`** resets min/max, **`q`** or Ctrl-C quits.

Use it to see what each control does and how much a resting stick jitters
(that sets the deadband, protocol §7).

### `dump`

One line per valid packet: a millisecond timestamp, the 32 bytes in hex, then
the 14 decoded channels.

```
ibus_reader dump --count 5               # five packets, then stop
ibus_reader dump --decimate 130          # about one a second
```

Totals go to stderr, so `dump > file.txt` keeps only data.

### `identify`

Guided channel discovery. For each control: type a name, press Enter, then
move only that control end to end within the window (`--window S`, default
4). It reports `CHn moved (min..max)`. Any other channel that also moved more
than 100 counts is listed as well, since a mix or a second control moving
shows up that way. Enter `q` to finish. The map is printed as the table for
protocol §4. `--out FILE` also saves it.

### `record`

Writes every valid frame to CSV: `t_ms` (monotonic, from the start of the
recording) and `ch1`..`ch14`, one header row. The default path is
`captures/ibus-<date>-<time>.csv` relative to where it runs, which is
`~/heron` under `deploy_cpp.sh`. Use `--out` for a different path, and
`--seconds` to stop on its own.

### `failsafe-test`

Measures what the link does when the transmitter goes away. **It needs you at
the transmitter**, and its result goes into protocol §5. The robot's stop
logic depends on that section.

1. Set the failsafe to test on the FS-i6X. Run it at least twice: once with
   every channel's failsafe **Off**, and once with a **value** set.
2. Flip a switch off its usual position and turn a knob off-centre. Then a
   receiver that holds the last values can be told apart from one that goes
   to failsafe values.
3. Follow the prompts. Describe the failsafe setting, choose `o` (switch off)
   or `r` (carry it out of range), wait for the 2 s baseline, then **press
   Enter and switch the transmitter off at the same moment**.
4. After the watch window (`--watch S`, default 10) it asks you to switch the
   transmitter back on and measures recovery.

It classifies the result as one of these:

| Verdict | Meaning for the robot |
|---|---|
| STOPS | the stale rule detects the loss |
| CONTINUES WITH THE LAST VALUES (FROZEN) | nothing in the data shows the loss. Unsafe as configured |
| CONTINUES WITH FAILSAFE VALUES | detectable only by recognising the failsafe values |
| ... FLAGGED IN THE HIGH NIBBLE | detectable by the flag as well |

It then prints a block between `----- paste into ... -----` and
`----- end -----` to paste into protocol §5. Times are measured from the
Enter key, so allow a few hundred milliseconds of hand timing.

### `stats`

Runs quietly for `--seconds` (default 10), then summarizes:

- bytes received and frames decoded, with the rate
- frame interval min, mean, max and standard deviation, with a 1 ms histogram
- checksum errors, and discarded bytes with the number of resyncs
- stale events
- any channel whose high nibble was set

## When it shows nothing

- **Device missing:** it says so, and points to `enable_uart=1` and
  `dtoverlay=disable-bt` (set by `target/provision/base.sh`; needs a reboot).
- **Permission denied:** add the user to `dialout`.
- **No valid frame within 2 s:** it says whether any bytes arrived at all,
  and lists what to check for each case:
  - no bytes: the transmitter is on, its output mode is i-BUS, the wiring is
    right, the UART is enabled and the serial console is off
  - bytes but no frames: the wire is on SERVO, not SENS; the mode is not
    S.BUS; the line settings are right

## Testing without hardware

`python/tools/ibus_fake.py` (standard library only) makes a pseudo-terminal
and streams synthetic frames every 7 ms. The host-native build can read it,
because macOS has termios, `poll()` and ptys. The cross-compiled binary
cannot run on the host, hence `build-host/`.

```
cmake --preset host && cmake --build --preset host
python3 python/tools/ibus_fake.py --link /tmp/ibus --move 4@2-5 --corrupt 0.02 &
build-host/bin/ibus_reader monitor --device /tmp/ibus
```

It can move channels on a schedule (for `identify`), corrupt frames, insert
junk, split writes, and lose the link in each of the four failsafe styles
(`--loss-mode stop|hold|values|flag`). `python3 python/tools/ibus_fake.py
--help` lists everything. A pty ignores baud rate and stop bits, so line
settings can only be tested on the Pi.

The unit tests (`ibus_parser_test`, `normalize_test`) run under `ctest`, on
the host or on the Pi.
