# The live viewer — server on the Pi, GUI on the host

A client/server pair for looking at the sensor in real time. The Pi owns the
hardware; the host draws it.

    python/server/lidar_server.py   on the Pi, owns the LiDAR, serves TCP
    cpp/tools/LidarGuiServer.cpp    the same, but serving the C++ adapter
    python/tools/lidar_gui.py       on the host, Tkinter, draws the scan
    python/heron/link/wire.py       the message format both ends share

Two servers, one viewer. The Python server shows the raw sensor, in the
sensor's own frame. The C++ server (`lidar_gui_server`) shows what the C++
adapter publishes to the robot: robot frame, coverage, unobserved arcs.

## Running it

Two terminals. First, the server — it stays in the foreground so Ctrl-C
reaches it and the motor stops:

```
scripts/deploy_pi.sh <user>@<pi> server/lidar_server.py
```

Then the GUI:

```
.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
```

`--connect` skips the Connect button. Both ends default to TCP 5555.

To view the **C++ adapter** instead, start its server (same port, same GUI):

```
scripts/deploy_cpp.sh <user>@<pi> lidar_gui_server
```

Or with no Pi at all, from a recording, both on the host:

```
build-host/bin/lidar_gui_server --replay captures/<stem>.rpraw
.venv/bin/python python/tools/lidar_gui.py --host 127.0.0.1 --connect
```

Differences with the C++ server, all by design of the adapter:

- **Robot frame.** Bearings arrive in REP 103: 0 still points up, but
  positive is to the **left**. The caption under the plot says which frame
  is on screen.
- **Unobserved arcs** are shaded, and the stats panel adds coverage,
  unobserved arcs and estimated dropped samples.
- **The motor runs only while scanning.** Motor-on without a scan is
  refused with an explanation; Reset stops instead. The adapter has no
  separate motor control (lidar-cpp-design.md §2.5).
- **All five modes are offered**, because the SDK decodes them all; the three
  ultra modes are labelled "decode unchecked". A replay offers only the mode
  it recorded.
- **Device identity appears once a scan starts.** The adapter reads it at
  start, at no extra cost, and the server sends a second hello.

The server is started on demand rather than run as a service, deliberately:
`deploy_pi.sh` rsyncs before it runs, so the code executing on the Pi is
always the code in the working tree. A long-lived systemd unit would run
whatever was deployed last, which is the staleness this repository is built to
avoid.

## What the controls do

With the Python server. The C++ server's differences are listed above.

| Control | Effect |
|---|---|
| **Motor** | `set_motor`. Turning it off stops a running scan and *remembers* it; turning it back on spins up, settles, and resumes that scan. |
| **Scan mode** | The five modes from `GET_LIDAR_CONF`. See the caveat below. |
| **Start / Stop scan** | Toggles the stream. Takes effect within ~one revolution. |
| **Reset** | `RESET`, then re-queries the device. |
| **Range filter** | Draws only returns within the distance, *and* rescales the plot so that distance fills the canvas. |

Changing the scan mode while scanning restarts the stream in the new mode
rather than making you stop and start, because comparing modes is the point.

**Stopping does not clear the plot.** The last revolution stays on screen,
because it is usually worth looking at — but it is drawn in a dead grey with a
`NOT LIVE -- stream stopped` banner, and the Stream panel is prefixed
`STOPPED`. Nothing on a stopped plot should ever read as the current state of
the room.

An explicit **Stop scan** is treated as a decision, so it clears the pending
resume: turning the motor off and on afterwards will not restart a scan you
deliberately stopped.

## Which modes render

With the **C++ server** all five render, because the SDK decodes them; the
three ultra modes are labelled "decode unchecked". With the **Python
server**, which decodes only legacy and express, two produce points:

| Mode | Streams | Viewer |
|---|---|---|
| 0 Standard | legacy 0x81 | points |
| 1 Express | express 0x82 | points |
| 2 Boost | ultra 0x84 | statistics only |
| 3 Sensitivity ★ | ultra 0x84 | statistics only |
| 4 Stability | ultra 0x84 | statistics only |

★ is the mode the device itself recommends, and it is one of the three nothing
here decodes. Selecting an ultra mode still starts the stream — the Stream
panel shows frame count, frame rate and byte rate, and the plot says plainly
that there is no decoder rather than drawing an empty circle you might read as
"the sensor sees nothing". Background in
[`rplidar-a1m8-findings.md`](rplidar-a1m8-findings.md).

The GUI defaults to the server's announced mode when it can render it, and
otherwise to the fastest mode that renders, so a fresh launch always shows
something: Express with the C++ server, Express with the Python server too
(it announces mode 3, which it cannot render).

## Orientation

**It depends on the server, and the caption in the plot's bottom-left corner
always says which.** With the Python server, 0° is drawn straight up and the
angle increases clockwise: the sensor's own frame, as measured on this unit,
which is left-handed. With the C++ server the plot is in the robot frame,
REP 103: 0° still points up, but bearings increase anticlockwise, so positive
is to the left. The two views of the same scene are mirror images. Checked by
eye on 2026-10-04: an object on the sensor's left draws on the left with the
C++ server.

## The wire format

Newline-delimited JSON over TCP, no dependency on either end. That makes
`nc <pi> 5555` a working diagnostic client:

```
{"cmd":"hello"}
{"cmd":"motor","on":true}
{"cmd":"scan_start","mode":1}
{"cmd":"scan_stop"}
```

Angles and distances travel in the sensor's own fixed point — `angle_q6` is
1/64°, `dist_q2` is 1/4 mm — so the numbers on the wire are the numbers in the
protocol spec, and nothing is rounded before it needs to be.

Only valid returns are sent; the *count* of everything survives as `total`,
because "nothing at 90°" and "did not look at 90°" are different facts.

One client at a time. A second connection is told why and closed.

## What is verified, and what is not

**Ran against the real sensor 2026-09-27.** It connects, drives the motor,
streams, and renders. The first hardware run found two bugs -- a stopped
stream kept drawing its last revolution as though it were live, and the motor
toggle stopped the scan without being able to restart it.

**Both fixes are now hardware-verified**, not just stub-verified. Driving the
reported sequence against the Pi and the actual sensor -- scan, motor off,
motor on -- the stream came back on its own: 19 revolutions before the motor
went off, none while it was off, 34 after it came back, all carrying real
geometry (357 valid points of 535 samples, nearest return 270 mm). A
deliberate Stop scan followed by a motor cycle correctly did *not* restart.
13 checks, all against hardware.

Verified on the host against a stub: the wire format, the polar transform and
range filter, mode handling, stale-frame marking, the accept loop,
single-client lock, command queue and teardown. 46 checks.

**Since then (2026-10-01):** all five modes were cycled on the hardware. The
ultra modes stream genuine 132-byte 0x84 frames, which the Python server
counts. No `NOTE: mode N advertised ...` line appeared, so `EXPRESS_SCAN` with
`working_mode` 1 to 4 answered with the type the mode table advertises. A
silent stream (mode 4, 2 of 11 attempts) used to hang the server's worker; it
can now be stopped.

**The C++ server (2026-10-04):** protocol checked on a replay and live from
the Pi (46 scans in 8 s, no seq gaps, coverage ok, about 4.3 KB per message);
start, stop, unknown commands and disconnect checked; the drawing checked by
eye.

Still unverified:

- **The `working_mode` mapping, in part.** `working_mode` 0 under
  `EXPRESS_SCAN` was not exercised (the server starts mode 0 with `SCAN`), and
  modes 2 to 4 all answer 0x84, so the descriptor cannot show whether they are
  permuted among themselves.
- **The shaded unobserved arcs** in the C++ view have never been drawn from
  real data: every lossy scan recorded so far arrived in a post-stall burst,
  which the adapter drops.
