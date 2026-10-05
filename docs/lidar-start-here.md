# Start here — learning the RPLIDAR A1M8

This is a **microscope for the sensor**, not a driver. Five programs you run
that print what the sensor is actually doing, with the arithmetic spelled out
so you can check it by hand.

Work through it once with the sensor in front of you — about 40 minutes. The
goal is that by the end you could write the driver yourself, in any language.

Everything runs from the repository root on the host. Nothing is edited on the
Pi. If a command prompts for a password, run `ssh-copy-id <user>@<pi>`
once and it will stop.

## What the files are

| Where | What it is | Do you read it? |
|---|---|---|
| `python/probes/` | **Five programs you run.** This is the lesson. | Run first, read later |
| `python/heron/lidar/` | The library the probes use. `protocol.py` is the one that matters — it is the reference the C++ is checked against. | Read `protocol.py`, eventually |
| `python/tools/` | Host-side: draws plots, verifies captures | Just run them |
| `docs/rplidar-a1m8-findings.md` | The answer key: every measured result | Read **after** the probes |

Everything under `python/probes/` and `python/tools/` is disposable. It exists
to be thrown away once you know what you need. The durable outputs are what
you learn, and the `.rpraw` capture files (see the last section).

---

## 1. How do I talk to it at all? — 3 minutes

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_01_port.py
```

**Listen to the sensor.** It spins up for three seconds and stops. That is the
DTR line on the serial port being toggled — not a command. There is no motor
command on the A1, and no speed control either.

Then watch it ask three questions and get three answers. Notice that every
reply begins with the same seven-byte header, and that the header is printed
field by field.

**What you should be able to say afterwards:** the motor is a wire, not a
message; and a response is always a 7-byte descriptor followed by a payload.

Try `--reset` too. The reply is an ASCII banner rather than a descriptor,
which is the kind of thing that wedges a read loop.

---

## 2. What does a measurement actually look like? — 10 minutes

This is the important one.

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_02_wire.py --nodes 6
```

It prints five raw bytes, then the exact shifts that turn them into an angle
and a distance:

```
node[0000] @0x000000  3E 7B 1F 88 13
  b0 = 0b00111110  S=0  !S=1 (ok)  quality=0b001111=15
  b1 = 0b01111011  C=1 (ok)  angle_lo7=0b0111101=61
  b2 = 0b00011111  angle_hi8=31
    angle_q6 = 61 | (31 << 7) = 4029  ->  4029/64 = 62.953 deg
  b3b4 = 0x1388 (LE) = 5000  ->  5000/4 = 1250.0 mm
```

**Check one by hand against the hex.** Not all of them — one. When you can do
that, you understand the wire format, and that is most of the job. Use the
reference below.

### Reference: the 5-byte legacy node

```
          bit 7   6   5   4   3   2   1   0
byte 0  | <------ quality (6 bits) ------> | !S |  S |
byte 1  | <---- angle_q6 bits 0-6 ---->    |  C |
byte 2  | <---- angle_q6 bits 7-14 ----------------> |
byte 3  | <---- distance_q2 low byte --------------> |
byte 4  | <---- distance_q2 high byte -------------> |
```

| Field | How to extract it | Meaning |
|---|---|---|
| `S` | `byte0 & 0x01` | start of a new revolution |
| `!S` | `(byte0 >> 1) & 0x01` | must **differ** from `S`, or the node is bad |
| `quality` | `byte0 >> 2` | 0-63. `0` means no return |
| `C` | `byte1 & 0x01` | check bit, always **1** |
| `angle_q6` | `(byte1 >> 1) \| (byte2 << 7)` | 15 bits, in units of 1/64 degree |
| `angle_deg` | `angle_q6 / 64` | |
| `dist_q2` | `byte3 \| (byte4 << 8)` | little-endian uint16, units of 1/4 mm |
| `dist_mm` | `dist_q2 / 4` | **`0` means no return, not zero distance** |

Note the asymmetry that makes this easy to fumble: the angle is split 7 bits
then 8 bits across a byte boundary, because bit 0 of byte 1 is stolen for the
check bit. The distance is a plain little-endian uint16 with no such trick.

### Worked example: `3E 7B 1F 88 13`

```
byte0 = 0x3E = 0b00111110
  S    = bit 0             = 0
  !S   = bit 1             = 1      differs from S -> valid
  qual = 0b001111          = 15

byte1 = 0x7B = 0b01111011
  C        = bit 0         = 1      -> valid
  angle_lo = 0b0111101     = 61     (0x7B >> 1)

byte2 = 0x1F              = 31
  angle_q6 = 61 | (31 << 7) = 61 + 3968 = 4029
  angle    = 4029 / 64      = 62.953 deg

byte3/4 = 88 13 -> 0x1388 = 5000    (little-endian: high byte comes second)
  dist    = 5000 / 4        = 1250.0 mm
```

### Check your work

Paste any five bytes from the hexdump into this:

```
PYTHONPATH=python python3 -c "from heron.lidar import render; \
    print(render.annotate_node(bytes.fromhex('3e7b1f8813')))"
```

### Where this comes from

This is only the legacy node and the descriptor. For the **complete** protocol
-- every command, every packet format, the express and ultra capsule layouts
-- see [`lidar-references.md`](lidar-references.md), which indexes SLAMTEC's official
50-page protocol specification by page number.

- `python/heron/lidar/protocol.py`, the `decode_legacy_node()` docstring --
  the same layout, sitting next to the code that implements it.
- The vendor SDK states it most compactly, in `sdk/include/sl_lidar_cmd.h`:
  ```c
  sl_u8   sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
  sl_u16  angle_q6_checkbit; // check_bit:1;angle_q6:15;
  sl_u16  distance_q2;
  ```
- SLAMTEC's "RPLIDAR Interface Protocol and Application Notes" PDF, from their
  download page, is the prose version.

### Reference: the 7-byte response descriptor

Every reply from the sensor starts with one of these, so it is worth being
able to read it too:

```
byte 0-1   A5 5A                     fixed sync
byte 2-5   little-endian uint32:     bits 0-29  = payload length
                                     bits 30-31 = mode (1 = keeps repeating)
byte 6     data type                 0x81 legacy scan, 0x82 express,
                                     0x04 device info, 0x06 health,
                                     0x15 sample rate, 0x20 lidar conf
```

The 30/2 split is the trap: read bytes 2-5 as a plain length and SCAN's
`length=5, mode=1` becomes `0x40000005`, which looks like a plausible buffer
size rather than an error.

Then break it on purpose:

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_02_wire.py --nodes 2 --desync 1
```

One byte is discarded, so the stream starts mid-node. Watch the alignment
search recover. There is **no sync word** in the scan stream — the only thing
to search for is the redundancy inside the nodes themselves (`S` must differ
from `!S`, and the check bit must be 1). That is three constrained bits, so a
random byte pair passes about one time in four, which is why ten consecutive
good nodes are required before alignment is believed.

**What you should be able to say afterwards:** how a measurement is encoded,
and why resynchronisation is the hardest part to get right in C++.

---

## 3. How do I get one 360° scan? — 10 minutes

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_03_revolution.py --revs 1 --table
```

Read the `--table` output carefully. **The angles are not in ascending order.**
Roughly 12% of samples arrive behind the one before them. This is the single
most important finding in the whole exercise: a revolution must be sorted by
angle before it can be treated as a scan. SLAMTEC's own C++ SDK ships an
`ascendScanData()` function for exactly this reason.

Look also at the summary: the S start flag fires around 358°, never at 0°. It
marks a *transmission* boundary, not an angular one.

Then, with a box or a book in your hand:

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_03_revolution.py --bearing-test
```

Move the object around the sensor and watch the bearing number. This is how
you find out where 0° points and whether the angle increases clockwise or
anticlockwise. **This is still unanswered**, and the robot's coordinate
transform depends on it. Write what you find at the bottom of this file.

**What you should be able to say afterwards:** why "a scan" is a software
concept, not something the sensor hands you.

---

## 4. What does it do over a minute? — 5 minutes

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_04_capture.py \
    --seconds 60 --tag room --note "describe the scene here"
scripts/fetch_captures.sh <user>@<pi>
.venv/bin/python python/tools/lidar_report.py captures/room_*.jsonl
```

Open `reports/room_*/polar.png`. That is your room. The red ticks around the
rim are the beams that got nothing back — about 68% of them in open space.
That is the common case, not an edge case, and it is why any API returning a
bare `float distance` is wrong.

`summary.txt` in the same directory is the numbers. Compare them against
`docs/rplidar-a1m8-findings.md`.

Worth doing twice: once in an open room, once with the sensor half a metre
from a wall. Compare the invalid fractions. That single comparison tests
whether "most beams simply exceed the range" is the right explanation.

**What you should be able to say afterwards:** the real sample rate, the real
rotation rate, and what fraction of your data is nothing at all.

---

## 5. Is there a faster mode? — 5 minutes

```
scripts/deploy_pi.sh <user>@<pi> probes/lidar_05_express.py
```

Express mode packs 32 measurements into an 84-byte capsule and sends a start
angle instead of a per-sample angle. Read the A/B table at the bottom: double
the sample rate, half the angular spacing, same rotation rate.

The cost is structural, not CPU. Express angles are *interpolated* between two
capsules' start angles, so a capsule cannot be decoded until the **next** one
arrives — roughly 21 ms of built-in latency. The probe shows this happening:
one capsule emits nothing, and the following capsule closes it.

**What you should be able to say afterwards:** what express buys, what it
costs, and why it needs a different buffering design.

---

## Then watch it live

The five lessons above print numbers. To *see* them, there is a viewer: a
server on the Pi and a GUI on the host.

```
scripts/deploy_pi.sh <user>@<pi> server/lidar_server.py
.venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
```

Motor and scan-mode control, a scan toggle, and a range filter that both hides
distant returns and rescales the plot so the kept region fills the canvas —
set it to 30 cm and a nearby object stops being a few pixels at the origin.
Move something through the beam and watch it move.

Full instructions, including why two of the five scan modes draw no points,
are in [`lidar-gui.md`](lidar-gui.md).

---

## Then read exactly one file

`python/heron/lidar/protocol.py` — about 400 lines, half of them comments.

It is deliberately boring: no I/O, no logging, no threads, only `dataclasses`,
`enum` and `struct`. Fixed-point values stay integers (`angle_q6`, `dist_q2`)
and every bit-field is an explicit shift and mask, commented with the field
name from the protocol document. That is all so it can be transliterated to
C++ rather than redesigned.

If you can read that file and the probe-2 output side by side and see that
they are the same thing, you are ready to design the real interface.

Read `driver.py` next if you want the stateful parts — the STOP-settle-flush
dance, the RESET banner, the motor lifecycle, the resync loop.

---

## The part that matters later

Every capture writes two files with the same basename:

- `.rpraw` — the exact bytes off the wire, with chunk boundaries preserved
- `.jsonl` — the decoded records, one JSON object per line

The `.rpraw` is the valuable one. Any probe replays it, on your Mac, with no
hardware attached:

```
PYTHONPATH=python python3 python/probes/lidar_03_revolution.py \
    --replay captures/room_*.rpraw --revs 1
```

And this checks that the raw bytes reproduce the decoded output exactly:

```
python3 python/tools/lidar_rawcat.py captures/room_*.rpraw --verify
```

**This is how the C++ is known to be correct.** The C++ library wraps the
vendor SDK rather than transliterating `protocol.py`, and is checked against
this decoder on the same recordings: `lidar_sdkcheck.py` for the SDK's decode,
`lidar_adaptercheck.py` for the whole library, point for point. A parser that
agrees with a real recording is a parser you can trust in the robot. Keep a
couple of good captures — an open room, a close wall, a cluttered scene — as
fixtures.

---

## Questions still open

Fill these in as you answer them. They need a physical setup, not more code.

- [x] **Where does 0° point, and which way does the angle increase?**
      (probe 3 `--bearing-test`) — **answer: 0° points out the front, along
      the housing's symmetry axis; angle increases clockwise viewed from
      above.** Target on the rear face read 175.3°, and 262.3° after a 90°
      clockwise swing (Δ +87.0°). This makes the sensor frame left-handed
      relative to REP 103, so the robot transform negates the angle. Detail
      in `rplidar-a1m8-findings.md`.
- [ ] Invalid fraction against a close wall vs an open room — answer: ______
- [ ] Behaviour against dark matte surfaces, mirrors, sunlit windows
- [ ] How many revolutions to discard after motor-on
- [ ] What happens on USB unplug and re-enumeration, and the reconnect state
      machine that implies
- [ ] Whether the higher rotation-period jitter seen on the replacement power
      supply comes from the supply or from the mounting (needs captures from
      an unchanged physical position). The old `throttled=0x50000` question is
      obsolete — the supply was replaced and now reads `0x0`.

One more that is not about the sensor but will bite the robot: a revolution
takes ~137 ms, so once the robot is moving, the points in a single scan come
from different positions. Treating a revolution as instantaneous will smear
the map. The per-sample timestamps in the capture format are what make
deskewing possible later.
