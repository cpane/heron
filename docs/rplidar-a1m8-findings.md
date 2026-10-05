# RPLIDAR A1M8 — measured behaviour

Everything here was measured on the actual unit, not taken from the datasheet.
Reproduce any of it with the probe named in each entry.

**Unit under test:** model `0x18`, firmware 1.29, hardware 7,
serial `6946595DF298E6A5C998EAC093ED87C8`, on a Raspberry Pi 3B (Debian 13,
Python 3.13.5) over a Silicon Labs CP2102 bridge at 115200 8N1.

**Power — resolved 2026-09-21.** The original captures were taken with a
marginal supply (`throttled=0x50000`: under-voltage and throttling had both
occurred). The supply was replaced and the Pi rebooted, which clears those
sticky bits. Re-tested: `throttled=0x0` idle, `0x0` with the motor spinning
and scanning for 30 s, and `0x0` with the motor plus four CPU burner threads
for another 30 s. Core voltage held 1.2875 V throughout, temperature peaked at
53.7 °C. **No under-voltage or throttling at any point.**

What changed in the data, comparing legacy captures across the two supplies:

| | rotation | revolution period stdev |
|---|---|---|
| old supply (`0x50000`) | 7.35, 7.39, 7.40 Hz | 0.00082, 0.00088, 0.00098 s |
| new supply (`0x0`) | 7.45, 7.47, 7.49, 7.52 Hz | 0.00116, 0.00139, 0.00201, 0.00232 s |

- **Sample rate is identical at 1954/s.** Power was never limiting throughput,
  so the rate figures in this document stand unchanged.
- Rotation is consistently ~0.1 Hz faster on the new supply — every new
  capture exceeds every old one. Consistent with a motor that free-runs at
  whatever voltage it is given (there is no speed control on the A1).
- Period jitter is consistently 1.5-2.8x higher, though still only ~1.7% of
  the period. **Not attributed:** the sensor was also moved between the two
  sets (invalid fraction shifted 68% -> 84%, mean distance 1520 -> 1749 mm),
  and mounting affects rotation stability. Isolating it would need captures
  from the same physical position.
- Invalid fraction and distance statistics are **not comparable** across the
  two sets, for the same reason: the scene changed.

Note when reading period stdev: express captures show ~0.0040 s in *both*
eras, about 4x legacy. That is an artifact of the express decoder, not the
motor — express sync bits are derived from the interpolated angle crossing
zero, so the revolution boundary is less precisely located than the legacy
S flag makes it.

## Rate and timing

| Question | Answer | Probe |
|---|---|---|
| Does the advertised sample rate hold? | **Yes.** Legacy: 1954 measured vs 1969 claimed (508 µs), −0.7%. Express: 3911 vs 3937 claimed (254 µs), −0.7%. | 4, 5 |
| Rotation rate | **7.3 Hz**, not the 5.5 Hz nominal. The motor is free-running at whatever the supply gives it; there is no speed control on the A1. | 4 |
| Rotation stability | **Very stable.** Period 134.2–138.1 ms, mean 136.0 ms, stdev 0.8 ms — 3.5% spread. | 4 |
| Samples per revolution | Legacy 264–267 (mean 265.8, stdev 0.43). Express 526–532 (mean 531). Varies because sample rate and rotation rate are independent — nothing locks them. | 4, 5 |
| Chunk arrival latency | p50 2.0 ms, p95 4.5 ms, p99 7.0 ms. This bounds the *measurable* part of sensor-to-Python latency; the sensor-internal term is unknown and would need an external trigger. | 4 |

> An earlier, cruder measurement during planning reported 1509 samples/s and
> 5.39 Hz. Both were artifacts of that script's own read loop, not the sensor.
> This is the main reason the capture path records bytes read, chunk count and
> `bytes/sample` — so a measurement can be checked against itself.

## Geometry and data semantics

- **The stream is NOT in angular order.** About 12% of samples arrive behind
  the angle of the sample before them, and ~82% of those are quality 15 —
  real returns. The smooth 1.359° progression comes largely from quality-0
  no-return filler. **A revolution must be sorted by angle before it is used
  as a 360° scan.** SLAMTEC's own C++ SDK ships `ascendScanData()` for exactly
  this reason. (probe 3)
- **The S start flag does not fire at 0°.** Measured over 139 revolutions it
  fires at 351.4°–359.5°, mean 358.6°, stdev 1.4°. It marks a *transmission*
  boundary, not an angular one. (probe 4)
- **Angular spacing is not uniform.** Legacy: median 1.40°, p95 3.3°, p99 5.5°,
  worst gap per revolution ~8–9°. Express halves it: median 0.70°, p95 1.40°.
  (probes 3, 4, 5)
- **The worst gap does NOT halve in express.** Re-measured 2026-10-01 over all
  11 captures (981 revolutions), every sample included and the 360° seam
  closed: per-capture medians 8.3–8.7° legacy and 8.0–8.1° express; worst
  single revolution 8.9° legacy, 8.1° express. Within a capture the widest gap usually sits at
  one bearing (~40° in two captures, ~106° in another), so it is not a fixed
  hole in the sensor frame. Cause not understood.
- **These gaps are between *samples*, not returns.** Over valid returns only,
  the same revolutions read 30–115° (median per capture), because 61–84% of
  samples return nothing and they cluster. That figure measures the scene,
  not coverage; using it as a drop detector flags every healthy revolution.
  A no-return sample still proves the beam looked there.
- **Angle is Q6** (`angle_q6 / 64`), split 7 bits / 8 bits across a byte
  boundary because bit 0 of byte 1 is a constant check bit. **Distance is Q2**
  (`dist_q2 / 4`), a plain little-endian uint16.
- **0° points out the front of the housing, and angle increases clockwise**
  viewed from above. Measured 2026-09-26 with a target on the housing's
  symmetry axis: at the rear face it reads **175.3°** (309 mm, range
  174.1–176.6 over ~60 revolutions); swung 90° clockwise it reads **262.3°**
  (337 mm, range 259.6–263.8). Δ = **+87.0°** for a nominal +90° move. The 3°
  shortfall and the 28 mm radius change are hand-placement error, not sensor
  offset; anticlockwise-positive would have read ~85°, so the direction is
  unambiguous. Spec figure 4-6 (protocol PDF p.17) agrees that θ is referenced
  to the symmetry axis, but it was treated as a hypothesis and confirmed
  against the hardware, not taken on trust. (probe 3)
- **The sensor frame is therefore left-handed**, which the robot transform
  must undo. REP 103 (x forward, y left, z up) is anticlockwise-positive, so
  the conversion negates rather than merely offsets:

  ```
  bearing_robot = -bearing_sensor + mounting_offset
  ```

  Getting the sign wrong mirrors every obstacle across the forward axis and
  produces no compile error — the robot simply steers toward obstacles.

  **Caveat on precision:** the probe reports the nearest *return*, i.e. the
  closest point on the target's surface, not its centre. For a flat face
  square-on these nearly coincide; for an angled or curved target the reading
  tracks the nearest corner. Treat 175.3° as "the rear face, ±a few degrees",
  not as a calibration constant. The *direction* is exact; the *offset* is
  approximate and should be re-derived from the mounting geometry once the
  sensor is bolted down.

## Signal quality

- **~68% of returns are invalid** (`dist_q2 == 0`) in an open room; ~61% in
  express mode. This is the common case, not an edge case. Any API returning a
  bare float distance leaks the sentinel into every consumer.
- **`quality == 0` means exactly "no return".** Of 15,619 quality-0 samples in
  one capture, **zero** were valid. The converse does not hold: quality 15 had
  18,562 samples of which only 10,172 were valid. So quality 0 is a reliable
  invalid marker; a non-zero quality is not a validity guarantee.
- **Range:** 126.8 mm to 8080 mm observed indoors (spec is 0.15–12 m). Mean
  valid distance 1520 mm.
- Untested: behaviour against dark matte surfaces, mirrors, and sunlit
  windows. Capture with `--note` describing the scene and compare.

## Provenance — where these protocol details came from

For the protocol itself rather than this unit's behaviour, see
[`lidar-references.md`](lidar-references.md): SLAMTEC's official specification, indexed by
page, plus the SDK files that settle each question.

The decoder was written from prior knowledge of the RPLIDAR protocol, then
validated two ways: empirically against this device, and by audit against
SLAMTEC's own SDK source (`github.com/Slamtec/rplidar_sdk`, files
`sdk/include/sl_lidar_cmd.h`, `sdk/include/sl_lidar_protocol.h`,
`sdk/src/sl_lidarprotocol_codec.cpp`,
`sdk/src/dataunpacker/unpacker/handler_capsules.cpp` and `handler_normalnode.cpp`).

**Confirmed against the SDK source, byte for byte:** every command opcode;
every response data type; the descriptor's `A5 5A` sync, `0x3FFFFFFF` size
mask and 2-bit loop flag; the request framing and XOR checksum; the legacy
node bit fields; the device-info, health and sample-rate struct layouts; the
84-byte capsule struct; express sync in the high nibbles with the checksum in
the low nibbles over bytes 2–83; the express angle interpolation including the
`<<13` / `>>10` fixed-point moves and the derived sync bit; the 5-byte
EXPRESS_SCAN payload; DTR polarity (`setDTR(true)` stops the motor); and the
health enum (0 OK / 1 Warning / 2 Error).

**Where this code deliberately differs from the SDK:**

- *Legacy quality scale.* The SDK rescales the 6-bit field to 0–255
  (`(sync_quality >> 2) << 2`); this code keeps the raw 0–63. **SDK quality
  values are 4× these.** Do not compare the two scales.
- *Express quality.* The capsule format has no quality field. Both the SDK and
  this code synthesise one; this code now uses the SDK's `0x2F`, so a C++ port
  produces identical numbers.

**Still unverified — the SDK does not settle these:**

- Health status 1 and 2 have never been observed; this unit always reports 0.

## Scan modes this unit actually supports (probe 6)

`GET_LIDAR_CONF` (0x84, firmware 1.24+) makes the device describe itself. This
is better than any datasheet, because it is what *this* unit at *this*
firmware reports:

| id | name | µs/sample | implied rate | max range | streams |
|---|---|---|---|---|---|
| 0 | Standard | 508.00 | 1969/s | 12.0 m | legacy 5-byte nodes (0x81) |
| 1 | Express | 254.00 | 3937/s | 12.0 m | express 84-byte capsules (0x82) |
| 2 | Boost | 127.00 | 7874/s | 12.0 m | **ultra capsules (0x84)** |
| 3 | Sensitivity ★ | 127.00 | 7874/s | 12.0 m | **ultra capsules (0x84)** |
| 4 | Stability | 201.00 | 4975/s | 12.0 m | **ultra capsules (0x84)** |

★ = the mode the device itself recommends (`SCAN_MODE_TYPICAL`).

Modes 0 and 1 agree exactly with `GET_SAMPLERATE` (508 / 254 µs), which is a
good cross-check that the query is being decoded correctly.

**This matters for the production driver.** Express is *not* this sensor's
best mode, and the mode the device recommends is one none of this code can
decode. Issuing `EXPRESS_SCAN` with `working_mode` 2, 3 or 4 was confirmed to
start a stream: the descriptor comes back as **length 132, multi-response,
type 0x84** — the ultra-capsule format (`sl_lidar_response_ultra_capsule_
measurement_nodes_t`: 2 checksum bytes, a uint16 start angle, then 32 cabins
of a packed uint32 encoding three samples each, so 96 samples per frame).

Decoding it is a separate job. The SDK's `handler_capsules.cpp` has the
reference implementation (`_onScanNodeUltraCapsuleData`); note it uses a
varbit/predictive encoding, so it is materially harder than the express
format, not just wider.

Request encoding note: the `EXPRESS_SCAN` payload is
`working_mode (u8), working_flags (u16), param (u16)` — five bytes. The
working_mode value is the scan-mode id from the table above.

## FORCE_SCAN (probe 6)

Verified. The descriptor is the same as `SCAN` — length 5, multi-response,
type 0x81 — and it streams legacy nodes at the full ~1473/s with the head
stationary.

What is *in* those nodes varies, and this was not fully isolated:

- **10 of 12 observed runs:** every node null — angle 0.00, quality 0,
  distance 0, a single distinct angle across thousands of samples.
- **2 runs:** real measurements instead, 100% valid, clustered near 353° at
  roughly 770 mm.

Settle time is not the variable (seven consecutive runs at 2 s and 10 s settle
all produced nulls), and neither is motor coasting (spinning first then
waiting 1 s, 5 s or 20 s all produced nulls). The most plausible remaining
explanation is simply what the parked head is aimed at, but that cannot be
varied without physically moving the sensor. **To finish this: put an object
~30 cm from the head and re-run probe 6.**

Practical conclusion either way: FORCE_SCAN confirms the link, the command
handling and the data stream, but a null result does not imply broken optics.

**Commands the SDK supports that this code does not implement:** `HQ_SCAN`
(0x83), `SET_LIDAR_CONF` (0x85), `SET_MOTOR_PWM` (0xF0),
`HQ_MOTOR_SPEED_CTRL` (0xA8), `GET_ACC_BOARD_FLAG` (0xFF), and
`NEW_BAUDRATE_CONFIRM` (0x90, firmware 1.30+). The two motor commands belong
to different motor-control capability classes than this device's — the A1 is
a DTR-controlled unit, which the SDK models as `MotorCtrlSupportDtr`.

## Protocol traps

1. **DTR is the motor.** Asserted = stopped, de-asserted = running. Opening the
   port asserts it. The motor spins whenever DTR is low **even with no scan
   running**, so a crashed process leaves it spinning. There is no PWM speed
   control on the A1 (`SET_MOTOR_PWM` 0xF0 is A2/A3).
2. **The descriptor packs 30 bits of length and 2 bits of mode** into a
   little-endian uint32. Read naively, SCAN's `length=5, mode=1` becomes
   `0x40000005` — a plausible-looking buffer size, so it fails late.
3. **Checksums apply only to payload-bearing requests.** `A5 20` (SCAN) has
   none; `A5 82 05 00×5 <xor>` (EXPRESS_SCAN) does. Get it wrong and the
   device simply never replies.
4. **No frame sync word in the scan stream.** Alignment can only be recovered
   from in-node redundancy (S ≠ !S, check bit = 1) — three constrained bits, so
   a random byte pair passes ~1 in 4. Require ~10 consecutive plausible nodes.
   Verified working with `lidar_02_wire.py --desync 1`.
5. **Express sync is in the HIGH nibbles, checksum in the LOW nibbles.**
   Headers arrive as `a9 5d`, `ac 5b`. Getting this backwards still passes a
   naive sync test often enough to look like intermittent corruption.
6. **An express capsule cannot be decoded until the next one arrives** — the
   angles are interpolated between two capsules' start angles. That is ~21 ms
   of structural latency and a different buffering design from legacy.
7. **Express carries no real quality field.** This code reports 0/15 to match
   the legacy invalid convention; those values must not be compared against
   legacy quality.
8. **RESET replies with an ASCII banner, not a descriptor.** Measured, not
   assumed (probe 1 `--reset`). The reply is 64 bytes of CRLF text:

   ```
   RP LIDAR System.
   Firmware Ver 1.29 - rtm, HW Ver 7
   Model: 18
   ```

   Timing, consistent across three trials: the whole banner arrives in a
   single burst **0.671 s** after the request, and `GET_INFO` is accepted
   **4 ms after the banner's last byte** — the device is ready as soon as the
   banner is out. **Wait for the banner, do not sleep a fixed interval.** A
   read loop expecting `A5 5A` wedges here; the vendor SDK simply fires RESET
   and ignores the reply, so it gives no guidance on either point.

   Note also that the banner prints the model byte as its **hex** digits:
   `GET_INFO` returns `0x18`, and the device calls itself "Model: 18".
   Rendering that byte in decimal (24) produces a number that matches nothing
   SLAMTEC prints.
9. **STOP is not acknowledged.** Wait, then flush, or the next command's
   descriptor gets parsed out of stale scan nodes.
10. **Reads do not return whole nodes.** In one capture, 30 of 506 sampled
    nodes straddled a read boundary. Buffer and reassemble; never assume a read
    yields an integral number of nodes.

    **Buffer overrun is silent, and this was measured.** Starving the reader
    (four competing CPU threads) while the sensor streamed produced:

    | | rate | bytes/sample | resyncs | max `in_waiting` | angular gaps >10° |
    |---|---|---|---|---|---|
    | reader alone | 1800/s | 5.000 | 0 | 11 | 0 |
    | reader starved | 961/s | 5.001 | 3 (8 bytes) | **4020** | **6, worst 174.2°** |

    The kernel tty buffer filled to ~4 KB and overflowed; roughly 128
    consecutive samples vanished at the worst gap. Note what did *not* happen:
    no error, no exception, byte alignment held (5.001 bytes/sample, only 8
    bytes discarded by resync). The **only** evidence of losing a third of the
    data was the angular discontinuity. At ~9770 bytes/s a 4 KB buffer is
    about 0.41 s of slack, so the production reader needs its own thread and
    must implement the gap detector — there is no sequence number on the wire
    to tell you.

    **Re-measured with the C++ SDK, express mode (2026-10-04).** Recorded with
    `lidar_record` through `TapChannel`, analysed with
    `python/tools/lidar_overrun.py`. Ground truth for loss is the capsule start
    angles on the wire: healthy consecutive steps are 1.00–1.02x the median
    (~21.5°) in all 9 healthy express captures, so any larger step is lost
    capsules.

    | Load | Capsules lost | What `grabScanDataHq()` delivered |
    |---|---|---|
    | 4 CPU hogs (`yes`), 80 revs | 0 | normal; longest mid-stream read gap 15 ms (idle: 9 ms) |
    | reader frozen 0.3 s | 0 | normal |
    | reader frozen 0.8 s (run 1) | ~1 (42.6° step) | one scan, 471 samples, **45.2° gap** |
    | reader frozen 1.5 s (run 1) | ~3 (95.4° step) | one scan, **929 samples**, widest gap 5.4° |
    | frozen 0.8 s + 1.5 s (run 2) | ~20 (332.4° and 152.2° steps) | one scan, **827 samples**, widest gap 6.9°; no gap over 10° anywhere |

    "Frozen" means `SIGSTOP`/`SIGCONT` on `lidar_record`, with the state
    confirmed `Tl` during each pause. Three conclusions:

    - **CPU load alone did not starve the native SDK reader**, unlike the
      Python reader above. Likely because the Python reader shared the
      interpreter lock with the competing threads; not demonstrated.
    - **An overrun does not reliably appear as a gap.** A small loss did; a
      larger one made the SDK publish two revolutions merged into one scan
      (normal is ~525–540 samples), which has no gap at all. The reliable
      signal in every case was the **sample count per scan**.
    - **Far less was lost than a 4 KB buffer predicts.** A 1.5 s stall at
      ~8.5 KB/s lost about three capsules (~250 bytes), so something below the
      reader buffers much more than 4 KB on this setup. Where (kernel tty
      layer or the CP2102) is not established.

    The two scans at each discontinuity are also the only ones where the SDK
    disagrees with the Python decoder, and where replay disagrees with live
    (98 of 100 match in both comparisons, in both runs). Cause not established.

    **After a stall the SDK's scan timestamps are arrival times.** The backlog
    is decoded in a burst and each revolution is stamped as it is decoded:
    measured intervals of 6 ms, and even −2 and −4 ms (stamps running
    backwards), against a normal 134–137 ms. Those revolutions claim to be
    fresh while holding data up to the stall's length old. The adapter drops
    them (`lidar-cpp-design.md` §6).
11. **`/dev/ttyUSB0` is not stable.** Use `/dev/rplidar` (see
    `target/udev/99-heron-rplidar.rules`). The LiDAR's protocol-level serial
    is invisible to udev; the CP2102's USB serial descriptor is the generic
    `"0001"`.

## Still open

- Invalid fraction against controlled surfaces and distances.
- Startup transient: how many revolutions to discard after motor-on.
- Behaviour on USB re-enumeration, and the reconnect state machine it implies.
- Whether the SDK's decode of the ultra modes (2, 3, 4) is correct. They
  stream genuine 0x84 frames and the SDK decodes them reproducibly, but
  nothing independent has checked the numbers.
- Whether the higher period jitter on the new supply is the supply or the
  mounting (needs captures from an unchanged physical position).
- **Scan deskewing.** A revolution takes ~137 ms; once the robot moves, its
  points come from different poses. Treating a revolution as instantaneous
  will smear the map. The per-sample timestamps in the capture format are what
  make deskewing possible later.
