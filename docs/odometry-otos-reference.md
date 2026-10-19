# SparkFun OTOS (PAA5160E1) — sensor reference

The odometry sensor: SparkFun's Optical Tracking Odometry Sensor, SEN-24904,
on the Pi's I2C bus. This document is living. As measurements replace
claims, update the tags and the text.

**Tags.** **[ours]** measured on our hardware. **[community]** stated by a
source in §9 (the bracket names it). **[unverified]** our inference, or a
claim no source makes directly. Little is **[ours]** yet: the sensor answers
on the bus, and nothing else has been measured.

Written in our own words from the sources in §9, retrieved 2026-10-12. **No
code from any source is in this repository.**

## 1. What it is

- One 1" × 1" board with three parts [community: PP, HG, FW]:
  - a **PixArt PAA5160E1-Q** optical flow chip, which reads the floor with
    an 850 nm class 1 laser;
  - an **ST LSM6DSO** IMU (gyro and accelerometer);
  - an **STM32C011** microcontroller that fuses the two.
- **The board does the fusion.** Three Kalman filters (x, y, heading) each
  estimate position, velocity and acceleration. The host only reads the
  results [community: FW].
- **Heading comes from the gyro alone.** The optical chip supplies only x/y
  motion, which the board rotates into the world frame by the heading and
  fuses with the accelerometer [community: FW, F1].
- **Outputs:** pose (x, y, heading), velocity and acceleration, each with a
  standard deviation, all in the world frame. Also raw IMU words and raw
  optical bytes. The standard deviations are filter statistics, not
  measured error [community: RM, AR].
- **Units:** metres and radians on the device. Inches and degrees exist only
  in SparkFun's host libraries [community: AR].

## 2. Electrical and wiring

| Item | Value | Tag |
|---|---|---|
| Supply | **3.3 V only.** The 3.3 V net feeds the MCU and IMU directly; there is no 3.3 V regulator. The optical chip's absolute maximum is 3.3 V | [community: SCH, DS] |
| Logic level | 3.3 V, so it connects to the Pi's GPIO2/GPIO3 directly | [community: SCH] |
| Connectors | 2 × Qwiic (JST-SH 4-pin; either one, the second is for chaining). A 6-hole 0.1" edge header J3, not populated: GND, 3V3, SDA, SCL, IO9, IO10 | [community: SCH, HG] |
| Pull-ups | 2.2 kΩ on SDA and SCL, behind a trace jumper that ships closed | [community: SCH, HG] |
| Current | Not stated for the board. Optical chip about 14.5 mA, plus 2 mA for the laser | [community: DS] |
| IO9, IO10 | IO9 is a data-ready output in normal use; both are the bootloader UART for firmware updates. Neither is needed to read the sensor | [community: HG, FW] |

**Our wiring** (done 2026-10-13, through a HAT that passes the pins through): 3V3 to Pi pin 1, SDA to pin 3 (GPIO2), SCL to
pin 5 (GPIO3), GND to a ground pin. IO9 and IO10 unconnected. Never a 5 V
pin (2 or 4).

**Pull-ups in parallel [unverified].** The Pi 3 has its own pull-ups on
GPIO2/3 (1.8 kΩ, general knowledge). Together with the board's 2.2 kΩ that
is about 1 kΩ, slightly above the I2C spec's sink current. Leave the jumper
alone unless the bus misbehaves.

## 3. I2C

- **Address 0x17,** fixed; there is no jumper [community: PP, HG, FW].
  `i2cdetect -y 1` shows 0x17 and nothing else on bus 1 [ours, 2026-10-13].
- **Product ID:** register 0x00 reads **0x5F** [community: RM, AR].
- **Speed:** no source states a maximum. SparkFun advise 100 kHz
  [community: GI#5]. Provisioning sets the Pi's bus to 100 kHz.
- **Clock stretching:** the board stretches the clock [community: FW, GI#5].
  The Pi 3's I2C controller has a long-documented problem with clock
  stretching [community: PI]. **Probe O1 (`otos_bus`), 2026-10-13, at
  100 kHz:** 4000 combined write-then-read transactions in 11.8 s, 2000 of
  them 37-byte reads from 0x1F, with 0 failures, 0 bad product IDs and 0
  pose jumps [ours]. A 37-byte read took 3.66 ms at least (about the bare
  bus time), 5.30 ms on average, 5.92 ms at most [ours]. Whether the extra
  is clock stretching or scheduling is not known [unverified]. One short run
  on a still sensor: not yet a soak test.
- **Soak, 2026-10-18** [ours]: `otos_bus 150000` with `lidar_adapter` and
  `remote_adapter` running throughout, the Pi on Wi-Fi, the sensor still.
  300 000 transactions, 150 000 of them 37-byte reads, in 976.8 s: **0
  failures, 0 bad product IDs, 0 pose jumps.** A 37-byte read took 3.66 ms
  at least, 5.83 ms on average, 6.28 ms at most: the minimum unchanged,
  the average 0.5 ms above the unloaded run. Meanwhile the LiDAR delivered
  7653 scans with 0 sequence skips (2 grab timeouts), and the remote link
  stayed OK at about 130 frames/s with 0 stale events.
  - About 152 iterations a second for most of the run, but about 245 a
    second for the first 3 s and for 5 s around 612 s. The cause is not
    known [unverified].
  - No start-up sequence (the probe writes nothing): status 0x01, tilt, in
    every snapshot, and the heading drifted +19.6° over the run, about
    1.2°/min, as an uncalibrated board does (§8).
- **Transfers:** write the register index, then read or write; the index
  auto-increments. Reads past 0x5F return 0 [community: FW].
- **A single read is a coherent snapshot.** The board updates the readable
  registers only between host reads, so a burst read in one transaction is
  consistent [community: FW]. Read whole blocks in one transaction
  [unverified].
- **Byte order:** every multi-byte value is a signed 16-bit little-endian
  word [community: RM, AR, FW].

### Register map [community: RM, AR, FW]

| Reg | Name | Access | Format, raw int16 to SI |
|---|---|---|---|
| 0x00 | Product ID | R | 0x5F |
| 0x01, 0x02 | HW, FW version | R | high nibble major, low nibble minor; FW 1.0 reports HW 1.1, FW 1.0; ours reads HW 1.1, FW 1.0 [ours] |
| 0x04, 0x05 | Linear, angular scalar | R/W | int8: scalar = 1 + raw × 0.001, so 0.872 to 1.127 |
| 0x06 | IMU calibration | R/W | write N samples (1–255); reads back the remaining count, 0 when done |
| 0x07 | Reset | R/W | bit 0 resets tracking (pose to 0); reads back 0 when done |
| 0x0E | Signal processing | R/W | bit 0 LUT, bit 1 accelerometer, bit 2 rotation, bit 3 Kalman variance; default 0x0F |
| 0x0F | Self test | R/W | bit 0 start, 1 in progress, 2 pass, 3 fail; tests the IMU only |
| 0x10–0x15 | Offset x, y, h | R/W | position scaling; **write all 6 bytes at once**; always reads back 0 |
| 0x1F | Status | R | bit 0 tilt warning, 1 optical tracking warning, 6 optical chip error, 7 IMU error |
| 0x20–0x25 | Position x, y, h | R/W | x, y: raw × 10/32768 m (±10 m, about 0.3 mm); h: raw × π/32768 rad. Writing all 6 bytes re-seats the pose |
| 0x26–0x2B | Velocity | R | x, y: raw × 5/32768 m/s; h: raw × (2000°/s in rad)/32768 |
| 0x2C–0x31 | Acceleration | R | x, y: raw × 16 × 9.80665/32768 m/s²; h: raw × 1000π/32768 rad/s² |
| 0x32–0x43 | Std dev of position, velocity, acceleration | R | the same scalings as the blocks above |
| 0x44–0x4F | Raw IMU gyro x/y/z, accel x/y/z | R | LSM6DSO formats |
| 0x50–0x5B | Raw optical bytes | R | undocumented |

SparkFun's libraries read 18 bytes from 0x20 (pose, velocity,
acceleration), 18 from 0x32 (standard deviations), or 36 from 0x20.

## 4. Start-up and initialisation

- **On power-up the board starts by itself.** It does a quick 64-sample gyro
  calibration about 100 ms in, still or not [community: FW]. The optical
  chip needs at least 120 ms before motion is valid [community: DS].
- **Nothing persists across a power cycle:** scalars, offset and
  calibration all reset [community: AR, FW]. The host must apply its
  settings after every power-up. There is no reset flag; a scalar that reads
  back 0 after being set would show a restart [unverified].
- **What SparkFun's libraries do** [community: AR, PY]:
  1. Check that 0x00 reads 0x5F.
  2. Optionally run the self test: about 20 ms.
  3. **IMU calibration:** write 0x06 = 255 and poll until it reads 0, about
     0.6 s. **The board must be still and flat** throughout.
  4. Set the scalars and the offset.
  5. Reset tracking (0x07 = 1).
- **Steps 3 and 5 are needed [ours, probe O2, 2026-10-13].** Without them, a
  still board read a tilt warning and drifted 12°/min (§8, runs 1–3). After
  them: calibration took 619 ms and the reset 11 ms; status 0x00 for 30 s;
  pose stayed at 0; heading wandered within 0.04° and ended at 0.000°. One
  30 s run, sensor side down, still.
- **The library's sequence works on the board [ours, probe O5, 2026-10-18].**
  `OtosDevice::start()` with scalars 0.995 and 1.002: product ID and
  versions read (HW 1.1, FW 1.0), calibration 619 ms, reset 11 ms, **both
  scalars read back as written** (0xFB, 0x02). Then 10 s still: status 0x00
  throughout, 651 reads with 0 failures, heading −0.027°.
  - **Velocity read 0.3–1.1 mm/s in x and y while the pose stayed at
    0.0000 m.** Integrated, that would be several millimetres in 10 s, so
    the board does not simply integrate its reported velocity while still.
    Why is not known [unverified].
  - 651 reads in 10 s is 65 a second, not the 100 the 10 ms sleep
    suggests: each loop is the sleep plus a read of about 5.8 ms (§3). The
    reader thread must schedule by deadline, not sleep after each read.

## 5. Calibration

**Probe O3 (`otos_move`), run 1, 2026-10-15** [ours]: five pushes of
1000 mm along a tape measure, default scalars, sled and floor not recorded.

| # | Reported mm | Error | x, y mm | Heading change | Path mm | Status |
|---|---|---|---|---|---|---|
| 1 | 1013.2 | +1.32% | +60.7, +1011.4 | −39.77° | 1033.8 | 0x01 |
| 2 | 953.8 | −4.62% | −545.3, +782.5 | −4.99° | 1106.7 | 0x03 |
| 3 | 943.7 | −5.63% | −405.6, +852.1 | −7.99° | 1101.4 | 0x01 |
| 4 | 999.4 | −0.06% | −419.0, +907.3 | −11.85° | 1053.1 | 0x00 |
| 5 | 1010.9 | +1.09% | −53.7, +1009.5 | −28.58° | 1028.5 | 0x00 |

- The two clean trials (status 0x00) were within 1.1% [ours].
- Trials 2–4 moved 25–35° off the sensor's y axis, and every trial showed a
  heading change of 5–40°. Whether the sled turned, was pushed at an angle,
  or the heading is wrong is not known [unverified]. A 40° turn spread over
  the push would have bent the path far more than trial 1's 61 mm sideways,
  so trial 1's turn, if real, came at the start or the end [ours,
  arithmetic].
- Tilt (bit 0) in trials 1–3 and an optical warning (bit 1) in trial 2.
- Too mixed to set a scalar. Next: a guided, straight push.

**Run 2, 2026-10-17** [ours]: the sled stiffened so it could not tip, and
pushed along a straight edge. Five pushes of 1000 mm, default scalars.
Wood panel floor; sensor height not recorded.

| # | Reported mm | Error | x, y mm | Heading change | Path mm | Status |
|---|---|---|---|---|---|---|
| 1 | 1001.2 | +0.12% | −22.0, +1001.0 | −0.02° | 1009.6 | 0x00 |
| 2 | 999.8 | −0.02% | −6.7, +999.8 | −1.01° | 1003.4 | 0x00 |
| 3 | 996.4 | −0.36% | +0.6, +996.4 | −0.49° | 999.8 | 0x00 |
| 4 | 1004.9 | +0.49% | +0.3, +1004.9 | +0.97° | 1010.6 | 0x00 |
| 5 | 1006.8 | +0.68% | +0.3, +1006.8 | +1.24° | 1010.2 | 0x00 |

- **Mean +0.18%, spread −0.36% to +0.68%, standard deviation 4.1 mm
  (0.41%)** [ours]. The mean is within one standard error (0.18%) of zero,
  so a linear scalar of 0.998 is not supported by five trials; **the scalar
  stays at 1.0** on this floor for now [ours, arithmetic].
- Status 0x00 throughout; heading change within ±1.3°; start heading 0.00°
  after every reset; travel within 1.3° of the sensor's +y axis.
- **Run 1's heading changes and warnings came from the sled turning and
  tipping**, not the sensor: the user saw it turn while being lined up, and
  a rigid, guided sled removed them [ours].
- Path length runs 3–10 mm over the straight-line distance: small sideways
  wander along the push.

**Probe O4 runs 1–2 are not valid for the scalar** [ours]: the sled was
tethered by its cable, which wound up and pulled on it (notes below). They
are kept as a record of that. The angular scalar is to be measured again
with out-and-back trials, which keep the cable from winding.

**Probe O4 (`otos_spin`), run 1, 2026-10-17** [ours]: the same sled spun
by hand 10 turns in place, from a mark back to it, all in the same
direction (negative heading). Default scalars, read back as raw 0. Wood
panel floor.

| # | Reported deg | Error | End heading | Moved x, y mm | Time s | Fastest deg/s | Status |
|---|---|---|---|---|---|---|---|
| 1 | −3599.27 | −0.02% | +0.73° | −270, +469 | 55.4 | 544 | 0x03 |
| 2 | −3601.39 | +0.04% | −1.39° | −418, +1044 | 48.5 | 364 | 0x03 |
| 3 | −3605.28 | +0.15% | −5.28° | +1377, −870 | 43.6 | 513 | 0x03 |
| 4 | −3611.26 | +0.31% | −11.26° | +137, −2 | 44.2 | 338 | 0x03 |
| 5 | −3601.63 | +0.05% | −1.63° | +102, +240 | 48.2 | 322 | 0x03 |

- **Mean +0.11%, standard deviation 0.13%** over 3600° [ours]. The
  correcting scalar would be 0.999, one register step; **the angular
  scalar stays at 1.0** for now [ours, arithmetic].
- Trials 1, 2 and 5 ended within 1.7° of the mark, about what lining up
  by eye allows. Trials 3 and 4 ended 5° and 11° over. Whether that was
  the lining up or the gyro is not known [unverified]. Trial 4 had the
  slowest top speed, so it does not point to the gyro saturating.
- Gyro drift while still (§8, up to about 0.6°/min) adds under 0.5° to a
  trial of under a minute [ours, arithmetic].
- **Tilt and optical warnings (0x03) in every trial**, and the position
  moved 0.1–1.6 m although the sled stayed in place. The sensor is off the
  spin's centre, so some motion is real; how much of it is the optical
  tracking losing the floor while turning is not known [unverified]. It
  does not affect the heading, which is gyro only (§1).
- Only one direction was spun in run 1; run 2 spun the other way.

**Run 2, 2026-10-17** [ours]: the same, spun the other way (positive
heading), after a fresh start-up.

| # | Reported deg | Error | End heading | Moved x, y mm | Time s | Fastest deg/s | Status |
|---|---|---|---|---|---|---|---|
| 1 | +3581.60 | −0.51% | −18.40° | +76, −64 | 41.0 | 325 | 0x01 |
| 2 | +3580.38 | −0.54% | −19.62° | +76, −46 | 41.8 | 271 | 0x01 |
| 3 | +3559.10 | −1.14% | −40.90° | +126, −193 | 43.6 | 406 | 0x01 |
| 4 | +3571.51 | −0.79% | −28.49° | +39, −115 | 33.4 | 410 | 0x01 |
| 5 | +3576.79 | −0.64% | −23.21° | +35, +6 | 37.2 | 341 | 0x01 |

- **Mean −0.72%, standard deviation 0.26%:** every trial short by 18–41°,
  far more than lining up by eye allows [ours].
- **The error depends on the direction.** Spun negative, the board
  over-reports by 0.11%; spun positive, it under-reports by 0.72% [ours].
  Correcting scalars 0.999 and 1.007. One scalar multiplies both
  directions, so it cannot remove the difference: the best single value,
  about 1.003, would leave about ±0.4% each way [ours, arithmetic].
- **Not explained by the still drift.** A constant offset fitted to both
  runs would be about −0.34°/s (−20°/min), against 0.2–0.6°/min measured
  still (§8) [ours, arithmetic]. Whether the cause is a direction
  dependent gyro scale, a bias that appears only while turning, or the
  sled tipping while spun by hand is not known [unverified].
- Tilt warning (0x01) in every trial, but no optical warning, and the
  position moved only 0.04–0.23 m, against 0.1–1.6 m in run 1. The sled
  may have been spun differently [unverified].
- **The cable wound up in both runs:** by turn 6 or 7 the user was
  fighting it [ours, the user's report]. Its pull may have tipped or
  dragged the sled, which would fit the tilt warnings and some of the
  error; runs 1–2 cannot separate it from the sensor.
- In both runs the sled was stopped at the mark after every turn
  [ours, the user's report]: ten stops and starts per trial.
- **Angular scalar stays at 1.0** until the cause is known.

**Run 3, 2026-10-18** [ours]: out and back, 5 turns one way, 5 back to
the mark, so the true total is 0; the Pi on Wi-Fi with its Ethernet cable
unplugged, one cable fewer on the sled. Trials 1–3
stopped at the mark every turn, 4–6 turned smoothly. The probe was the
build before the leg split, so only the net angle was recorded.

| # | Net deg | Moved x, y mm | Time s | Fastest deg/s | Status |
|---|---|---|---|---|---|
| 1 | +9.48 | +24, −14 | 47.9 | 361 | 0x01 |
| 2 | +3.15 | +3, +40 | 44.9 | 417 | 0x01 |
| 3 | +4.12 | −20, −4 | 41.8 | 407 | 0x01 |
| 4 | −6.46 | +9, −45 | 37.0 | 297 | 0x01 |
| 5 | +12.60 | +15, +34 | 36.9 | 416 | 0x01 |
| 6 | +9.77 | +20, +19 | 27.6 | 374 | 0x01 |

- **Net mean +5.4°, standard deviation 6.6°, over 3600° turned** [ours].
  Runs 1–2's errors (+0.11% and −0.72% by direction) would have left about
  −15° net every trial [ours, arithmetic]. They did not: **the direction
  asymmetry of runs 1–2 came from the cable**, not the gyro.
- What remains, about 0.15% of the angle turned, is the same size as
  lining the sled up by eye (a few degrees) [ours, arithmetic]. No
  difference between stopping every turn and turning smoothly.
- Tilt warning (0x01) in every trial, no optical warning; position moved
  under 50 mm.
- Each direction's own scale is still unmeasured: the leg split will give
  it.

**Run 4, 2026-10-18** [ours]: out and back with the leg split. Each trial:
5 turns positive, a brief pause on the mark, 5 turns negative back to the
mark, smoothly. Wi-Fi, Ethernet unplugged. Default scalars.

| # | Out deg | Back deg | Out err | Back err | Both legs | Net deg | Status |
|---|---|---|---|---|---|---|---|
| 1 | +1767.06 | −1791.52 | −1.83% | −0.47% | −1.15% | −24.46 | 0x01 |
| 2 | +1795.30 | −1785.67 | −0.26% | −0.80% | −0.53% | +9.64 | 0x01 |
| 3 | +1796.36 | −1796.39 | −0.20% | −0.20% | −0.20% | −0.03 | 0x01 |
| 4 | +1800.60 | −1797.78 | +0.03% | −0.12% | −0.05% | +2.82 | 0x01 |
| 5 | +1798.33 | −1800.40 | −0.09% | +0.02% | −0.04% | −2.07 | 0x01 |

"Both legs" is (|out| + |back|) / 3600 − 1. It does not depend on where
the sled was reversed, which shifts one leg's error up and the other's
down by the same amount [ours, arithmetic]. Each trial took 23–26 s,
fastest 366–432°/s; moved under 0.21 m.

- **The error shrank trial by trial:** −1.15%, −0.53%, −0.20%, −0.05%,
  −0.04% [ours]. The sled turned the same 3600° each time, so what changed
  was the handling, not the sensor: practice spinning it flat and smoothly
  [ours, inference]. The tilt warning (0x01) was on in every trial, so the
  sled was still tipping [ours].
- **No difference between directions:** trials 2–5, positive −0.13%,
  negative −0.28%, within the scatter and the turnaround's effect [ours].
  Runs 1–2's asymmetry was the cable.
- Trials 2–5, both legs: mean −0.20%, standard deviation 0.23% [ours].
  The two smoothest trials were within 0.05%.
- **Angular scalar stays at 1.0** [ours]. The remaining error is the size
  of the handling error and falls with steadier spinning. The robot, which
  turns under motor power and without tipping, is where to measure it
  again.

- **Scalars:** 0.872 to 1.127, in 0.001 steps [community: RM, AR].
- **Angular first, then linear,** both starting from 1.0 [community: AR]:
  - **angular:** spin about 10 full turns; scalar = true angle / reported
    angle. Independent of the floor, since it is gyro only [community: F1];
  - **linear:** drive known distances; scalar = true / measured.
- Typical scalars are 0.95–1.05. A need for much more than 5% points to
  poor tracking, not scaling [community: F1, F2, F3].
- **The board's speed-correction table (LUT) was built on FTC foam tiles
  at 10 mm** [community: FW]. On other floors it may not fit [unverified].
- Encode a scalar as a properly rounded two's-complement int8. SparkFun's
  Arduino rounding can be one step off for negative corrections
  [unverified: our reading of AR].

## 6. Mounting

- **Height:** 10–27 mm from the sensor's face to the floor. Glossy floors
  work across the range; dark or diffuse ones may need the low end
  [community: DS, HG]. Height changes of more than 1 mm can affect
  accuracy, especially at speed [community: GI#2].
- **Rigid, flat and level,** within 3° [community: DS, HG].
- **Remove the yellow Kapton tape** over the sensor [community: HG]. Dust
  on the lens is a common cause of under-reading [community: F1, F2, F3].
- **Floors:**
  - good: concrete, epoxy, laminated wood, glossy floors [community: HG];
  - hard: dark, absorptive or very rough surfaces, such as black carpet
    [community: DS].
- **Speed:** typically up to 1.5 m/s [community: DS].
- **Offset from the robot centre:** write the sensor's pose in the robot
  frame to 0x10–0x15, and the board then reports the robot centre's pose
  [community: FW, AR]. The offset cannot be read back, and one Linux user
  saw writes fail silently [community: GI#1]. Verify an offset by its effect
  [unverified].

## 7. Update rate

- The fusion loop runs on the IMU's data-ready interrupt at **416 Hz**, so
  outputs update about every **2.4 ms** [community: FW].
- No source prescribes a polling rate. The board integrates internally, so
  slow polling loses nothing except the position range limit in §8
  [unverified].
- **IO9 falling edge = fresh data** [unverified: our reading of FW]. It could
  be wired to a Pi GPIO later for timestamps.

## 8. Limits and known problems

- **Accuracy:** 3–5% out of the box; under 1% calibrated, in ideal
  conditions, cumulative over the path [community: PP, F1]. No drift
  specification.
- **Heading drift:** gyro only, with no temperature compensation. One user
  measured about 18°/h while stationary [community: GI#2]. A rapid spin
  points to a bad IMU calibration [community: F4, HG].
  **Ours, probe O2, 600 s, 2026-10-15** (board level to 1.3° by the
  accelerometer, sensor side down, still, after calibration and reset):
  heading +5.85° in 10 min, 0.59°/min, about 35°/h on average [ours]. The
  rate grew during the run: per minute +0.28, +0.18, +0.34, +0.56, +0.52,
  +0.59, +0.68, +0.89, +0.92, +0.90° [ours]. So the gyro bias moved after
  calibration; why is not known (temperature is a candidate, given no
  compensation) [unverified]. Status 0x00 and pose exactly 0 throughout;
  43215 readings, 0 failed [ours].
  **Run 2, straight after:** heading +3.22° in 10 min, 0.32°/min, about
  19°/h [ours]. Per minute +0.21, +0.20, +0.23, +0.30, +0.34, +0.37,
  +0.40, +0.35, +0.37, +0.47° [ours]. So a fresh calibration brought the
  rate back to about 0.2°/min, and it grew again, more slowly [ours]. Between
  the runs, uncalibrated, the heading had moved to +34° and the tilt warning
  had come back, with the board untouched [ours]; consistent with roll and
  pitch drifting as a gyro integral (below) [unverified].
- **Position range ±10 m** in the registers. What happens beyond is
  unknown; possibly wraps [unverified]. Matters for a floor robot.
- **Tilt warning on the bench [ours]:** status read 0x01 (tilt) in all
  2000 snapshots of probe O1, with the board loose on the bench, sensor
  side up. The working orientation is sensor side down, facing the floor,
  so the board may simply have been upside down [unverified].
  **Run 2,** tape off, turned sensor side down and raised, without a power
  cycle: status 0x03 (tilt and optical tracking) in every snapshot; pose
  x +2.06 m, y −6.31 m, heading −40° after the handling; heading drifting
  −4.2° in 11.4 s, against −0.011° in run 1 [ours]. A guess: roll and pitch
  are a gyro integral from power-up (§8), so after being turned over the
  board still thinks it is upside down [unverified].
  **Run 3,** after a full power cycle, sensor side down, raised, still:
  status 0x01 (tilt only) in every snapshot; heading −1.6° in 8.2 s; pose a
  few cm from boot [ours]. So the tilt warning is not from being turned over
  after power-up. Every read took 3.66–3.71 ms, against 5.3 ms on average in
  runs 1 and 2 [ours]; why is not known.
  **Probe O2** (full IMU calibration and reset, §4) cleared the warning
  (0x00 for 30 s), in the same position [ours]. The raw accelerometer read
  x −185, y −360, z +2044 (mean): gravity on +z with the sensor side down,
  and a tilt of about 11.2° from the vector [ours, arithmetic]. That is
  just under the 11.5° threshold, so the earlier warnings may have been a
  real borderline tilt, and calibration may take the current attitude as
  level [unverified]. 2044 counts for 1 g fits a ±16 g range (2048/g)
  [unverified].
- **Tilt:** beyond about 11.5° of roll or pitch the board stops using the
  accelerometer and sets the tilt warning [community: FW]. Long runs might
  raise a false tilt warning, because roll and pitch are a pure gyro
  integral [unverified].
- **Partial writes** of the offset or position are discarded silently
  [community: FW].

## 9. Sources and licenses

Retrieved 2026-10-12. **No code from any source is in this repository.**
Reusing any of it needs the maintainer's approval first, whatever its
license.

| Tag | Source | URL | License | What we may do |
|---|---|---|---|---|
| PP | Product page | <https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html> | none stated | read-only reference |
| HG | Hookup guide | <https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/single_page/> | none stated | read-only reference |
| RM | Register map PDF, in the hardware repository | <https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor> (`Firmware/OTOS_Register_Map.pdf`) | none stated | facts only |
| DS | PixArt PAA5160E1-Q datasheet v0.8 | <https://cdn.sparkfun.com/assets/c/6/8/0/5/PAA5160_Datasheet_General.pdf> | © PixArt, all rights reserved | read-only reference |
| AR | Arduino library v1.1.0 | <https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library> | MIT | reusable with notice, **after asking**; read-only so far |
| PY | Python library | <https://github.com/sparkfun/qwiic_otos_py> | MIT | as AR |
| FW | Board firmware v1.0, in the hardware repository | as RM (`Firmware/`) | **inconsistent:** MIT file headers; boilerplate license file mentions an unrelated SLA; contains Apache-2.0 and ST parts | read-only reference |
| SCH | Schematic, hardware design | <https://cdn.sparkfun.com/assets/e/7/0/1/0/24904_SparkFun_Optical_Tracking_Odometry_Sensor_Schematic.pdf> | **CC BY-SA 4.0 (share-alike)** | facts only; never copy design files |
| GI#n | Hardware repository issues #1–#6 | <https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor/issues> | none stated | community claims |
| F1–F4 | SparkFun forum threads: speed (F1), accuracy (F2), configuration (F3), heading drift (F4) | <https://community.sparkfun.com/t/otos-not-measuring-properly-at-medium-to-high-speeds/65662>, <https://community.sparkfun.com/t/sparkfun-optical-tracking-odometry-sensor-otos-accuracy/61178>, <https://community.sparkfun.com/t/sparkfun-optical-tracking-odometry-sensor-otos-not-holding-configuration-in-ftc-sdk/60220>, <https://community.sparkfun.com/t/sparkfun-otos-sensor-heading-drift/67406> | none stated | community claims |
| PI | Pi I2C clock-stretching reports | <https://github.com/raspberrypi/linux/issues/254> | none stated | community claims; read only as search summaries |

**Not read:** SparkFun's "Calibrating Your Odometry Sensor" tutorial
(search snippets only). PixArt publishes no register-level document for the
optical chip.

## 10. Open questions, for the hardware

1. **Answered for 100 kHz:** probe O1 was clean, alone and in a 977 s
   soak with the LiDAR and remote running (§3).
   **Does the Pi 3's I2C read the board reliably** at 100 kHz, given clock
   stretching? Can a 36-byte burst be read in one combined transaction? If
   not: a delay between write and read, or the bit-banged `i2c-gpio`.
2. Does an offset write over `/dev/i2c-1` take effect every time?
3. Stationary heading drift: two 10 min runs (§8). A fresh calibration
   resets the rate to about 0.2°/min, and it then grows. Why it grows, and
   how often the robot should recalibrate while stopped, are open.
4. The right mounting height for our floor, and how often the optical
   tracking warning appears.
   No optical warning on a rigid, guided sled (§5, run 2).
5. **Partly answered:** the linear scalar stays at 1.0, within 0.2% (§5).
   The angular scalar stays at 1.0: out and back, the steadiest spins
   were within 0.05% in both directions (§5, O4 run 4). Runs 1–2 were
   spoiled by the cable. Measure both scalars again on the robot. Also open: does the foam-tuned LUT
   help or hurt (0x0E bit 0)?
6. Distance error against speed. Repeatability at slow push speed: 0.41%
   standard deviation over 1 m (§5, run 2).
7. What happens beyond ±10 m.
8. Time from power-up to valid readings, and detecting a mid-run restart.
9. IO9: is the falling edge really fresh data at 416 Hz?
