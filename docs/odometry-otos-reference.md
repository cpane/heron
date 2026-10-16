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

## 5. Calibration

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

1. **Partly answered:** probe O1 was clean (§3). Still open: a long soak,
   and reads while the LiDAR and remote are running.
   **Does the Pi 3's I2C read the board reliably** at 100 kHz, given clock
   stretching? Can a 36-byte burst be read in one combined transaction? If
   not: a delay between write and read, or the bit-banged `i2c-gpio`.
2. Does an offset write over `/dev/i2c-1` take effect every time?
3. Stationary heading drift: two 10 min runs (§8). A fresh calibration
   resets the rate to about 0.2°/min, and it then grows. Why it grows, and
   how often the robot should recalibrate while stopped, are open.
4. The right mounting height for our floor, and how often the optical
   tracking warning appears.
5. Scalars on our floor; does the foam-tuned LUT help or hurt (0x0E bit 0)?
6. Distance error against speed, and repeatability.
7. What happens beyond ±10 m.
8. Time from power-up to valid readings, and detecting a mid-run restart.
9. IO9: is the falling edge really fresh data at 416 Hz?
