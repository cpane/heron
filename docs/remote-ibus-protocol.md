# FlySky iBUS — protocol reference

The remote control link: a FlySky **FS-i6X** transmitter, an **FS-iA6B**
receiver, and the receiver's iBUS SERVO output wired to the Pi's GPIO UART.
FlySky publishes no iBUS specification, so everything here comes from
community reverse engineering, FlySky's user manuals, and our own
measurements. Section 8 lists the sources.

This document is living. When `ibus_reader identify`, `failsafe-test` or
`stats` teaches us something, the matching section is updated. The tool is
described in [`remote-ibus-reader.md`](remote-ibus-reader.md).

**Confidence markers.** Every fact carries one:

| Marker | Meaning |
|---|---|
| **[ours]** | verified on our hardware: our receiver, our Pi, our scope |
| **[community]** | stated by one or more sources in §8, not yet checked by us |
| **[unverified]** | an inference, a single unconfirmed report, or sources that disagree |

---

## 1. Overview

iBUS is FlySky's serial bus for receivers. A receiver can put all its channels
on one wire instead of one PWM wire per servo **[community]**.

The FS-iA6B has two iBUS connectors, and they carry two different protocols:

| Port | Direction | What it carries | Used here |
|---|---|---|---|
| **SERVO** (labelled i-BUS/S.BUS in FlySky's manual) | receiver to host, one-way | channel frames, continuously | **yes** |
| **SENS** | half-duplex, receiver polls | telemetry: the receiver asks sensors for values and relays them to the transmitter | no (§6) |

FlySky's FS-iA6B manual names both ports. It says the servo port can output
i-BUS or S.BUS and the SENS port is for i-BUS sensors **[community]**. The
details of each protocol come from the community sources.

**We use the SERVO port only**, receive-only. `IBusPort` opens the device
read-only, so nothing on the Pi can transmit on this line **[ours: by
construction]**.

Wiring: the receiver's iBUS SERVO signal goes to GPIO15 (RXD, physical pin
10). It appears as `/dev/serial0`, a symlink to `ttyAMA0` (the PL011 UART)
**[ours]**. Provisioning frees that UART from Bluetooth and the serial console
(`target/provision/base.sh`).

## 2. Physical and serial parameters

| Parameter | Value | Confidence |
|---|---|---|
| Signal level | high level about **3.35 V** | **[ours]** scope, SIG line |
| Idle state | **high** (standard UART idle, not inverted) | **[ours]** scope |
| Baud rate | 115200 | **[community]** all sources that state it agree; **[ours]** the Pi decodes valid frames at this rate |
| Data bits / parity | 8, none | **[community]**; **[ours]** consistent with the checksum passing |
| Stop bits | **1 (8N1)** | **[community]** see below; **[ours]** 8N1 decodes |
| Frame length | 32 bytes | **[community]**; **[ours]** |
| Frame interval | **7.70 ms** (129.9 frames/s), very regular | **[ours]** `stats`, below; **[community]** basejunction also measured about 7.7 ms; other sources say "about 7 ms", possibly describing the SENS port's poll cycle |
| Byte time | about 87 µs at 8N1; a frame is about 2.8 ms on the wire | arithmetic |

**Stop bits: 8N1 or 8N2.** 8N2 is sometimes quoted for iBUS, but **none of
the sources fetched for this document says 8N2**. basejunction states 8n1
from its own capture. IBusBM opens the port 8N1 explicitly. Betaflight's
receiver code passes no stop-bit flag, and its default is one stop bit (our
reading of its flag definitions). Betaflight's telemetry docs page states no
framing. 8N1 works on our setup **[ours]**.

The difference is mostly harmless in one direction. A UART set for one stop
bit treats a second stop bit as idle line, so an 8N1 reader accepts 8N2 data
**[unverified: general UART behaviour, not tested here]**. The reverse can
fail: an 8N2 reader would report framing errors whenever bytes arrive back to
back. That is why `SerialConfig` keeps 1 as the default, with `stopBits = 2`
(`--stop-bits 2`) available. The question stays open (§9) until a scope
measurement of the gap between bytes settles it.

**Timing at the Pi.** Frame stamps are taken when `read()` returns, so the
interval statistics include the Pi's scheduling jitter as well as the
receiver's own timing.

**Measured 2026-10-10** with `ibus_reader stats --seconds 30` on the Pi,
transmitter on, controls at rest **[ours]**:

| Measure | Value |
|---|---|
| Frames | 3896 in 30.0 s, **129.9 Hz** |
| Interval | min 7.66, **mean 7.70**, max 7.73 ms, std dev 0.01 ms |
| Checksum errors | 0 |
| Discarded bytes | 20, in 1 resync: the partial frame in flight when the port opened |
| Bytes received | 124692 = 3896 × 32 + 20, so **the SERVO line carries nothing but channel frames** |
| High nibble | never set |

Even with read-time stamps the interval spread is only ±0.04 ms, so the
receiver's frame clock is very regular, and an idle Pi adds little jitter.

## 3. Packet layout

| Byte | Content | Notes |
|---|---|---|
| 0 | `0x20` | length of the whole frame (32), and the first header byte |
| 1 | `0x40` | command: servo/channel data |
| 2–3 | channel 1, little-endian | low byte first: 1500 = `0x05DC` arrives as `DC 05` |
| 4–5 | channel 2 | |
| … | … | |
| 28–29 | channel 14 | |
| 30–31 | checksum, little-endian | `0xFFFF − (sum of bytes 0..29)`, modulo 2¹⁶ |

All **[community]**. The parser decodes live frames on the Pi with exactly
this layout and checksum **[ours]**.

The sources agree that the first byte doubles as a length byte. IBusBM
accepts any length up to `0x20` followed by a command byte. Betaflight treats
`0x20` as a channel frame and `0x04` as a telemetry request, and does not check
byte 1. parse_rc_ibus requires `20 40` exactly. **We require `20 40`**: on the
SERVO port only channel frames are expected, and a stricter header means fewer
false starts while resyncing.

### Checksum, worked example

A frame captured from our receiver with `ibus_reader dump` on 2026-10-10,
transmitter on, controls at rest **[ours]**:

```
20 40 DE 05 DC 05 7E 04 DC 05 93 07 7D 07 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 52 F4
```

| Bytes | Hex | Meaning |
|---|---|---|
| 0–1 | `20 40` | header |
| 2–3 | `DE 05` | CH1 = `0x05DE` = 1502 |
| 4–5 | `DC 05` | CH2 = 1500 |
| 6–7 | `7E 04` | CH3 = `0x047E` = 1150 |
| 8–9 | `DC 05` | CH4 = 1500 |
| 10–11 | `93 07` | CH5 = `0x0793` = 1939 |
| 12–13 | `7D 07` | CH6 = `0x077D` = 1917 |
| 14–29 | `DC 05` × 8 | CH7–CH14 = 1500 (unused) |
| 30–31 | `52 F4` | checksum = `0xF452` |

The checksum, step by step:

1. Sum bytes 0–29 as unsigned bytes: `0x20 + 0x40` = 96 for the header, plus
   2893 for the 28 channel bytes, gives **2989** (`0x0BAD`).
2. Subtract from `0xFFFF`: `0xFFFF − 0x0BAD` = **`0xF452`**.
3. It is sent low byte first: `52 F4`. That matches bytes 30–31.

Equivalently, the sum of bytes 0–29 plus the 16-bit checksum is `0xFFFF`.
`IBusParserTest` decodes this exact frame.

### How a reader resyncs

The sources use two strategies:

- **Gap-based** (IBusBM, Betaflight): a silence on the line marks the next
  byte as the start of a frame. IBusBM uses a gap of 3 ms or more, Betaflight
  more than 500 µs **[community]**. This needs byte timestamps, which a
  `read()` of a tty does not give.
- **Content-based** (ours, and the approach parse_rc_ibus's README
  suggests): look for `20 40`, take 32 bytes, check the checksum. If anything
  fails, drop **one** byte and look again.

`IBusParser` is content-based. A `20 40` pair inside a frame (for example in
the checksum bytes) can look like a header after sync is lost. The checksum
catches it. A false start costs at most one frame. Unit tests
cover junk before a frame, partial frames at startup, frames split at every
possible byte boundary, and back-to-back frames.

## 4. Channel semantics

| Aspect | Value | Confidence |
|---|---|---|
| Nominal range | 1000–2000 | **[community]**; **[ours]** the first live run read 1039 and 2000 on two channels |
| Center | 1500 | **[community]**; **[ours]** two sticks read 1500 and 1502 at rest |
| Channel slots | 14, always present | **[community]** |
| Channels the i6X sends | 6 by default, configurable to 10 (System › Aux Switches) | **[community]** FS-i6X manual; **[ours]** 6 at the defaults, switches on none |
| Unused slots | read 1500 | **[community]** |

**What the transmitter does to the values** (FS-i6X manual, **[community]**,
to be measured with `identify` and `monitor`):

- **Trims and subtrim** move the center, and the endpoints with it.
- **End points** limit each direction's travel, so a channel may not reach
  1000 or 2000.
- **Reverse** swaps which direction is high.
- **Mixes** (for example elevon or V-tail) make one control move two
  channels. `identify` reports every channel that moved, not just the largest,
  so a mix shows up.

### The top 4 bits

A channel word is 16 bits, but values up to 2000 need only 11. Sources
disagree about the spare high bits:

- **Betaflight** keeps only the low 12 bits of each slot. It rebuilds
  channels 15–18 from the top 4 bits of the slots, which its comments say
  newer receivers use **[community]**.
- **IBusBM** and **parse_rc_ibus** use all 16 bits **[community]**.
- **IBusBM issue #24:** one user reports that with a failsafe value set,
  failsafe sets bit `0x2000`. For example, 1000 arrived as 9192 (`0x23E8`)
  **[unverified: a single report]**.

Our parser keeps all 16 bits. `channelValue()` and `channelHighNibble()`
separate the two halves, and `dump` and `monitor` show any non-zero high
nibble. **Robot code must not normalize a raw word without checking its high
nibble**: under the `0x2000` report, a failsafe throttle of 1000 would arrive
as 9192 and clamp to full scale.

### Our i6X channel map

Measured with `ibus_reader identify` on 2026-10-11 **[ours]**, transmitter
at its defaults (6 channels, mode 2), 4 s per control:

| Control on the FS-i6X | Channel | Range seen | Notes |
|---|---|---|---|
| right stick left-right | CH1 | 1016..2000 | also CH2 (1481..1581) |
| right stick up-down | CH2 | 1010..2000 |  |
| left stick up-down | CH3 | 1003..1996 |  |
| left stick left-right | CH4 | 1002..1999 |  |
| VrA | CH5 | 1000..1745 |  |
| VrB | CH6 | 1000..2000 |  |

- **The switches SwA–SwD are on no channel** at the defaults: moving each
  one moved nothing. CH7–CH14 read 1500 throughout.

With the i6X set to 10 channels (System setup › Aux switches, Ch 10) and
the switches assigned in Functions setup › Aux channels, measured the same
day **[ours]**:

| Control on the FS-i6X | Channel | Range seen | Notes |
|---|---|---|---|
| SwA | CH7 | 1000..2000 |  |
| SwB | CH8 | 1000..2000 |  |
| SwC | CH9 | 1000..2000 | 3-position; middle 1500 |
| SwD | CH10 | 1000..2000 |  |

This is now the transmitter's configuration. CH11–CH14 are unused.
SwA reads **1000 up, 2000 down**, and SwC reads **1500 in the middle**
(`monitor`) **[ours]**, inside `decodeSwitch3()`'s default Mid band
(1250–1750).
- The "also CH2" on CH1 is about 100 counts, most likely the stick moved a
  little off-axis by hand rather than a mix. No mix is configured.
- Ranges short of 1000 or 2000 (CH1 1016, CH2 1010, VrA 1745) are probably
  travel not reached by hand within the window, not end points. Not yet
  checked.
- The roles match the manual's mode 2 layout: right stick roll and pitch,
  left stick throttle and yaw **[community]**. The throttle stick stays
  where it is left (it held mid-travel through the §5 runs) **[ours]**.

## 5. Failsafe behaviour of our FS-iA6B

> ⚠️ **PARTLY VERIFIED.** Two hardware runs, transmitter switched off
> [ours]. **The frames never stop.** With failsafe Off, the values freeze
> and the loss is invisible. With a failsafe value set, that channel moves
> to it about 1 s after the loss, with no flag in the top bits. Radio loss
> is therefore only detectable through a failsafe value the robot
> recognizes. **But run 3, after the i6X was set to 10 channels, applied
> no failsafe value at all:** cause not yet known. Out of range is still to
> test.

**What the sources say [community, not verified by us]:**

- The receiver **keeps streaming** iBUS frames when the radio link is lost
  (IBusBM README; issues #24 and #35). The frames stop only if the wire or
  the receiver itself is disconnected (issues #35 and #58).
- What the frames contain then depends on the **transmitter's** failsafe
  setting (FS-i6X manual §8.3, per channel). **Off** means the receiver
  holds the last value it received. **A percentage** means it outputs that
  position.
- #24 reports that with every channel's failsafe Off, an FS-i6X and iA6B kept
  sending the last values after the transmitter was switched off: a frozen
  stream that looks healthy. **Our run 1 below confirms this for ours [ours].**
- No source we read describes a "no output" failsafe mode for the iA6B's iBUS
  port.

**If this holds for our receiver,** the stale-link rule (§7) only detects a
broken wire or a dead receiver. Detecting radio loss would then need a
failsafe value set on the transmitter that the robot recognizes (or a flag,
if the `0x2000` report holds). That is a design consequence of the sources,
**[unverified]**.

**Results.** Each run is the block `failsafe-test` printed, unedited.

#### failsafe-test, 2026-10-11 13:09

| Item | Result |
|---|---|
| Transmitter failsafe setting | all off |
| Link lost by | switching the transmitter off |
| Before the loss | 129.9 Hz; CH1-14: 1502 1500 1595 1500 1563 1562 1500 1500 1500 1500 1500 1500 1500 1500 |
| Frames after the loss | **continued**: 1299 frames in 10.0 s (129.9 Hz) |
| Values while lost | unchanged from rest |
| Detecting the loss | **not detectable**: frames and values carried on unchanged |
| Recovery | nothing to recover from |
| Verdict | CONTINUES WITH THE LAST VALUES (FROZEN). The stale rule cannot see this, and nothing in the data shows it. Unsafe for robot control as configured. If the transmitter was really off, set a failsafe value and test again. |

Times are from pressing Enter, which was matched to the switch-off by hand: allow a
few hundred ms. Checksum errors during the watch: 0.

Set-up for that run: CH3 (the throttle stick, by its value) and CH5, CH6
(the knobs, by their values) were off their rest positions, so a jump to
other values would have shown. None changed. The rate did not even shift:
129.9 Hz before and after.

#### failsafe-test, 2026-10-11 13:30

| Item | Result |
|---|---|
| Transmitter failsafe setting | CH3 -100% |
| Link lost by | switching the transmitter off |
| Before the loss | 129.9 Hz; CH1-14: 1502 1500 1623 1501 1563 1561 1500 1500 1500 1500 1500 1500 1500 1500 |
| Frames after the loss | **continued**: 1299 frames in 10.0 s (129.9 Hz) |
| Values while lost | CH3 1623 -> 1001 (+1028 ms) |
| Detecting the loss | **not by the stale rule**; values changed from 1028 ms after Enter |
| Recovery | not within 20 s of the switch-on prompt |
| Verdict | CONTINUES WITH FAILSAFE VALUES. The stale rule cannot see this; only recognising the failsafe values can. |

Times are from pressing Enter, which was matched to the switch-off by hand: allow a
few hundred ms. Checksum errors during the watch: 0.

Notes on that run [ours]:

- **About 1 s from loss to failsafe.** Until then the receiver sends the
  last values, exactly as in run 1. The robot keeps acting on the last
  stick positions for that long whatever it does.
- **No flag.** The verdict would have said FLAGGED had any channel's top 4
  bits been set. The `0x2000` report (IBusBM #24) does not hold for ours,
  at least in this configuration.
- **Only the channel with a value moved.** The other five, failsafe Off,
  held their last values.
- **"Recovery: not within 20 s" is the test's limit, not the receiver's.**
  On switch-on the FS-i6X beeped a throttle warning and would not transmit
  until the throttle stick was lowered. Lowered, the stick reads about
  1000, which is the failsafe value itself, and the test counts recovery
  only when the values return to rest. The tool now asks for the controls
  to be put back. It also shows a limit of the approach: **a failsafe value
  that a control can also produce cannot be told from that control.**

#### failsafe-test, 2026-10-11 18:19

| Item | Result |
|---|---|
| Transmitter failsafe setting | CH7 SwA up (disarm), CH3 -100% |
| Link lost by | switching the transmitter off |
| Before the loss | 129.9 Hz; CH1-14: 1502 1500 1606 1500 1164 1884 2000 1000 1500 1000 1500 1500 1500 1500 |
| Frames after the loss | **continued**: 1299 frames in 10.0 s (129.9 Hz) |
| Values while lost | unchanged from rest |
| Detecting the loss | **not detectable**: frames and values carried on unchanged |
| Recovery | nothing to recover from |
| Verdict | CONTINUES WITH THE LAST VALUES (FROZEN). The stale rule cannot see this, and nothing in the data shows it. Unsafe for robot control as configured. If the transmitter was really off, set a failsafe value and test again. |

Times are from pressing Enter, which was matched to the switch-off by hand: allow a
few hundred ms. Checksum errors during the watch: 0.

**Unexplained [ours]:** in this run **neither failsafe value was applied**,
not even CH3's, which run 2 applied after about 1 s. Between the two runs
the i6X went from 6 to 10 channels and CH7's failsafe was set. The cause is
not known yet; until it is, a failsafe value set on the transmitter cannot
be relied on.

**Still to run:**

| Run | State |
|---|---|
| Failsafe all Off, transmitter off | done, FROZEN (above) |
| Failsafe value set on a channel, transmitter off | done, FAILSAFE VALUES after about 1 s, no flag (above) |
| Out of range instead of off | to do |

## 6. Telemetry (SENS port): not used by this project

Summarized for completeness. **This project does not wire or use the SENS
port.** **[community]** throughout.

- It is a half-duplex bus. The receiver polls sensor addresses and relays
  their answers to the transmitter.
- Requests are 4 bytes: length, command, then a checksum computed the same way
  as on the SERVO port. The command's high nibble selects the request:
  `0x8_` discover, `0x9_` sensor type, `0xA_` measurement. The low nibble is
  the sensor address.
- Address 0 is the receiver's own voltage sensor. Discovery stops at the first
  address that does not answer.
- A sensor replies after a short delay, with the same length-command-data-
  checksum layout. Data is little-endian.
- Wiring a microcontroller onto it needs a diode (or a resistor) to merge TX
  and RX onto one wire.

Betaflight's telemetry docs page contains one checksum error: its example
gives `0x5E` where the arithmetic gives `0x58` (the research agent's
calculation, not re-checked).

## 7. Using iBUS for robot control

> **Design guidance, not protocol.** These are proposals for the robot's
> control code, to be revisited once §4 and §5 are measured.

**The stale-link rule.** The link is lost when **no valid frame has arrived
for 100 ms**, about 14 frames. Frames that fail the checksum do not count.
One implementation, `Heron::Remote::LinkMonitor`, is used by every program,
configurable with `--stale-ms`. Robot code reads it through
`RemoteControl::getLatest()`, which judges the link when called
([`remote-cpp-design.md`](remote-cpp-design.md)). On loss the robot stops. **This catches a dead
wire or receiver. Whether it catches radio loss depends on §5.**

**Normalization** (`heron/remote/Normalize.h`, pure functions, unit tested):

1. Check the high nibble first (§4) and treat a non-zero one as invalid, until
   §5 says what it means.
2. **Axes** (sticks): `normalizeAxis()` maps raw to −1000..+1000 with a
   configurable center, deadband and endpoints. The output rises from 0 at the
   deadband edge, so there is no step. Each side scales separately, so a
   trimmed center still reaches both ends. Values beyond an endpoint clamp.
   Proposed starting deadband: 20 counts (4% of half travel), to be tuned from
   `monitor`'s observed jitter at rest.
3. **Knobs** (VrA, VrB): `normalizeKnob()` maps to 0..1000, clamped.
4. **Switches**: `decodeSwitch3()` gives LOW below 1250, HIGH above 1750,
   and MID in between. A two-position switch decodes as LOW or HIGH.

**Intended controls** (the assignment to channels waits for §4's map):

| Function | Control | Rule |
|---|---|---|
| Drive (forward/back) | a stick axis | `normalizeAxis` with deadband |
| Turn | the other axis of the same stick | `normalizeAxis` with deadband |
| Speed ceiling | a knob | `normalizeKnob`; scales the drive output, never adds to it |
| Arm | a two-position switch | motors enabled only while HIGH, and only if the switch was seen LOW since the link came up, so a robot cannot power up armed |
| Mode | a three-position switch | `decodeSwitch3` |

## 8. References and licenses

Retrieved 2026-10-10. **No code from any source is in this repository.**
Reusing any of it needs the maintainer's approval first, whatever its license.

| Source | URL | License | What we may do |
|---|---|---|---|
| basejunction, "FlySky i6 IBus: 14 channels!" part 1 (2015, edited 2016) | <https://basejunction.wordpress.com/2015/08/23/en-flysky-i6-14-channels-part1/> | unknown (none stated) | read-only reference |
| basejunction, "FlySky i6 part 2: Inside the Tx" (2015) | <https://basejunction.wordpress.com/2015/09/13/en-flysky-i6-part-2-inside-the-tx/> | unknown (none stated) | read-only reference |
| bmellink/IBusBM, Arduino library v1.1.4, and its issue tracker | <https://github.com/bmellink/IBusBM> | **copyleft, and inconsistent:** `LICENSE` file is GPL-3.0; source file headers say LGPL-2.1-or-later | read-only reference |
| Betaflight `src/main/rx/ibus.c`, `src/main/telemetry/ibus.c`, `ibus_shared.c` (master) | <https://github.com/betaflight/betaflight> | **GPL-3.0-or-later (copyleft)** | read-only reference |
| Betaflight docs, "Single Wire FlySky IBUS Telemetry" | <https://betaflight.com/docs/wiki/guides/current/IBus-telemetry> | unknown | read-only reference |
| Betaflight GitHub wiki, "Single-wire FlySky (IBus) telemetry" | <https://github.com/betaflight/betaflight/wiki/Single-wire-FlySky-(IBus)-telemetry> | — | **could not fetch**: the URL returned the repository page, not the wiki; the docs page above is used instead |
| parse_rc_ibus, Rust crate v0.2.0 | <https://docs.rs/parse_rc_ibus>, repository <https://github.com/sergeant-savage/parse_rc_ibus> | MIT | reusable under MIT with its copyright notice, **after asking**; read-only so far |
| endoflifecycle, "Flysky iA6 iBUS Setup" (2016) | <http://endoflifecycle.blogspot.com/2016/10/flysky-ia6-ibus-setup.html> | unknown (none stated) | read-only reference |
| FlySky, FS-i6X user manual, edition 2016-08-17 | <https://www.flysky-cn.com/s/FS-i6X-User-manual-20160817-6lm7.pdf> | unknown; © FlySky | read-only reference |
| FlySky, FS-iA6B user manual, edition 2024-03-27 (retailer-hosted copy) | <https://originhobbies.com/wp-content/uploads/2025/11/FS-iA6B-User-manual-20240327.pdf> | unknown; © Flysky Technology | read-only reference. The official page <https://www.flysky-cn.com/ia6b> returned 404 |

**Notes on individual sources:**

- **The older FS-iA6 is different, and does not apply to our iA6B.** It has
  no official serial output. The endoflifecycle post describes a hardware
  modification plus custom firmware to get one. Betaflight models the
  resulting stream as a separate variant: sync byte `0x55`, 31 bytes, and a
  checksum that is the plain sum of the channel words. The post itself gives
  no protocol details.
- **The FS-i6X manual edition read here** is bundled with the FS-iA6, not the
  iA6B. Its RX output-mode menu offers only PWM and PPM, with no serial
  i-BUS option. Later firmware is said to add one, but we read no official
  manual that confirms it. Our receiver does output iBUS **[ours]**, so ours
  has it.
- **basejunction** measured the stream itself but only guessed that the last
  two bytes were a checksum. The checksum definition comes from the code
  sources, which all agree, and from our parser decoding live frames.

## 9. Open questions

1. **Failsafe (§5):** our iA6B never stops sending on radio loss; it holds
   the last values, then about 1 s later applies any failsafe values, with
   no flag **[ours]**. Still open: whether out of range behaves the same as
   off, and which channel and value make a failsafe the robot can tell from
   a real control.
2. **Stop bits:** a scope measurement of the gap between bytes would show
   whether the receiver sends one stop bit or two.
3. **Interval under load:** 7.70 ms ± 0.04 ms on an idle Pi **[ours]**. How
   much does it spread with the LiDAR and the robot running?
4. **The channel map (§4)** is measured, with the switches on CH7–CH10.
   Open: confirming SwA as the arm switch with a failsafe run.
5. **The high nibble:** never set in 3896 frames at rest, and not set in
   failsafe either (§5) **[ours]**. Its meaning stays as in §4.
