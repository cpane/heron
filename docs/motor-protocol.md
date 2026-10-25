# Motor link protocol — Pi to Arduino

The wire format between the Pi and the Arduino motor controller, over USB
serial (`/dev/arduino`). **Version 1, written in `protocol/motor/` and
tested on the host (2026-10-25); not yet on a real link.** Nothing here has
been measured; when it is, tag the result **[ours]** as in the other
workstreams' references.

The protocol is the contract between the two sides, and both are written
here: the Pi's holonomic control and the Arduino firmware. It must work
unchanged on the bench rig, the mini robot and the full-size robot
([`motor-design.md`](motor-design.md) §6): nothing about pins, the board or
the motor driver reaches the wire.

## 1. What the Pi sends: four wheel values

- **Four values, one per motor, from −1.0 to 1.0.** The holonomic mixing
  runs on the Pi ([`motor-design.md`](motor-design.md) §4); the Arduino
  never sees a body velocity.
- **Order** (decided 2026-10-23), clockwise from front-left:

  | Index | Motor |
  |---|---|
  | 1 | front-left |
  | 2 | front-right |
  | 3 | back-right |
  | 4 | back-left |

  This is the protocol's order. The firmware maps each index to a shield
  terminal (M1–M4), so the wiring need not follow it.
- **Sign** (decided 2026-10-24): a positive value turns the wheel so that it
  drives the robot forward. A motor that spins the wrong way is fixed by
  swapping its two wires or by a per-index sign in the firmware, **never on
  the wire**.
- **1.0 means full PWM duty, open loop,** on all three platforms
  (decided 2026-10-25). The firmware maps |value| to a duty above its dead
  band (on the bench, for example, 70–255; the real band is measured per
  platform) plus a direction. It never tries to hold a speed. Speed
  control, when it comes, is closed on the Pi ([`motor-design.md`](motor-design.md)
  §4): the fields stay the same, only what the Pi feeds into the mixing
  changes.

## 2. Integers on the wire

Each value travels as a **signed 16-bit integer, little-endian, −1000 to
1000** for −1.0 to 1.0, 0.001 per step. The Pi code works in floats;
`heron_motor` sends round(x × 1000), clamped, and the firmware converts
back.

Why integers rather than floats:

- **One meaning on both sides.** A float needs agreement on format, byte
  order and NaN, and a corrupted float can arrive as NaN. An integer can
  only be out of range, and the rule for that is simple: clamp.
- **The AVR has no floating-point unit.** Float maths on the Uno is slow
  software.
- **No precision lost.** 0.001 is finer than any output here: the L293D
  shield's PWM has 255 steps. 0 and full scale are exact.
- **Smaller:** 8 bytes for four motors instead of 16.
- **Exact tests,** with no tolerances.

If finer steps are ever needed, the scale can grow to ±32767 under a new
protocol version (§3). Since speed control is closed on the Pi, not on the
Arduino (§1), that is unlikely.

## 3. Frames

Every message, both ways:

| Field | Size | Value |
|---|---|---|
| Start byte | 1 | `0xC5`, for resynchronising after noise |
| Length | 1 | Type, version and payload: 2 plus the payload length, at most 62 |
| Message type | 1 | Below |
| Protocol version | 1 | `1`. So a format change fails loudly, not silently |
| Payload | 0–60 | Per message type, §4 and §5 |
| Checksum | 2 | CRC-16/CCITT-FALSE over everything before it, start byte included |

Chosen 2026-10-25, when `protocol/motor/` was written:

- **Start byte `0xC5`:** not `0x00` or `0xFF`, which an idle or floating
  line produces; not printable ASCII; and not the RPLIDAR's `0xA5` or
  `0x5A`. The LiDAR's USB bridge is identical to the Arduino's, so if the
  ports were ever mixed up, a motor frame must not read as a LiDAR command.
- **Message types:** the high bit gives the direction.

  | Code | Message | Direction | Payload | Frame |
  |---|---|---|---|---|
  | `0x01` | Drive | Pi to Arduino | 9 | 15 |
  | `0x02` | Stop | Pi to Arduino | 1 | 7 |
  | `0x03` | Info request | Pi to Arduino | 1 | 7 |
  | `0x81` | Status | Arduino to Pi | 33 | 39 |
  | `0x82` | Boot | Arduino to Pi | 12 | 18 |

- **Checksum: CRC-16/CCITT-FALSE** (polynomial `0x1021`, initial value
  `0xFFFF`, no reflection, no final XOR; `"123456789"` gives `0x29B1`). A
  CRC rather than a sum because it also catches swapped and shifted bytes;
  the tests check that every single-bit error in a frame is caught. Computed
  bit by bit: a lookup table would take 512 of the Uno's 2 KB of RAM.
- **Payloads up to 60 bytes,** though version 1's largest is 33, so a longer
  frame from a later version is counted as a version mismatch, not
  discarded as noise.

**Resynchronising:** whenever the front of the buffer cannot start a valid
frame (no start byte, a length out of range, or a failed checksum), the
receiver drops one byte and looks again. A frame with a good checksum and
another version is counted in Status and not acted on.

The worked example: Drive, sequence 7, wheels 1000, −1000, 197, −1:

```
C5 0B 01 01 07 E8 03 18 FC C5 00 FF FF 16 98
```

`C5` start, `0B` length 11, `01` Drive, `01` version, `07` sequence, then
the four wheels (`C5 00` is 197: a start byte inside a payload is harmless,
since the checksum decides), then the CRC `0x9816`.

All multi-byte fields are little-endian. Counters only count up and wrap;
the receiver takes differences.

## 4. Pi to Arduino

### Drive

| Field | Size | Value |
|---|---|---|
| Sequence | 1 | Counts up per command, wrapping at 255; echoed in Status |
| Motor 1 to 4 | 4 × 2 | Signed 16-bit, −1000 to 1000 |

- **Rate** (decided 2026-10-24): every 20 ms, 50 Hz, **even when nothing
  has changed**.
- **Out of range clamps:** 1200 is applied as 1000, not rejected. Both the
  encoder and the decoder clamp, so the firmware never sees a value out of
  range.

### Stop

The header, a sequence and the checksum; no motor fields. All four motors
are released.

### Info request

The header, a sequence and the checksum. The Arduino answers with Boot
(§5).

## 5. Arduino to Pi

### Status, every 100 ms

| Field | Size | Value |
|---|---|---|
| Last sequence | 1 | Sequence of the newest valid command; the Pi measures round-trip time from it |
| State | 1 | 0 stopped, 1 running |
| Stop reason | 1 | 0 none (running), 1 boot (no command yet), 2 Stop message, 3 watchdog |
| Target, motor 1 to 4 | 4 × 2 | The values last commanded |
| Applied, motor 1 to 4 | 4 × 2 | Where the ramp has got to |
| Battery | 2 | Motor supply in millivolts, from a divider on A0; 0 if not fitted |
| Watchdog stops | 2 | Since boot |
| Frames received | 2 | Valid frames since boot |
| Checksum errors | 2 | Since boot |
| Version mismatches | 2 | Since boot |
| Bytes discarded | 2 | Skipped while resynchronising, since boot |
| Longest loop | 2 | Microseconds, since the previous Status |

About 40 bytes, 400 bytes a second at 10 Hz: under 4% of a 115 200-baud
link. The L293D gives no fault signal (its thermal shutdown is invisible to
the Arduino), so there is no fault field for the bench.

### Boot

Sent once after start-up, and again in reply to an Info request, for
example when the Pi opens the port after the Arduino is already running.

| Field | Size | Value |
|---|---|---|
| Firmware version | 3 | Major, minor, patch |
| Protocol version | 1 | The highest this firmware speaks |
| Platform ID | 1 | 0 unknown, 1 bench, 2 mini, 3 full-size |
| Reset cause | 1 | Bits: `0x01` power-on, `0x02` external, `0x04` brown-out, `0x08` watchdog. A brown-out means motor current pulled the Uno's supply down. The bits are the ATmega328P's MCUSR, so the Uno sends it as it is |
| Uptime | 4 | Milliseconds since boot |
| Free RAM | 2 | Bytes between the heap and the stack, of 2 KB |

## 6. The watchdog

**No valid command for 250 ms, and the Arduino stops the motors by itself**
(decided 2026-10-24: about 12 missed frames at 50 Hz). A dead Pi, a crashed
program or a pulled cable must not leave the motors running. It is the
motor-side twin of the remote's stale rule
([`remote-cpp-design.md`](remote-cpp-design.md) §3). Stop and the watchdog
both release all four motors; Status reports which happened and counts
watchdog stops.

## 7. Open questions

None. Answered 2026-10-25: what 1.0 means (full duty, open loop; §1); the
start byte, type codes and checksum (§3).

## 8. The code

`protocol/motor/include/heron/motor/`, header-only and compiled by both
sides:

- `Protocol.h`: the constants, the message structs, the CRC, and an encode
  and a decode function per message.
- `FrameParser.h`: bytes in, checked frames out through a callback, with
  the link counters Status reports.

No C++ standard library, so the AVR compiler can build it
([`motor-design.md`](motor-design.md) §3). On the host,
`heron_motor_protocol_nostd` compiles both headers with `-nostdinc++`, so
a standard header slipping in fails the host build. `motor_protocol_test`
(`ctest`) covers the worked example above, every message's round trip,
clamping, and the parser's resynchronising.
