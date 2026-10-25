# Motor link protocol — Pi to Arduino

The wire format between the Pi and the Arduino motor controller, over USB
serial (`/dev/arduino`). **Agreed design, not yet implemented**
(2026-10-25). Nothing here has been measured; when it is, tag the result
**[ours]** as in the other workstreams' references.

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
- **Meaning of 1.0: open** (§7, question 1). On the bench it can only be
  full PWM duty, since the bench motors have no encoders.

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

If closed-loop control ever needs finer steps, the scale can grow to
±32767 under a new protocol version (§3).

## 3. Frames

Every message, both ways:

| Field | Size | Value |
|---|---|---|
| Start byte | 1 | A fixed marker, for resynchronising after noise |
| Length | 1 | Bytes that follow, up to the checksum |
| Message type | 1 | §4 and §5 |
| Protocol version | 1 | So a format change fails loudly, not silently |
| Payload | — | Per message type |
| Checksum | 2 | Over everything before it |

**Not yet chosen:** the start byte's value, the message type codes and the
checksum algorithm. Pick them when `protocol/motor/` is written, and
record them here.

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
- **Out of range clamps:** 1200 is applied as 1000, not rejected.

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
| State | 1 | Running or stopped |
| Stop reason | 1 | Stop message, watchdog, or boot (no command yet) |
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
| Platform ID | 1 | Bench, mini or full-size |
| Reset cause | 1 | The AVR's reset flags: power-on, external, brown-out, watchdog. A brown-out means motor current pulled the Uno's supply down |
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

1. **What does 1.0 mean on the robot?** Full PWM duty (open loop), or a
   fraction of a maximum wheel speed held by a control loop? Suggested:
   keep 1.0 as full duty on the Arduino, and close the loop on the Pi,
   comparing the commanded body velocity with the OTOS's measured velocity
   before mixing. Then no wheel encoders are needed, and the same speed in
   m/s means the same on every platform. The fields would not change, only
   their meaning, which must be written down here.
2. The start byte, type codes and checksum (§3).
