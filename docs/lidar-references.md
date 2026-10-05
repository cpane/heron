# Reference documents

## 1. The protocol specification (authoritative)

**RPLIDAR 360 Degree Laser Range Scanner — Interface Protocol and Application
Notes**, rev 2.2, 2021-02-25. Applies to the RPLIDAR **A and S series**, so it
covers the A1M8. 50 pages, and it is complete: every command, every packet
format, every bit field.

Downloaded to `docs/vendor/LR001_SLAMTEC_rplidar_protocol_v2.2_en.pdf`
(git-ignored by default — see the note at the end).

Source:
<https://wiki.slamtec.com/download/attachments/83066883/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf>

SLAMTEC publishes several versions under similar names; there are also
S&C-series-specific editions (v2.8) that do **not** apply to the A1. Check the
cover page says "Applied to RPLIDAR A and S Series".

### Page index

Use this rather than hunting through the contents page.

| Page | Section |
|---|---|
| 4 | Basic communication mode, the three request/response modes |
| 6 | **Request packet format** — sync, command, payload, checksum |
| 7 | **Response packet format** — the 7-byte descriptor |
| 10 | Working states and transitions |
| 12 | Scan modes and measurement frequency |
| 13 | **Requests overview — the full command table** |
| 14 | STOP (note: wait ≥ 1 ms before the next request) |
| 15 | RESET |
| 15 | SCAN request and response |
| 18 | EXPRESS_SCAN request and response |
| 19 | — Legacy Version vs Extended Version |
| 22 | — **Capsuled** packet format (the 84-byte frame this repo decodes) |
| 25–27 | — **Ultra Capsuled** format, including the varbit/predictive encoding |
| 28–30 | — Dense Capsuled format |
| 30 | — Data processing, capsuled |
| 31 | — **Data processing, ultra capsuled** (the angle/distance reconstruction) |
| 32 | — Data processing, dense capsuled |
| 33 | FORCE_SCAN |
| 34 | GET_INFO |
| 36 | GET_HEALTH |
| 37 | GET_SAMPLERATE |
| 38–44 | GET_LIDAR_CONF and its configuration selectors |
| 44 | Motor speed control |
| 45 | Typical workflow for retrieving scan data |
| 46 | Calculating RPLIDAR scanning speed |

Pages 25–27 and 31 are the ones to read if you want the ultra-capsule modes
(Boost / Sensitivity / Stability), which this repository does **not** decode
and which are what your unit reports as its recommended mode.

### Cross-check against this repository

The official command table on page 13 matches `protocol.Command` exactly,
including which commands carry a payload and the minimum firmware version:

| Request | Value | Payload | Response mode | Min firmware |
|---|---|---|---|---|
| STOP | 0x25 | no | none | 1.0 |
| RESET | 0x40 | no | none | 1.0 |
| SCAN | 0x20 | no | multiple | 1.0 |
| EXPRESS_SCAN | 0x82 | **yes** | multiple | 1.17 |
| FORCE_SCAN | 0x21 | no | multiple | 1.0 |
| GET_INFO | 0x50 | no | single | 1.0 |
| GET_HEALTH | 0x52 | no | single | 1.0 |
| GET_SAMPLERATE | 0x59 | no | single | 1.17 |
| GET_LIDAR_CONF | 0x84 | **yes** | single | 1.24 |

The payload column is exactly the bit-7 rule (`CMDFLAG_HAS_PAYLOAD = 0x80`)
that `protocol.build_request()` enforces. This unit runs firmware 1.29, so
every command above is available to it.

## 2. The SDK source (the machine-readable spec)

<https://github.com/Slamtec/rplidar_sdk> — BSD-licensed C++. Often clearer
than the PDF because the structs *are* the spec:

| File | What it settles |
|---|---|
| `sdk/include/sl_lidar_cmd.h` | Command opcodes, response types, all packed structs, config selectors |
| `sdk/include/sl_lidar_protocol.h` | Sync bytes, `CMDFLAG_HAS_PAYLOAD`, the 30-bit size mask |
| `sdk/src/sl_lidarprotocol_codec.cpp` | Request framing and checksum, descriptor state machine |
| `sdk/src/dataunpacker/unpacker/handler_normalnode.cpp` | Legacy node decode |
| `sdk/src/dataunpacker/unpacker/handler_capsules.cpp` | Express, ultra and dense capsule decode |

**Version read for `docs/lidar-cpp-design.md`:** commit `99478e5` (2024-04-09),
`RPLIDAR_SDK_VERSION "2.0.0"`, BSD 2-clause. This is the version pinned as the
`third_party/rplidar_sdk` submodule. The layout has moved before and the notes
below are version-specific, so re-check them on any SDK update.

Three things that reading established and that are easy to get wrong:

- **`ascendScanData()` does not drop invalid samples.** It fabricates an angle
  for each one assuming uniform `360/count` spacing, then sorts. Spacing on
  this unit is *not* uniform, and the fabricated angles will paper over the
  silent buffer overrun if you gap-detect before filtering. See
  `lidar-cpp-design.md` §6.
- **The HQ node is Q14, not Q6.** `sl_lidar_response_measurement_node_hq_t`
  carries `angle_z_q14` (degrees = `q14 * 90 / 16384`), while the wire and this
  repository's Python are Q6. Distance stays Q2 in both.
- **`IChannel` is public and pure virtual**, so a `.rpraw` replay channel is
  straightforward — but `setMotorSpeed()` downcasts to `ISerialPortChannel*`
  gated only on `getChannelType()`, so a replay channel must either derive from
  `ISerialPortChannel` or not claim to be a serial port.

The most compact statement of the legacy node layout is three lines of
`sl_lidar_cmd.h`:

```c
sl_u8   sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
sl_u16  angle_q6_checkbit; // check_bit:1;angle_q6:15;
sl_u16  distance_q2;
```

Note the SDK reorganised in recent versions: capsule decoding moved out of
`rplidar_driver.cpp` into `sdk/src/dataunpacker/`. Older tutorials pointing at
`_capsuleToNormal()` in the driver are out of date.

## 3. The A1M8 datasheet (hardware, not protocol)

A separate document covering range, accuracy, optical and mechanical specs and
the electrical signal levels. Find it on <https://wiki.slamtec.com> under the
RPLIDAR A1 downloads. The protocol document explicitly defers to it for the
bottom-layer electrical definitions.

## 4. What this repository adds

`docs/rplidar-a1m8-findings.md` is not a substitute for the above — it records
what this *particular unit* actually does, which in several places is not what
a reader would assume from the spec: the scan stream is not in angular order,
the S flag fires near 358° rather than 0°, `quality == 0` means exactly "no
return", and buffer overrun is completely silent.

---

**Note on the vendored PDF:** `docs/vendor/` is git-ignored, so the PDF is
local to this checkout. Vendoring it is defensible — SLAMTEC moves download
URLs, and it is the document this code was audited against — but committing a
1.4 MB third-party file is a call for the repository owner, not a default.
