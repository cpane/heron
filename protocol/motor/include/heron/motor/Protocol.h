// File        : Protocol.h
// Author      : Chris Pane
// Description : The Pi-to-Arduino motor link: frame layout, checksum, and
//               the encoding and decoding of each message.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The Pi drives Heron's four wheels through an Arduino on a USB serial port
// (/dev/arduino). The Pi does all the motion maths and sends the Arduino
// four wheel values; the Arduino turns them into PWM on the motor driver.
// This header defines the bytes they exchange, and is the contract between
// the two programs: both compile this same file, so they cannot disagree.
//
// Messages
// --------
//   Pi to Arduino   Configure    the rig's motor calibration, after every Boot
//                   Drive        four wheel values, sent every 20 ms
//                   Stop         release all four motors now
//                   InfoRequest  "send me a Boot"
//   Arduino to Pi   Status       what the motors are doing, every 100 ms
//                   Boot         firmware version, platform and reset cause;
//                                once at start-up and on InfoRequest
//
// If the Arduino receives no valid Drive for 250 ms (a crashed program, a
// pulled cable), it stops the motors by itself: the watchdog. That is why
// the Pi sends Drive every 20 ms even when nothing has changed.
//
// The Arduino starts unconfigured
// -------------------------------
// The firmware holds no rig-specific numbers: which driver terminal each
// motor is on, which way round it is, where it starts to turn, how fast it
// may ramp. Those are measured per rig and kept in the rig's profile on the
// Pi (docs/rig-profiles.md). So after every reset the Arduino refuses Drive,
// its motors released, until the Pi sends Configure with the profile's motor
// numbers. Status then carries a checksum of what the Arduino holds, which
// the Pi compares with what it sent. A reset in the middle of a drive (a
// brown-out, say) therefore ends with the motors released, never running on
// defaults, until the Pi notices the new Boot and configures again:
//
//   Pi                                   Arduino
//   opens the port            ------->   resets, unconfigured
//                             <-------   Boot
//   Configure                 ------->   holds it, if stopped and valid
//                             <-------   Status, configChecksum = configChecksum(sent)
//   Drive, every 20 ms        ------->   drives
//
// Frames
// ------
// A serial port carries a stream of bytes with no boundaries, and noise or
// a dropped byte can corrupt it. So every message travels inside a frame:
//
//   start  length  type  version  payload ...  checksum
//    C5     0B      01    02      9 bytes      2 bytes
//
//   start     always 0xC5: where a frame may begin
//   length    how many bytes follow, up to the checksum (type, version and
//             payload), so the receiver knows where the frame ends
//   type      which message (MessageType)
//   version   kProtocolVersion; a frame from another version is ignored and
//             counted, so a mismatched Pi and firmware fail visibly
//   payload   the message's fields, multi-byte ones little-endian
//   checksum  CRC-16 over everything before it. A frame whose checksum does
//             not match is thrown away
//
// The receiver hunts for the start byte, reads the length, waits for the
// rest, and checks the CRC. If anything is wrong it drops one byte and hunts
// again, so it finds the next good frame after any corruption. That is
// FrameParser.h.
//
// How the pieces fit
// ------------------
// Sending, on either side: fill in a message struct, encode it into a byte
// buffer, write the bytes.
//
//   DriveCommand cmd;
//   cmd.sequence = next++;
//   cmd.wheels[0] = 500;                 // front-left at half duty, forward
//   uint8_t buf[kMaxFrameLen];
//   const size_t n = encodeDrive(cmd, buf, sizeof(buf));
//   write(fd, buf, n);                   // or Serial.write(buf, n) on the Arduino
//
// Receiving: feed the bytes to a FrameParser. It calls back with a Frame for
// each good one; a Frame is the checked frame's type and a pointer to its
// payload, still undecoded. Pass it to the decode function for the message
// you expect, which checks the type and the length and fills in the struct.
//
//   FrameParser parser;
//   parser.feed(byte, [&](const Frame &frame) {
//     DriveCommand cmd;
//     StopCommand stop;
//     if (decodeDrive(frame, cmd)) {
//       applyWheels(cmd.wheels);
//     } else if (decodeStop(frame, stop)) {
//       releaseMotors();
//     }
//   });
//
// Nothing here does I/O or keeps time: the caller reads and writes the port
// and runs the 20 ms and 250 ms timers. That is what lets the tests drive
// all of it with plain byte arrays (cpp/tests/MotorProtocolTest.cpp).
//
// Constraints, because the Arduino compiles it too
// --------------------------------------------------
// The AVR compiler has no C++ standard library, so this header uses only the
// C headers <stdint.h> and <stddef.h>, plain arrays, and no exceptions: a bad
// input is reported by a return value. That is the AVR exception in
// CPP_CODING_STANDARD.md.
//
// On the AVR an int is 16 bits. A uint8_t promotes to that int, so
// `byte << 8` can overflow it; a uint16_t promotes to unsigned int. Every
// shift here therefore widens to uint16_t or uint32_t first. The host tests
// cannot catch a mistake there, since the host's int is 32 bits.
//
// The wire format in full, field by field, with the reasons behind each
// choice, is docs/motor-protocol.md.

#ifndef HERON_MOTOR_PROTOCOL_H
#define HERON_MOTOR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace Heron::Motor {

// -- frame layout -------------------------------------------------------------
//
//   start  length  type  version  payload...  checksum (2, little-endian)
//
// length counts type, version and payload. The checksum covers every byte
// before it, the start byte included.

/** Not 0x00 or 0xFF (an idle or floating line), not printable ASCII, and not
 *  the RPLIDAR's 0xA5 or 0x5A: the two USB bridges are identical, so a mixed-up
 *  port must not turn a motor frame into a LiDAR command. */
inline constexpr uint8_t kStartByte = 0xC5;
/** 2 added Configure, the unconfigured state and Status's configuration
 *  checksum, and made Boot's platform the hardware design, not the rig. */
inline constexpr uint8_t kProtocolVersion = 2;

inline constexpr size_t kLengthOffset = 1;
inline constexpr size_t kTypeOffset = 2;
inline constexpr size_t kVersionOffset = 3;
inline constexpr size_t kPayloadOffset = 4;
inline constexpr size_t kChecksumLen = 2;
/** Start, length, type, version and checksum: a frame with no payload. */
inline constexpr size_t kFrameOverhead = kPayloadOffset + kChecksumLen;

/** Room for every payload (Status is the largest, 35) and for growth,
 *  so a longer frame from a newer version is counted as a version mismatch
 *  rather than discarded as noise. */
inline constexpr size_t kMaxPayloadLen = 60;
inline constexpr uint8_t kMinLengthField = 2;  // type and version, no payload
inline constexpr uint8_t kMaxLengthField = kMinLengthField + kMaxPayloadLen;
inline constexpr size_t kMaxFrameLen = kFrameOverhead + kMaxPayloadLen;

/** The high bit gives the direction: clear from the Pi, set from the Arduino. */
enum class MessageType : uint8_t {
  Drive = 0x01,
  Stop = 0x02,
  InfoRequest = 0x03,
  Configure = 0x04,
  Status = 0x81,
  Boot = 0x82
};

// -- values -----------------------------------------------------------------------

/** Motor order on the wire: 1 front-left, 2 front-right, 3 back-right,
 *  4 back-left, as indices 0 to 3. */
inline constexpr size_t kMotorCount = 4;

/** Wheel values: -1000..1000 for -1.0..1.0 of full duty. Positive drives the
 *  robot forward. */
inline constexpr int16_t kWheelMax = 1000;
inline constexpr int16_t kWheelMin = -1000;

/** @return value limited to kWheelMin..kWheelMax: out of range clamps, it is
 *  not rejected */
inline int16_t clampWheel(int16_t value) {
  if (value > kWheelMax) {
    return kWheelMax;
  }
  if (value < kWheelMin) {
    return kWheelMin;
  }
  return value;
}

enum class MotorState : uint8_t {
  Stopped = 0,
  Running = 1
};

enum class StopReason : uint8_t {
  None = 0,          // running
  Boot = 1,          // configured, no Drive yet
  StopMessage = 2,   // the Pi sent Stop
  Watchdog = 3,      // no valid command for 250 ms
  Unconfigured = 4   // no Configure since start-up: Drive is refused
};

/** The hardware design the firmware was built for: which motor driver it
 *  drives. Several rigs share one (the bench and the mini robot are both
 *  UnoL293d); which rig it is, is the Pi's business. */
enum class Platform : uint8_t {
  Unknown = 0,
  UnoL293d = 1  // Uno with the HW-130 L293D shield
};

/** Reset-cause bits in Boot. They match the ATmega328P's MCUSR, so the Uno
 *  sends the register as it is; another board maps its own flags onto these. */
inline constexpr uint8_t kResetPowerOn = 0x01;
inline constexpr uint8_t kResetExternal = 0x02;
inline constexpr uint8_t kResetBrownOut = 0x04;
inline constexpr uint8_t kResetWatchdog = 0x08;

// -- messages -------------------------------------------------------------------
//
// One struct per message, holding its fields as the program uses them. The
// encode*() and decode*() functions below turn each into its payload bytes
// and back; the order and size of every field on the wire is in
// docs/motor-protocol.md §4 and §5.
//
// Every message from the Pi carries a sequence number, which the Pi
// increments for each one it sends, wrapping at 255. The Arduino echoes the
// newest it accepted in Status, so the Pi can tell that its commands are
// arriving, and how long they take.

/**
 * Pi to Arduino, every 20 ms, even when nothing has changed: it is also the
 * heartbeat that keeps the watchdog from stopping the motors.
 *
 * wheels[] is in motor order (kMotorCount), each -1000..1000 for full
 * reverse to full forward duty, positive driving the robot forward.
 */
struct DriveCommand {
  uint8_t sequence{0};
  int16_t wheels[kMotorCount]{};
};

/** Pi to Arduino: release all four motors now, without waiting for the
 *  watchdog. The motors stay released until the next Drive. */
struct StopCommand {
  uint8_t sequence{0};
};

/** Pi to Arduino: asks for a Boot, for when the Pi starts or reconnects
 *  after the Arduino has been running and its first Boot was missed. */
struct InfoRequest {
  uint8_t sequence{0};
};

/** The fastest ramp Configure may ask for: -1000 to 1000 in 1 ms. */
inline constexpr uint16_t kMaxRampPerMs = 2000;

/**
 * Pi to Arduino, after every Boot: the rig's motor calibration, from the
 * [motor] section of its profile (docs/rig-profiles.md). Until one is
 * accepted the Arduino refuses Drive. It is accepted only while the motors
 * are stopped and only if isValidConfig() holds; otherwise it is ignored,
 * and the Pi sees that from configChecksum in Status.
 *
 * Arrays are in motor order, as DriveCommand's wheels.
 */
struct MotorConfig {
  uint8_t sequence{0};
  uint8_t terminal[kMotorCount]{1, 2, 3, 4};  // driver terminal, 1..4 for M1..M4, each once
  uint8_t reversed{0};       // bit i set: motor i+1 is wired backwards, so its
                             // driver direction is flipped
  uint16_t rampPerMs{0};     // 0..kMaxRampPerMs wheel units per ms; 0 is no ramp
  uint8_t minDutyForward[kMotorCount]{};   // PWM duty for the smallest value
  uint8_t minDutyBackward[kMotorCount]{};  // above 0 and below 0: the dead band
};

/**
 * Arduino to Pi, every 100 ms: what the motors are doing and how healthy
 * the link is. The Pi learns from it that the firmware is alive, whether its
 * commands arrive (lastSequence), and why the motors stopped if they did.
 *
 * Counters are 16 bits, count up since boot and wrap; the Pi takes the
 * difference between two Status messages, which stays right across a wrap.
 */
struct Status {
  uint8_t lastSequence{0};  // of the newest valid command
  MotorState state{MotorState::Stopped};
  StopReason stopReason{StopReason::Unconfigured};
  int16_t target[kMotorCount]{};   // as last commanded
  int16_t applied[kMotorCount]{};  // where the ramp has got to
  uint16_t batteryMillivolts{0};   // 0 if no divider is fitted
  uint16_t watchdogStops{0};
  uint16_t framesReceived{0};
  uint16_t checksumErrors{0};
  uint16_t versionMismatches{0};
  uint16_t bytesDiscarded{0};
  uint16_t longestLoopMicros{0};   // since the previous Status
  uint16_t configChecksum{0};      // configChecksum() of the configuration
                                   // held; 0 while unconfigured
};

/** Arduino to Pi, once after start-up and in reply to InfoRequest. */
struct BootInfo {
  uint8_t firmwareMajor{0};
  uint8_t firmwareMinor{0};
  uint8_t firmwarePatch{0};
  uint8_t protocolVersion{kProtocolVersion};  // the highest this firmware speaks
  Platform platform{Platform::Unknown};
  uint8_t resetFlags{0};  // kReset* bits
  uint32_t uptimeMillis{0};
  uint16_t freeRamBytes{0};
};

inline constexpr size_t kDrivePayloadLen = 1 + 2 * kMotorCount;
inline constexpr size_t kStopPayloadLen = 1;
inline constexpr size_t kInfoRequestPayloadLen = 1;
inline constexpr size_t kConfigurePayloadLen = 1 + kMotorCount + 1 + 2 + 2 * kMotorCount;
inline constexpr size_t kStatusPayloadLen = 3 + 2 * 2 * kMotorCount + 8 * 2;
inline constexpr size_t kBootPayloadLen = 12;

static_assert(kStatusPayloadLen <= kMaxPayloadLen, "Status must fit a frame");

/**
 * One received frame whose checksum and version were good, as FrameParser
 * hands it to its callback: the halfway point between bytes and a message.
 *
 * The parser knows framing but not messages, so it does not decode the
 * payload. The decode*() functions below do that: each checks that the type
 * and the payload length are what its message needs, and fills in the
 * struct. The type is not checked against the list in MessageType, so an
 * unknown type can arrive here; no decode function accepts it.
 *
 * payload points into the parser's buffer, which is reused for the next
 * frame: it is valid only during the callback. Decode it there, or copy it.
 */
struct Frame {
  MessageType type{MessageType::Drive};
  uint8_t version{0};               // always kProtocolVersion, for now
  const uint8_t *payload{nullptr};  // the bytes between version and checksum
  size_t payloadLen{0};
};

// -- little-endian fields -----------------------------------------------------------

inline uint16_t getU16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0] | static_cast<uint16_t>(static_cast<uint16_t>(p[1]) << 8));
}

/** Two's complement on every compiler this is built with (GCC, Clang). */
inline int16_t getI16(const uint8_t *p) {
  return static_cast<int16_t>(getU16(p));
}

inline uint32_t getU32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline void putU16(uint8_t *p, uint16_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>(value >> 8);
}

inline void putI16(uint8_t *p, int16_t value) {
  putU16(p, static_cast<uint16_t>(value));
}

inline void putU32(uint8_t *p, uint32_t value) {
  p[0] = static_cast<uint8_t>(value & 0xFF);
  p[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>(value >> 24);
}

// -- checksum ---------------------------------------------------------------------
//
// CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no reflection,
// no final XOR; "123456789" gives 0x29B1. A CRC rather than a sum because it
// also catches swapped and shifted bytes, which a sum does not. Bitwise rather
// than from a table: a table would take 512 of the Uno's 2 KB of RAM, and 46
// bytes at 8 steps each is a few microseconds.

inline constexpr uint16_t kCrcInit = 0xFFFF;
inline constexpr uint16_t kCrcPoly = 0x1021;

/** @return crc updated with one more byte */
inline uint16_t crc16Update(uint16_t crc, uint8_t byte) {
  crc = static_cast<uint16_t>(crc ^ static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8));
  for (uint8_t bit = 0; bit < 8; ++bit) {
    const bool top = (crc & 0x8000) != 0;
    crc = static_cast<uint16_t>(crc << 1);
    if (top) {
      crc = static_cast<uint16_t>(crc ^ kCrcPoly);
    }
  }
  return crc;
}

/** @return The CRC of count bytes from data */
inline uint16_t crc16(const uint8_t *data, size_t count) {
  uint16_t crc = kCrcInit;
  for (size_t i = 0; i < count; ++i) {
    crc = crc16Update(crc, data[i]);
  }
  return crc;
}

// -- encoding -----------------------------------------------------------------------
//
// Each encode*() writes one complete frame, ready to send, into the caller's
// buffer and returns its length. A buffer of kMaxFrameLen bytes always
// suffices; a smaller one that does not fit gets nothing written and a
// return of 0. encodeFrame() is the shared part; the others lay out their
// message's payload and call it.

/**
 * Writes a whole frame: header, payload and checksum.
 *
 * @param type The message type
 * @param payload payloadLen bytes; may be null when payloadLen is 0
 * @param payloadLen At most kMaxPayloadLen
 * @param out Where the frame goes
 * @param capacity Bytes available at out
 * @return The frame's length, or 0 if it does not fit or the payload is too long
 */
inline size_t encodeFrame(MessageType type, const uint8_t *payload, size_t payloadLen,
                          uint8_t *out, size_t capacity) {
  const size_t total = kFrameOverhead + payloadLen;
  if (payloadLen > kMaxPayloadLen || capacity < total) {
    return 0;
  }
  out[0] = kStartByte;
  out[kLengthOffset] = static_cast<uint8_t>(kMinLengthField + payloadLen);
  out[kTypeOffset] = static_cast<uint8_t>(type);
  out[kVersionOffset] = kProtocolVersion;
  for (size_t i = 0; i < payloadLen; ++i) {
    out[kPayloadOffset + i] = payload[i];
  }
  putU16(out + kPayloadOffset + payloadLen, crc16(out, kPayloadOffset + payloadLen));
  return total;
}

/** Wheel values are clamped as they are written. @return as encodeFrame() */
inline size_t encodeDrive(const DriveCommand &cmd, uint8_t *out, size_t capacity) {
  uint8_t payload[kDrivePayloadLen];
  payload[0] = cmd.sequence;
  for (size_t i = 0; i < kMotorCount; ++i) {
    putI16(payload + 1 + 2 * i, clampWheel(cmd.wheels[i]));
  }
  return encodeFrame(MessageType::Drive, payload, sizeof(payload), out, capacity);
}

/** @return as encodeFrame() */
inline size_t encodeStop(const StopCommand &cmd, uint8_t *out, size_t capacity) {
  const uint8_t payload[kStopPayloadLen] = {cmd.sequence};
  return encodeFrame(MessageType::Stop, payload, sizeof(payload), out, capacity);
}

/** @return as encodeFrame() */
inline size_t encodeInfoRequest(const InfoRequest &cmd, uint8_t *out, size_t capacity) {
  const uint8_t payload[kInfoRequestPayloadLen] = {cmd.sequence};
  return encodeFrame(MessageType::InfoRequest, payload, sizeof(payload), out, capacity);
}

/** The Configure payload without its sequence: what configChecksum() covers. */
inline void putConfigFields(const MotorConfig &config, uint8_t *p) {
  for (size_t i = 0; i < kMotorCount; ++i) {
    p[i] = config.terminal[i];
  }
  p[kMotorCount] = config.reversed;
  putU16(p + kMotorCount + 1, config.rampPerMs);
  for (size_t i = 0; i < kMotorCount; ++i) {
    p[kMotorCount + 3 + i] = config.minDutyForward[i];
    p[2 * kMotorCount + 3 + i] = config.minDutyBackward[i];
  }
}

/** @return as encodeFrame() */
inline size_t encodeConfigure(const MotorConfig &config, uint8_t *out, size_t capacity) {
  uint8_t payload[kConfigurePayloadLen];
  payload[0] = config.sequence;
  putConfigFields(config, payload + 1);
  return encodeFrame(MessageType::Configure, payload, sizeof(payload), out, capacity);
}

/**
 * The checksum Status reports for a configuration: CRC-16 over its fields,
 * the sequence left out, so the same calibration always gives the same
 * value. Never 0, which Status uses for "unconfigured": a CRC of 0 is
 * reported as 1.
 */
inline uint16_t configChecksum(const MotorConfig &config) {
  uint8_t fields[kConfigurePayloadLen - 1];
  putConfigFields(config, fields);
  const uint16_t crc = crc16(fields, sizeof(fields));
  return crc == 0 ? 1 : crc;
}

/**
 * @return true if every terminal is 1..kMotorCount and used once, no bit
 *         above the motors is set in reversed, and the ramp is at most
 *         kMaxRampPerMs. Any duty 0..255 is valid.
 */
inline bool isValidConfig(const MotorConfig &config) {
  bool used[kMotorCount + 1] = {};
  for (size_t i = 0; i < kMotorCount; ++i) {
    const uint8_t t = config.terminal[i];
    if (t < 1 || t > kMotorCount || used[t]) {
      return false;
    }
    used[t] = true;
  }
  const uint8_t motorBits = static_cast<uint8_t>((1U << kMotorCount) - 1U);
  return (config.reversed & static_cast<uint8_t>(~motorBits)) == 0 &&
         config.rampPerMs <= kMaxRampPerMs;
}

/** @return as encodeFrame() */
inline size_t encodeStatus(const Status &status, uint8_t *out, size_t capacity) {
  uint8_t payload[kStatusPayloadLen];
  payload[0] = status.lastSequence;
  payload[1] = static_cast<uint8_t>(status.state);
  payload[2] = static_cast<uint8_t>(status.stopReason);
  uint8_t *p = payload + 3;
  for (size_t i = 0; i < kMotorCount; ++i, p += 2) {
    putI16(p, status.target[i]);
  }
  for (size_t i = 0; i < kMotorCount; ++i, p += 2) {
    putI16(p, status.applied[i]);
  }
  const uint16_t counters[] = {status.batteryMillivolts, status.watchdogStops,
                               status.framesReceived,    status.checksumErrors,
                               status.versionMismatches, status.bytesDiscarded,
                               status.longestLoopMicros, status.configChecksum};
  for (const uint16_t value : counters) {
    putU16(p, value);
    p += 2;
  }
  return encodeFrame(MessageType::Status, payload, sizeof(payload), out, capacity);
}

/** @return as encodeFrame() */
inline size_t encodeBoot(const BootInfo &boot, uint8_t *out, size_t capacity) {
  uint8_t payload[kBootPayloadLen];
  payload[0] = boot.firmwareMajor;
  payload[1] = boot.firmwareMinor;
  payload[2] = boot.firmwarePatch;
  payload[3] = boot.protocolVersion;
  payload[4] = static_cast<uint8_t>(boot.platform);
  payload[5] = boot.resetFlags;
  putU32(payload + 6, boot.uptimeMillis);
  putU16(payload + 10, boot.freeRamBytes);
  return encodeFrame(MessageType::Boot, payload, sizeof(payload), out, capacity);
}

// -- decoding -----------------------------------------------------------------------
//
// Each returns false, leaving out untouched, if the frame is another type or
// its payload is not exactly the right length. The parser has already checked
// the checksum and the version.

/** Wheel values are clamped as they are read, so the firmware never sees one
 *  out of range. */
inline bool decodeDrive(const Frame &frame, DriveCommand &out) {
  if (frame.type != MessageType::Drive || frame.payloadLen != kDrivePayloadLen) {
    return false;
  }
  out.sequence = frame.payload[0];
  for (size_t i = 0; i < kMotorCount; ++i) {
    out.wheels[i] = clampWheel(getI16(frame.payload + 1 + 2 * i));
  }
  return true;
}

inline bool decodeStop(const Frame &frame, StopCommand &out) {
  if (frame.type != MessageType::Stop || frame.payloadLen != kStopPayloadLen) {
    return false;
  }
  out.sequence = frame.payload[0];
  return true;
}

inline bool decodeInfoRequest(const Frame &frame, InfoRequest &out) {
  if (frame.type != MessageType::InfoRequest || frame.payloadLen != kInfoRequestPayloadLen) {
    return false;
  }
  out.sequence = frame.payload[0];
  return true;
}

/** Decodes the fields as they are; whether they make sense is
 *  isValidConfig()'s question, so the firmware can refuse a bad one. */
inline bool decodeConfigure(const Frame &frame, MotorConfig &out) {
  if (frame.type != MessageType::Configure || frame.payloadLen != kConfigurePayloadLen) {
    return false;
  }
  const uint8_t *p = frame.payload + 1;
  out.sequence = frame.payload[0];
  for (size_t i = 0; i < kMotorCount; ++i) {
    out.terminal[i] = p[i];
    out.minDutyForward[i] = p[kMotorCount + 3 + i];
    out.minDutyBackward[i] = p[2 * kMotorCount + 3 + i];
  }
  out.reversed = p[kMotorCount];
  out.rampPerMs = getU16(p + kMotorCount + 1);
  return true;
}

inline bool decodeStatus(const Frame &frame, Status &out) {
  if (frame.type != MessageType::Status || frame.payloadLen != kStatusPayloadLen) {
    return false;
  }
  const uint8_t *p = frame.payload;
  out.lastSequence = p[0];
  out.state = static_cast<MotorState>(p[1]);
  out.stopReason = static_cast<StopReason>(p[2]);
  p += 3;
  for (size_t i = 0; i < kMotorCount; ++i, p += 2) {
    out.target[i] = getI16(p);
  }
  for (size_t i = 0; i < kMotorCount; ++i, p += 2) {
    out.applied[i] = getI16(p);
  }
  uint16_t *const counters[] = {&out.batteryMillivolts, &out.watchdogStops,
                                &out.framesReceived,    &out.checksumErrors,
                                &out.versionMismatches, &out.bytesDiscarded,
                                &out.longestLoopMicros, &out.configChecksum};
  for (uint16_t *const counter : counters) {
    *counter = getU16(p);
    p += 2;
  }
  return true;
}

inline bool decodeBoot(const Frame &frame, BootInfo &out) {
  if (frame.type != MessageType::Boot || frame.payloadLen != kBootPayloadLen) {
    return false;
  }
  const uint8_t *p = frame.payload;
  out.firmwareMajor = p[0];
  out.firmwareMinor = p[1];
  out.firmwarePatch = p[2];
  out.protocolVersion = p[3];
  out.platform = static_cast<Platform>(p[4]);
  out.resetFlags = p[5];
  out.uptimeMillis = getU32(p + 6);
  out.freeRamBytes = getU16(p + 10);
  return true;
}

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_PROTOCOL_H
