// File        : Protocol.h
// Author      : Chris Pane
// Description : The Pi-to-Arduino motor link: frame layout, checksum, and
//               the encoding and decoding of each message.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Compiled by both sides: the Pi's heron_motor and the Arduino firmware. The
// AVR compiler has no C++ standard library, so this header uses only the C
// headers <stdint.h> and <stddef.h>, plain arrays, and no exceptions; a bad
// input is reported by a return value. That is the AVR exception in
// CPP_CODING_STANDARD.md. No I/O: bytes in, bytes out.
//
// On the AVR an int is 16 bits. A uint8_t promotes to that int, so
// `byte << 8` can overflow it; a uint16_t promotes to unsigned int. Every
// shift here therefore widens to uint16_t or uint32_t first. The host tests
// cannot catch a mistake there, since the host's int is 32 bits.
//
// The wire format, and why, is docs/motor-protocol.md.

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
inline constexpr uint8_t kProtocolVersion = 1;

inline constexpr size_t kLengthOffset = 1;
inline constexpr size_t kTypeOffset = 2;
inline constexpr size_t kVersionOffset = 3;
inline constexpr size_t kPayloadOffset = 4;
inline constexpr size_t kChecksumLen = 2;
/** Start, length, type, version and checksum: a frame with no payload. */
inline constexpr size_t kFrameOverhead = kPayloadOffset + kChecksumLen;

/** Room for every version-1 payload (Status is the largest, 33) and for growth,
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
  None = 0,         // running
  Boot = 1,         // no command since start-up
  StopMessage = 2,  // the Pi sent Stop
  Watchdog = 3      // no valid command for 250 ms
};

enum class Platform : uint8_t {
  Unknown = 0,
  Bench = 1,
  Mini = 2,
  FullSize = 3
};

/** Reset-cause bits in Boot. They match the ATmega328P's MCUSR, so the Uno
 *  sends the register as it is; another board maps its own flags onto these. */
inline constexpr uint8_t kResetPowerOn = 0x01;
inline constexpr uint8_t kResetExternal = 0x02;
inline constexpr uint8_t kResetBrownOut = 0x04;
inline constexpr uint8_t kResetWatchdog = 0x08;

// -- messages -------------------------------------------------------------------

/** Pi to Arduino, every 20 ms. */
struct DriveCommand {
  uint8_t sequence{0};
  int16_t wheels[kMotorCount]{};
};

/** Pi to Arduino: release all four motors. */
struct StopCommand {
  uint8_t sequence{0};
};

/** Pi to Arduino: answered with Boot. */
struct InfoRequest {
  uint8_t sequence{0};
};

/** Arduino to Pi, every 100 ms. Counters count up and wrap; take differences. */
struct Status {
  uint8_t lastSequence{0};  // of the newest valid command
  MotorState state{MotorState::Stopped};
  StopReason stopReason{StopReason::Boot};
  int16_t target[kMotorCount]{};   // as last commanded
  int16_t applied[kMotorCount]{};  // where the ramp has got to
  uint16_t batteryMillivolts{0};   // 0 if no divider is fitted
  uint16_t watchdogStops{0};
  uint16_t framesReceived{0};
  uint16_t checksumErrors{0};
  uint16_t versionMismatches{0};
  uint16_t bytesDiscarded{0};
  uint16_t longestLoopMicros{0};   // since the previous Status
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
inline constexpr size_t kStatusPayloadLen = 3 + 2 * 2 * kMotorCount + 7 * 2;
inline constexpr size_t kBootPayloadLen = 12;

static_assert(kStatusPayloadLen <= kMaxPayloadLen, "Status must fit a frame");

/**
 * One received frame, as the parser hands it over. payload points into the
 * parser's buffer and is valid only during the callback.
 */
struct Frame {
  MessageType type{MessageType::Drive};
  uint8_t version{0};
  const uint8_t *payload{nullptr};
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
                               status.longestLoopMicros};
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
                                &out.longestLoopMicros};
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
