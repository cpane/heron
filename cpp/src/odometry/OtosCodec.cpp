// File        : OtosCodec.cpp
// Author      : Chris Pane
// Description : Register bytes to SI units and back, for the OTOS.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "OtosCodec.h"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace Heron::Odometry {

namespace {

// Scalings, one LSB in SI units (docs/odometry-otos-reference.md §3).
constexpr double kPi = std::numbers::pi;
constexpr double kInt16Span = 32768.0;
constexpr double kMetresPerLsb = 10.0 / kInt16Span;                  // +-10 m
constexpr double kRadiansPerLsb = kPi / kInt16Span;                  // +-pi
constexpr double kMetresPerSecondPerLsb = 5.0 / kInt16Span;          // +-5 m/s
constexpr double kRadiansPerSecondPerLsb = 2000.0 * kPi / 180.0 / kInt16Span;  // +-2000 deg/s
constexpr double kStandardGravity = 9.80665;
constexpr double kMetresPerSecond2PerLsb = 16.0 * kStandardGravity / kInt16Span;  // +-16 g
constexpr double kRadiansPerSecond2PerLsb = 1000.0 * kPi / kInt16Span;

// Byte offsets inside the 37-byte snapshot: status, then six blocks of
// three int16 words each.
constexpr std::size_t kStatusOffset = 0;
constexpr std::size_t kBlockLength = 6;
constexpr std::size_t kPoseOffset = 1;
constexpr std::size_t kVelocityOffset = kPoseOffset + kBlockLength;
constexpr std::size_t kAccelerationOffset = kVelocityOffset + kBlockLength;
constexpr std::size_t kPoseStdDevOffset = kAccelerationOffset + kBlockLength;
constexpr std::size_t kVelocityStdDevOffset = kPoseStdDevOffset + kBlockLength;
constexpr std::size_t kAccelerationStdDevOffset = kVelocityStdDevOffset + kBlockLength;
static_assert(kAccelerationStdDevOffset + kBlockLength == kSnapshotLength);

// Signed 16-bit little-endian (reference §3).
std::int16_t readInt16(std::span<const std::uint8_t, kSnapshotLength> bytes, std::size_t at) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(bytes[at]) |
                                   (static_cast<std::uint16_t>(bytes[at + 1]) << 8));
}

void writeInt16(std::array<std::uint8_t, kOffsetLength> &out, std::size_t at, std::int16_t value) {
  const auto word = static_cast<std::uint16_t>(value);
  out[at] = static_cast<std::uint8_t>(word & 0xFF);
  out[at + 1] = static_cast<std::uint8_t>(word >> 8);
}

Planar readBlock(std::span<const std::uint8_t, kSnapshotLength> bytes, std::size_t at,
                 double linearPerLsb, double angularPerLsb) {
  return Planar{readInt16(bytes, at) * linearPerLsb, readInt16(bytes, at + 2) * linearPerLsb,
                readInt16(bytes, at + 4) * angularPerLsb};
}

// A value in register units, rounded to nearest and checked to fit an int16.
std::int16_t toInt16(double value, double perLsb, const char *what) {
  const double raw = std::round(value / perLsb);
  if (!(raw >= -kInt16Span && raw <= kInt16Span - 1.0)) {
    throw std::invalid_argument(std::string("Heron::Odometry::encodeOffset() : ") + what +
                                " is out of range");
  }
  return static_cast<std::int16_t>(raw);
}

}  // namespace

OtosSnapshot decodeSnapshot(std::span<const std::uint8_t, kSnapshotLength> bytes) {
  OtosSnapshot s;
  s.status = bytes[kStatusOffset];
  s.pose = readBlock(bytes, kPoseOffset, kMetresPerLsb, kRadiansPerLsb);
  s.velocity = readBlock(bytes, kVelocityOffset, kMetresPerSecondPerLsb, kRadiansPerSecondPerLsb);
  s.acceleration =
      readBlock(bytes, kAccelerationOffset, kMetresPerSecond2PerLsb, kRadiansPerSecond2PerLsb);
  s.poseStdDev = readBlock(bytes, kPoseStdDevOffset, kMetresPerLsb, kRadiansPerLsb);
  s.velocityStdDev =
      readBlock(bytes, kVelocityStdDevOffset, kMetresPerSecondPerLsb, kRadiansPerSecondPerLsb);
  s.accelerationStdDev = readBlock(bytes, kAccelerationStdDevOffset, kMetresPerSecond2PerLsb,
                                   kRadiansPerSecond2PerLsb);
  return s;
}

OtosVersion decodeVersion(std::uint8_t raw) noexcept {
  return OtosVersion{static_cast<std::uint8_t>(raw >> 4), static_cast<std::uint8_t>(raw & 0x0F)};
}

std::uint8_t encodeScalar(double scalar) {
  if (!std::isfinite(scalar)) {
    throw std::invalid_argument("Heron::Odometry::encodeScalar() : scalar is not finite");
  }
  // std::round is half away from zero, for negative steps too.
  const double steps = std::round((scalar - 1.0) / kScalarStep);
  if (steps < -128.0 || steps > 127.0) {
    throw std::invalid_argument(
        "Heron::Odometry::encodeScalar() : scalar must be within 0.872 to 1.127");
  }
  return static_cast<std::uint8_t>(static_cast<std::int8_t>(steps));
}

double decodeScalar(std::uint8_t raw) noexcept {
  return 1.0 + static_cast<std::int8_t>(raw) * kScalarStep;
}

std::array<std::uint8_t, kOffsetLength> encodeOffset(const Planar &offset) {
  if (!std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.h)) {
    throw std::invalid_argument("Heron::Odometry::encodeOffset() : a value is not finite");
  }
  std::array<std::uint8_t, kOffsetLength> out{};
  writeInt16(out, 0, toInt16(offset.x, kMetresPerLsb, "x"));
  writeInt16(out, 2, toInt16(offset.y, kMetresPerLsb, "y"));
  // Wrapped first, so it always fits; +pi itself rounds to -pi's word.
  const double raw = std::round(wrapAngle(offset.h) / kRadiansPerLsb);
  writeInt16(out, 4, static_cast<std::int16_t>(raw >= kInt16Span ? -kInt16Span : raw));
  return out;
}

double wrapAngle(double rad) noexcept {
  const double wrapped = rad - 2.0 * kPi * std::floor((rad + kPi) / (2.0 * kPi));
  // floor() can leave exactly +pi through rounding; fold it to -pi.
  return wrapped >= kPi ? wrapped - 2.0 * kPi : wrapped;
}

double headingDelta(double from, double to) noexcept {
  return wrapAngle(to - from);
}

}  // namespace Heron::Odometry
