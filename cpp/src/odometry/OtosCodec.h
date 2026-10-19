// File        : OtosCodec.h
// Author      : Chris Pane
// Description : The OTOS register map, its scalings, and the conversions
//               between register bytes and SI units.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Pure, no I/O: the tests feed it bytes. Private to the odometry library
// (src/, not include/), so robot code never sees a register number; the
// public headers speak metres, radians and seconds only
// (docs/odometry-cpp-design.md §1).
//
// Every number here comes from docs/odometry-otos-reference.md, written from
// that reference and not from SparkFun's code (§9). The register map and
// scalings are §3; the start-up values §4; the scalar encoding §5.

#ifndef HERON_ODOMETRY_OTOS_CODEC_H
#define HERON_ODOMETRY_OTOS_CODEC_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "heron/odometry/Planar.h"

namespace Heron::Odometry {

// -- the device -----------------------------------------------------------------

/** 7-bit I2C address; fixed, no jumper (reference §3, measured). */
inline constexpr std::uint16_t kOtosAddress = 0x17;
/** What register 0x00 reads (reference §3). */
inline constexpr std::uint8_t kOtosProductId = 0x5F;

// -- registers ------------------------------------------------------------------

inline constexpr std::uint8_t kRegProductId = 0x00;
inline constexpr std::uint8_t kRegHardwareVersion = 0x01;
inline constexpr std::uint8_t kRegFirmwareVersion = 0x02;
inline constexpr std::uint8_t kRegLinearScalar = 0x04;   // then 0x05, angular
inline constexpr std::uint8_t kRegAngularScalar = 0x05;
inline constexpr std::uint8_t kRegImuCalibration = 0x06;
inline constexpr std::uint8_t kRegReset = 0x07;
inline constexpr std::uint8_t kRegOffset = 0x10;         // 6 bytes: x, y, h
inline constexpr std::uint8_t kRegStatus = 0x1F;

/** Status, then position, velocity, acceleration and their standard
 *  deviations: the read the reader thread makes each period, and the one
 *  probe O1 soaked (reference §3). */
inline constexpr std::size_t kSnapshotLength = 37;
/** Product ID, hardware and firmware version, from 0x00. */
inline constexpr std::size_t kIdentityLength = 3;
/** Offset x, y, h; all 6 bytes in one write, or the board drops it (§8). */
inline constexpr std::size_t kOffsetLength = 6;

// -- values written ------------------------------------------------------------

/** Samples for the IMU calibration; the register counts down to 0. */
inline constexpr std::uint8_t kCalibrationSamples = 255;
/** Register 0x07, bit 0: reset tracking, pose to zero. */
inline constexpr std::uint8_t kResetTracking = 0x01;

// -- status bits (register 0x1F) -------------------------------------------------

inline constexpr std::uint8_t kStatusTilt = 0x01;
inline constexpr std::uint8_t kStatusOpticalTracking = 0x02;
inline constexpr std::uint8_t kStatusOpticalChipError = 0x40;
inline constexpr std::uint8_t kStatusImuError = 0x80;

// -- scalars -------------------------------------------------------------------

/** One register step of a scalar. */
inline constexpr double kScalarStep = 0.001;
/** The range a scalar register can hold: 1 + int8 x 0.001. */
inline constexpr double kScalarMin = 1.0 + -128 * kScalarStep;  // 0.872
inline constexpr double kScalarMax = 1.0 + 127 * kScalarStep;   // 1.127

// -- decoded -------------------------------------------------------------------

/**
 * One 37-byte snapshot in SI units, in the board's world frame.
 */
struct OtosSnapshot {
  std::uint8_t status{0};
  Planar pose{};                // m, m, rad; heading in [-pi, pi)
  Planar velocity{};            // m/s, m/s, rad/s
  Planar acceleration{};        // m/s^2, m/s^2, rad/s^2
  Planar poseStdDev{};          // the board's filter statistic, same units
  Planar velocityStdDev{};
  Planar accelerationStdDev{};
};

/**
 * A version byte: high nibble major, low nibble minor (reference §3).
 */
struct OtosVersion {
  std::uint8_t major{0};
  std::uint8_t minor{0};
};

/**
 * @param bytes 37 bytes read from register 0x1F
 * @return The snapshot in SI units
 */
[[nodiscard]] OtosSnapshot decodeSnapshot(std::span<const std::uint8_t, kSnapshotLength> bytes);

/** @return The version a hardware or firmware version byte encodes */
[[nodiscard]] OtosVersion decodeVersion(std::uint8_t raw) noexcept;

/**
 * Encodes a scalar for register 0x04 or 0x05.
 *
 * Rounded to the nearest step, half away from zero, then stored as a
 * two's-complement int8. Plain truncation is one step off for some negative
 * corrections (reference §5).
 *
 * @param scalar The correction, true / reported
 * @return The register byte
 * @throws std::invalid_argument if scalar is not finite, or rounds outside
 *         0.872..1.127
 */
[[nodiscard]] std::uint8_t encodeScalar(double scalar);

/** @return The scalar a register byte holds, 0.872..1.127 */
[[nodiscard]] double decodeScalar(std::uint8_t raw) noexcept;

/**
 * Encodes the mounting offset for registers 0x10-0x15.
 *
 * x and y take the position scaling, so each must lie in [-10, 10) m; the
 * heading is wrapped to [-pi, pi) first, so any angle is accepted.
 *
 * @param offset The sensor's pose in the robot frame: m, m, rad
 * @return The 6 bytes, to be written in one transaction
 * @throws std::invalid_argument if a value is not finite, or x or y is out
 *         of range
 */
[[nodiscard]] std::array<std::uint8_t, kOffsetLength> encodeOffset(const Planar &offset);

/**
 * Wraps an angle to [-pi, pi), the range of the heading register.
 *
 * @param rad Any finite angle
 * @return The same direction, in [-pi, pi)
 */
[[nodiscard]] double wrapAngle(double rad) noexcept;

/**
 * The turn from one heading to the next, taken the short way round, for
 * unwrapping a heading read at intervals (as probe O4 does, reference §5).
 * Correct only while the true turn between the two reads is under half a
 * turn.
 *
 * @param from The earlier heading, rad
 * @param to The later heading, rad
 * @return to - from, in [-pi, pi)
 */
[[nodiscard]] double headingDelta(double from, double to) noexcept;

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_OTOS_CODEC_H
