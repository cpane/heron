// File        : OtosConfig.h
// Author      : Chris Pane
// Description : What the robot tells the odometry library: the bus, the
//               correction scalars, the mounting offset and the timing.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The defaults are the uncorrected sensor with no offset. The robot fills in
// its measured values, as it fills in the remote's failsafe channel: the
// scalars measured so far are both 1.0 (docs/odometry-otos-reference.md §5),
// and the offset depends on where the sensor is mounted. Design:
// docs/odometry-cpp-design.md §2.1.

#ifndef HERON_ODOMETRY_OTOS_CONFIG_H
#define HERON_ODOMETRY_OTOS_CONFIG_H

#include <chrono>
#include <string>

#include "heron/odometry/Planar.h"

namespace Heron::Odometry {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

/**
 * Settings fixed for the life of an odometry instance.
 */
struct OtosConfig {
  std::string device{"/dev/i2c-1"};
  double linearScalar{1.0};    // 0.872 to 1.127: true / reported distance
  double angularScalar{1.0};   // 0.872 to 1.127: true / reported angle
  Planar offset{};             // the sensor's pose in the robot frame: m, m, rad
  Duration pollPeriod{std::chrono::milliseconds(10)};   // 100 Hz
  Duration staleAfter{std::chrono::milliseconds(100)};
};

/**
 * Checks a configuration without touching the bus.
 *
 * @param config The configuration to check
 * @throws std::invalid_argument if a scalar is outside 0.872..1.127, the
 *         offset's x or y is outside [-10, 10) m, a value is not finite, the
 *         device is empty, or a period is not positive
 */
void checkOtosConfig(const OtosConfig &config);

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_OTOS_CONFIG_H
