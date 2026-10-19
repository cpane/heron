// File        : OtosDevice.cpp
// Author      : Chris Pane
// Description : The OTOS start-up sequence and snapshot reads.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "OtosDevice.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

namespace Heron::Odometry {

namespace {

std::string hex(std::uint8_t value) {
  char text[8];
  std::snprintf(text, sizeof text, "0x%02X", value);
  return text;
}

void throwIf(std::error_code ec, const std::string &what) {
  if (ec) {
    throw std::system_error(ec, "Heron::Odometry::OtosDevice::start() : " + what);
  }
}

}  // namespace

void checkOtosConfig(const OtosConfig &config) {
  if (config.device.empty()) {
    throw std::invalid_argument("Heron::Odometry::checkOtosConfig() : device is empty");
  }
  if (config.pollPeriod <= Duration::zero() || config.staleAfter <= Duration::zero()) {
    throw std::invalid_argument(
        "Heron::Odometry::checkOtosConfig() : pollPeriod and staleAfter must be positive");
  }
  // The encoders hold the range rules; each throws std::invalid_argument.
  (void)encodeScalar(config.linearScalar);
  (void)encodeScalar(config.angularScalar);
  (void)encodeOffset(config.offset);
}

OtosStartup OtosDevice::start(const OtosConfig &config) {
  checkOtosConfig(config);
  OtosStartup up;

  // 1. Identify.
  std::array<std::uint8_t, kIdentityLength> identity{};
  throwIf(m_bus.read(kRegProductId, identity), "read of the product ID failed");
  if (identity[0] != kOtosProductId) {
    throw std::runtime_error("Heron::Odometry::OtosDevice::start() : product ID reads " +
                             hex(identity[0]) + ", not " + hex(kOtosProductId) +
                             ": not an OTOS");
  }
  up.hardware = decodeVersion(identity[1]);
  up.firmware = decodeVersion(identity[2]);

  // 2. IMU calibration: the board must be still.
  up.calibrationTime = writeAndWait(kRegImuCalibration, kCalibrationSamples,
                                    m_timing.calibrationTimeout, "IMU calibration");

  // 3. Scalars, both in one write, then read back: a scalar is the one
  //    setting that can be checked (the offset always reads 0, reference §6).
  const std::array<std::uint8_t, 2> scalars{encodeScalar(config.linearScalar),
                                            encodeScalar(config.angularScalar)};
  throwIf(m_bus.write(kRegLinearScalar, scalars), "write of the scalars failed");
  std::array<std::uint8_t, 2> readBack{};
  throwIf(m_bus.read(kRegLinearScalar, readBack), "read-back of the scalars failed");
  if (readBack != scalars) {
    throw std::runtime_error("Heron::Odometry::OtosDevice::start() : scalars read back " +
                             hex(readBack[0]) + " " + hex(readBack[1]) + ", written " +
                             hex(scalars[0]) + " " + hex(scalars[1]));
  }

  // 4. Offset, all six bytes in one write; a partial write is dropped
  //    silently (reference §8).
  const auto offset = encodeOffset(config.offset);
  throwIf(m_bus.write(kRegOffset, offset), "write of the offset failed");

  // 5. Reset tracking: pose to zero.
  up.resetTime = writeAndWait(kRegReset, kResetTracking, m_timing.resetTimeout, "reset");

  // 6. One read, to prove the snapshot path.
  throwIf(readSnapshot(up.first), "first snapshot read failed");
  return up;
}

std::error_code OtosDevice::readSnapshot(OtosSnapshot &out) {
  std::array<std::uint8_t, kSnapshotLength> bytes{};
  const std::error_code ec = m_bus.read(kRegStatus, bytes);
  if (!ec) {
    out = decodeSnapshot(bytes);
  }
  return ec;
}

Duration OtosDevice::writeAndWait(std::uint8_t reg, std::uint8_t value,
                                  std::chrono::milliseconds timeout, const char *what) {
  const TimePoint begin = Clock::now();
  const std::array<std::uint8_t, 1> data{value};
  throwIf(m_bus.write(reg, data), std::string("write for ") + what + " failed");
  std::array<std::uint8_t, 1> remaining{value};
  while (remaining[0] != 0) {
    if (Clock::now() - begin > timeout) {
      throw std::runtime_error("Heron::Odometry::OtosDevice::start() : " + std::string(what) +
                               ": " + hex(reg) + " still reads " + std::to_string(remaining[0]) +
                               " after " + std::to_string(timeout.count()) + " ms");
    }
    std::this_thread::sleep_for(m_timing.poll);
    throwIf(m_bus.read(reg, remaining), std::string("poll for ") + what + " failed");
  }
  return Clock::now() - begin;
}

}  // namespace Heron::Odometry
