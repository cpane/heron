// File        : Drive.cpp
// Author      : Chris Pane
// Description : Sticks to a body velocity, and mecanum mixing.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/drive/Drive.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace Heron::Drive {

namespace {

std::uint16_t channel(const Heron::Remote::IBusFrame &frame, std::size_t slot) {
  if (slot >= frame.channels.size()) {
    throw std::invalid_argument("Heron::Drive::sticksToVelocity() : channel slot " +
                                std::to_string(slot) + " is out of range");
  }
  return frame.channels[slot];
}

// -1000..1000 from the remote library to -1..1.
double axis(const Heron::Remote::IBusFrame &frame, std::size_t slot,
            const Heron::Remote::AxisConfig &config) {
  return static_cast<double>(Heron::Remote::normalizeAxis(channel(frame, slot), config)) /
         Heron::Remote::kNormFullScale;
}

}  // namespace

double readThrottle(const Heron::Remote::IBusFrame &frame, const StickConfig &config) {
  return static_cast<double>(Heron::Remote::normalizeKnob(channel(frame, config.throttleChannel),
                                                          config.throttleMin,
                                                          config.throttleMax)) /
         Heron::Remote::kNormFullScale;
}

BodyVelocity sticksToVelocity(const Heron::Remote::IBusFrame &frame, const StickConfig &config) {
  const double throttle = readThrottle(frame, config);
  BodyVelocity v;
  // A stick pushed toward forward, left or anticlockwise reads low or high
  // depending on the transmitter; the config says which, per axis (header,
  // "The sticks").
  const auto toward = [](bool readsLow) { return readsLow ? -1.0 : 1.0; };
  v.vx = toward(config.forwardReadsLow) * throttle * axis(frame, config.forwardChannel, config.axis);
  v.vy = toward(config.leftReadsLow) * throttle * axis(frame, config.sideChannel, config.axis);
  v.wz = toward(config.anticlockwiseReadsLow) * throttle * config.turnGain *
         axis(frame, config.turnChannel, config.axis);
  return v;
}

WheelValues mix(const BodyVelocity &v) {
  WheelValues m = {
      v.vx - v.vy - v.wz,  // front-left
      v.vx + v.vy + v.wz,  // front-right
      v.vx - v.vy + v.wz,  // back-right
      v.vx + v.vy - v.wz,  // back-left
  };
  double largest = 1.0;
  for (const double value : m) {
    largest = std::max(largest, std::abs(value));
  }
  for (double &value : m) {
    value /= largest;  // keeps the direction when a combination saturates
  }
  return m;
}

}  // namespace Heron::Drive
