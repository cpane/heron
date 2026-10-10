// File        : Normalize.cpp
// Author      : Chris Pane
// Description : Raw iBUS channel values to axes, knobs and switch positions.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/remote/Normalize.h"

#include <algorithm>
#include <stdexcept>

namespace Heron::Remote {

namespace {

// numerator * 1000 / denominator, rounded half away from zero. Both are
// non-negative here and small (16-bit range), so nothing can overflow.
std::int32_t scaleRounded(std::int32_t numerator, std::int32_t denominator) {
  return (numerator * kNormFullScale + denominator / 2) / denominator;
}

}  // namespace

std::int32_t normalizeAxis(std::uint16_t raw, const AxisConfig &config) {
  if (!(config.minRaw < config.center && config.center < config.maxRaw) ||
      config.deadband < 0 || config.deadband >= config.center - config.minRaw ||
      config.deadband >= config.maxRaw - config.center) {
    throw std::invalid_argument(
        "Heron::Remote::normalizeAxis() : need minRaw < center < maxRaw and "
        "0 <= deadband < each half");
  }
  const std::int32_t offset = static_cast<std::int32_t>(raw) - config.center;
  const std::int32_t magnitude = offset < 0 ? -offset : offset;
  if (magnitude <= config.deadband) {
    return 0;
  }
  const std::int32_t half = offset > 0 ? config.maxRaw - config.center
                                       : config.center - config.minRaw;
  const std::int32_t span = half - config.deadband;
  const std::int32_t past = std::min(magnitude - config.deadband, span);
  const std::int32_t value = scaleRounded(past, span);
  return offset > 0 ? value : -value;
}

std::int32_t normalizeKnob(std::uint16_t raw, std::int32_t minRaw, std::int32_t maxRaw) {
  if (minRaw >= maxRaw) {
    throw std::invalid_argument("Heron::Remote::normalizeKnob() : need minRaw < maxRaw");
  }
  const std::int32_t clamped = std::clamp(static_cast<std::int32_t>(raw), minRaw, maxRaw);
  return scaleRounded(clamped - minRaw, maxRaw - minRaw);
}

Switch3 decodeSwitch3(std::uint16_t raw, std::int32_t lowBelow, std::int32_t highAbove) {
  if (lowBelow > highAbove) {
    throw std::invalid_argument("Heron::Remote::decodeSwitch3() : need lowBelow <= highAbove");
  }
  const std::int32_t value = raw;
  if (value < lowBelow) {
    return Switch3::Low;
  }
  if (value > highAbove) {
    return Switch3::High;
  }
  return Switch3::Mid;
}

const char *toString(Switch3 position) noexcept {
  switch (position) {
    case Switch3::Low:
      return "LOW";
    case Switch3::Mid:
      return "MID";
    case Switch3::High:
      return "HIGH";
  }
  return "?";
}

}  // namespace Heron::Remote
