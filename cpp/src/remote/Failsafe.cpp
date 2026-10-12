// File        : Failsafe.cpp
// Author      : Chris Pane
// Description : Radio-loss detection: the failsafe value on one channel.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/remote/Failsafe.h"

#include <stdexcept>
#include <string>

namespace Heron::Remote {

void checkFailsafeConfig(const FailsafeConfig &config) {
  if (config.channel >= kIBusChannelCount) {
    throw std::invalid_argument("Heron::Remote::checkFailsafeConfig() : channel index " +
                                std::to_string(config.channel) + " out of range, need < " +
                                std::to_string(kIBusChannelCount));
  }
  if (config.value > kIBusValueMask) {
    throw std::invalid_argument("Heron::Remote::checkFailsafeConfig() : value " +
                                std::to_string(config.value) + " does not fit in 12 bits");
  }
}

bool isFailsafe(const IBusFrame &frame, const FailsafeConfig &config) {
  checkFailsafeConfig(config);
  const std::uint16_t raw = frame.channels[config.channel];
  if (channelHighNibble(raw) != 0) {
    return true;
  }
  // int32: the difference of two 12-bit values, and a tolerance up to 0xFFFF.
  const std::int32_t offset =
      static_cast<std::int32_t>(channelValue(raw)) - static_cast<std::int32_t>(config.value);
  const std::int32_t distance = offset < 0 ? -offset : offset;
  return distance <= static_cast<std::int32_t>(config.tolerance);
}

}  // namespace Heron::Remote
