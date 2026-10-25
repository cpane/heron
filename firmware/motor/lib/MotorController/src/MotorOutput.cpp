// File        : MotorOutput.cpp
// Author      : Chris Pane
// Description : A wheel value turned into a PWM duty and a direction.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "MotorOutput.h"

#include "heron/motor/Protocol.h"

namespace Heron::Motor {

MotorOutput toMotorOutput(int16_t value, int8_t sign, uint8_t minDuty) {
  if (value > kWheelMax) {
    value = kWheelMax;
  } else if (value < kWheelMin) {
    value = kWheelMin;
  }
  if (value == 0) {
    return MotorOutput{};
  }

  const bool forward = (value > 0) != (sign < 0);
  // 1..1000, held in 32 bits: the product below reaches about 255 000, far
  // past the AVR's 16-bit int.
  const uint32_t magnitude = static_cast<uint32_t>(value > 0 ? value : -value);

  // 1 maps to minDuty and 1000 to 255, linearly between, rounded.
  const uint32_t span = static_cast<uint32_t>(kMaxDuty - minDuty);
  const uint32_t steps = static_cast<uint32_t>(kWheelMax - 1);  // 999
  const uint32_t duty = minDuty + ((magnitude - 1) * span + steps / 2) / steps;

  MotorOutput output;
  output.duty = static_cast<uint8_t>(duty);
  output.direction = forward ? Direction::Forward : Direction::Backward;
  return output;
}

}  // namespace Heron::Motor
