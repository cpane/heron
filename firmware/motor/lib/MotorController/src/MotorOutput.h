// File        : MotorOutput.h
// Author      : Chris Pane
// Description : A wheel value (-1000..1000) turned into what a motor driver
//               takes: a PWM duty and a direction. No Arduino code.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The last step before the pins. MotorController decides each motor's
// applied value, in the protocol's units: -1000..1000, positive drives the
// robot forward. A motor driver wants something else: how hard to push (a
// PWM duty, 0..255 on the Uno's 8-bit timers) and which way. This file is
// that conversion, and nothing more:
//
//   applied value  --toMotorOutput(value, sign, minDuty)-->  { duty, direction }
//                                                                   |
//                                       the driver (L293DShield) puts it on the pins
//
// Three things happen in it:
//
//   Sign       Each motor has a sign, +1 or -1, from the platform's
//              configuration. A motor wired so that "forward" on the driver
//              turns its wheel backwards gets -1. This is the firmware fix
//              for a wrong-way motor (docs/motor-protocol.md §1); the Pi
//              never knows.
//   Dead band  A DC motor does not turn at a low duty: it only hums. So any
//              non-zero value maps into minDuty..255, not 0..255:
//                    1  -> minDuty      (the slowest the motor really turns)
//                 1000  -> 255          (full duty)
//              and 0 means released, duty 0. minDuty is measured per
//              platform with motor_link sweep; 0 turns the dead band off.
//   Release    Zero is not "forward at duty 0" but Direction::Release: the
//              driver lets the motor coast. Stop and the watchdog end here.
//
// Example, with minDuty 70 and sign +1:
//
//   toMotorOutput(0, 1, 70)      -> { 0,   Release }
//   toMotorOutput(1, 1, 70)      -> { 70,  Forward }
//   toMotorOutput(500, 1, 70)    -> { 162, Forward }
//   toMotorOutput(-1000, 1, 70)  -> { 255, Backward }
//   toMotorOutput(-1000, -1, 70) -> { 255, Forward }   (a reversed motor)
//
// "Forward" here is the driver's own forward, whatever the wiring makes of
// it; the sign is what lines it up with the robot's forward.
//
// No C++ standard library: shared AVR code (CPP_CODING_STANDARD.md, "Shared
// code for the AVR"). Tested by cpp/tests/MotorControllerTest.cpp.

#ifndef HERON_MOTOR_OUTPUT_H
#define HERON_MOTOR_OUTPUT_H

#include <stdint.h>

namespace Heron::Motor {

/** Which way a driver pushes a motor. */
enum class Direction : uint8_t {
  Release = 0,  // not driven: the motor coasts
  Forward = 1,  // the driver's forward
  Backward = 2,
};

/** One motor's drive, as a driver takes it. */
struct MotorOutput {
  uint8_t duty{0};  // PWM duty, 0..255
  Direction direction{Direction::Release};
};

/** The highest PWM duty on the Uno's 8-bit timers: always on. */
inline constexpr uint8_t kMaxDuty = 255;

/**
 * Turns an applied wheel value into a duty and a direction.
 *
 * @param value The applied value, -1000..1000; clamped if outside
 * @param sign +1, or -1 for a motor wired backwards; anything negative counts as -1
 * @param minDuty The duty for the smallest non-zero value: the dead band. 0 for none
 * @return Release at duty 0 for a value of 0; otherwise a duty in minDuty..255
 */
[[nodiscard]] MotorOutput toMotorOutput(int16_t value, int8_t sign, uint8_t minDuty);

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_OUTPUT_H
