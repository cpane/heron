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
//   applied value  --toMotorOutput(value, reversed, minDutyForward,
//                                  minDutyBackward)-->  { duty, direction }
//                                                                   |
//                                       the driver (L293DShield) puts it on the pins
//
// Three things happen in it:
//
//   Reversed   A motor wired so that "forward" on the driver turns its
//              wheel backwards is marked reversed in the rig's profile, and
//              its driver direction is flipped. This is the firmware fix for
//              a wrong-way motor (docs/motor-protocol.md §1); the Pi's
//              values never change for it.
//   Dead band  A DC motor does not turn at a low duty: it only hums. So any
//              non-zero value maps into minDuty..255, not 0..255:
//                    1  -> minDuty      (the slowest the motor really turns)
//                 1000  -> 255          (full duty)
//              and 0 means released, duty 0. Each motor has its own minDuty
//              for each direction, measured per rig with motor_link sweep
//              and sent in Configure: minDutyForward for positive values,
//              minDutyBackward for negative ones. 0 turns the dead band off.
//   Release    Zero is not "forward at duty 0" but Direction::Release: the
//              driver lets the motor coast. Stop and the watchdog end here.
//
// Example, with minDutyForward 70 and minDutyBackward 60:
//
//   toMotorOutput(0, false, 70, 60)      -> { 0,   Release }
//   toMotorOutput(1, false, 70, 60)      -> { 70,  Forward }
//   toMotorOutput(500, false, 70, 60)    -> { 162, Forward }
//   toMotorOutput(-1, false, 70, 60)     -> { 60,  Backward }
//   toMotorOutput(-1000, false, 70, 60)  -> { 255, Backward }
//   toMotorOutput(-1000, true, 70, 60)   -> { 255, Forward }   (a reversed motor)
//
// "Forward" in the result is the driver's own forward, whatever the wiring
// makes of it; reversed is what lines it up with the robot's forward. The
// dead band follows the value's sign, the robot's direction, because that is
// how motor_link sweep measures it.
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
 * @param reversed true for a motor wired backwards: the direction is flipped
 * @param minDutyForward The duty for the smallest positive value: the dead band. 0 for none
 * @param minDutyBackward The same for negative values
 * @return Release at duty 0 for a value of 0; otherwise a duty from the
 *         value's minimum duty to 255
 */
[[nodiscard]] MotorOutput toMotorOutput(int16_t value, bool reversed, uint8_t minDutyForward,
                                        uint8_t minDutyBackward);

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_OUTPUT_H
