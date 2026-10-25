// File        : PlatformConfig.h
// Author      : Chris Pane
// Description : The settings that differ between Heron's platforms: ramp,
//               dead band, and which shield terminal and sign each motor has.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// One block of constants per platform (docs/motor-design.md §6), chosen by
// HERON_PLATFORM, which each environment in platformio.ini sets. The code
// that uses them is the same on every platform; only these numbers change.
//
//   kRampPerMs       How fast an applied value may change, in wheel units
//                    per millisecond (MotorController.h, "Target and
//                    applied"). 4 takes 0 to full in 250 ms.
//   kMinDuty         The dead band: the PWM duty for the smallest non-zero
//                    value, below which the motors only hum
//                    (MotorOutput.h). 0 turns it off.
//   kMotorTerminal   For each protocol motor (1 front-left, 2 front-right,
//                    3 back-right, 4 back-left), the shield terminal it is
//                    wired to: 0..3 for M1..M4.
//   kMotorSign       For each protocol motor, +1, or -1 if the driver's
//                    forward turns its wheel backwards. A wrong-way motor
//                    is fixed here or in the wiring, never on the wire
//                    (docs/motor-protocol.md §1).
//
// How to fill them in on a new platform (the bench checklist in
// docs/RESUME-motor.md):
//
//   1. Drive each protocol motor alone (motor_link drive 600 0 0 0, ...).
//      Note which wheel turns, and which way. Fix kMotorTerminal until
//      motor 1 turns the front-left wheel, and so on.
//   2. Any wheel that turns backwards for a positive value: -1 in
//      kMotorSign (or swap its two wires).
//   3. motor_link sweep on each motor with kMinDuty 0: the duty where the
//      slowest motor starts to turn is kMinDuty.
//   4. Watch Status while stepping the drive: tune kRampPerMs until starts
//      and reversals are firm without jolting.

#ifndef HERON_MOTOR_PLATFORM_CONFIG_H
#define HERON_MOTOR_PLATFORM_CONFIG_H

#include <stdint.h>

#include "heron/motor/Protocol.h"

namespace Heron::Motor {

#if HERON_PLATFORM == 1  // bench: the Uno and the HW-130 L293D shield

inline constexpr uint16_t kRampPerMs = 4;
// Not measured yet: 0 until motor_link sweep finds the dead band.
inline constexpr uint8_t kMinDuty = 0;
// Not checked yet: protocol motor n on terminal Mn, all the same way round,
// until the bench drives each motor alone.
inline constexpr uint8_t kMotorTerminal[kMotorCount] = {0, 1, 2, 3};
inline constexpr int8_t kMotorSign[kMotorCount] = {1, 1, 1, 1};

#else
#error "No PlatformConfig.h settings for this HERON_PLATFORM"
#endif

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_PLATFORM_CONFIG_H
