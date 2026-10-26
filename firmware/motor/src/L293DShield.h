// File        : L293DShield.h
// Author      : Chris Pane
// Description : Driver for the bench's L293D motor shield (HW-130): four DC
//               motors, a PWM pin each, directions through a 74HC595.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The one file that knows how the bench's motor shield is wired. Everything
// above it speaks in MotorOutput (a duty and a direction, MotorOutput.h);
// this turns that into pin changes. A platform with another motor driver
// gets another class with the same three calls, and nothing above changes:
//
//   begin()                    pins to outputs, every motor released
//   set(terminal, output)      stage one motor's duty and direction
//   apply()                    write what changed to the pins
//
// The shield
// ----------
// The HW-130 is a copy of Adafruit's original (v1) Motor Shield. Each of its
// two L293D chips holds two H-bridges, so four motor terminals, M1..M4. An
// H-bridge needs two direction inputs and an enable per motor: twelve
// signals, more than the Uno can spare. The shield's answer:
//
//   Enables     One PWM pin per motor, straight from the Uno. The duty on
//               this pin is the motor's speed.
//   Directions  All eight direction inputs come from a 74HC595 shift
//               register, loaded serially through four Uno pins. Each motor
//               has an A bit and a B bit:
//                  A on, B off   forward (the driver's own forward)
//                  A off, B on   backward
//                  both off      released: the motor coasts
//
// The wiring, from Adafruit's v1 design and its library (AFMotor, public
// domain); checked on the bench's HW-130 2026-10-25, each terminal driving
// its own motor the right way (docs/RESUME-motor.md):
//
//   Terminal   PWM pin (timer)     74HC595 bits A, B
//   M1         11 (Timer 2, OC2A)  2, 3
//   M2          3 (Timer 2, OC2B)  1, 4
//   M3          6 (Timer 0, OC0A)  5, 7
//   M4          5 (Timer 0, OC0B)  0, 6
//
//   74HC595: data (SER) pin 8, clock (SRCLK) pin 4, latch (RCLK) pin 12,
//   output enable (/OE, active low) pin 7.
//
// PWM uses analogWrite(), so the timers keep the Arduino core's settings:
// about 490 Hz on M1 and M2, about 980 Hz on M3 and M4. Timer 0 also runs
// millis(), which the watchdog depends on; analogWrite() does not change
// its prescaler. The motors may whine at these frequencies; raising Timer
// 2's is a later option, Timer 0's is not.
//
// Before begin() runs -- while the bootloader waits, about 1.5 s after each
// reset -- every Uno pin is an input, so what the shield does then is up to
// its own resistors [unverified]. begin() is the first thing setup() does.
//
// Terminals are numbered 0..3 here for M1..M4. Which terminal each protocol
// motor uses, and which way round it is, is the rig's calibration, sent by
// the Pi in Configure (docs/rig-profiles.md), not this class's.

#ifndef HERON_MOTOR_L293D_SHIELD_H
#define HERON_MOTOR_L293D_SHIELD_H

#include <stdint.h>

#include "MotorOutput.h"

namespace Heron::Motor {

/**
 * The HW-130 L293D shield: four DC motor terminals.
 *
 * Thread Safety: Not thread-safe. Call from loop() only.
 */
class L293DShield {
  public:
    /** Motor terminals on the shield, M1..M4. */
    static constexpr uint8_t kTerminalCount = 4;

    /** Sets the pins up and releases every motor. Call first in setup(). */
    void begin();

    /**
     * Stages one motor's drive; nothing reaches the pins until apply().
     *
     * @param terminal 0..3 for M1..M4; anything else is ignored
     * @param output The duty and direction
     */
    void set(uint8_t terminal, const MotorOutput &output);

    /**
     * Writes what changed since the last apply(): the PWM pins whose duty
     * changed, and the direction register if any direction did. Cheap when
     * nothing changed, so it can run on every loop pass.
     */
    void apply();

  private:
    void writeDirections(uint8_t bits);

    uint8_t m_duty[kTerminalCount]{};
    uint8_t m_writtenDuty[kTerminalCount]{};
    uint8_t m_directionBits{0};
    uint8_t m_writtenDirectionBits{0};
};

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_L293D_SHIELD_H
