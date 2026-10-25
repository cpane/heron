// File        : L293DShield.cpp
// Author      : Chris Pane
// Description : Driver for the bench's L293D motor shield (HW-130).
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "L293DShield.h"

#include <Arduino.h>

namespace Heron::Motor {

namespace {

// The 74HC595 that holds the direction bits.
constexpr uint8_t kPinData = 8;     // SER
constexpr uint8_t kPinClock = 4;    // SRCLK: shifts one bit in per rising edge
constexpr uint8_t kPinLatch = 12;   // RCLK: copies the shifted byte to the outputs
constexpr uint8_t kPinEnable = 7;   // /OE: low turns the outputs on

// Per terminal, M1..M4. The header has the table and where it comes from.
constexpr uint8_t kPwmPin[L293DShield::kTerminalCount] = {11, 3, 6, 5};
constexpr uint8_t kBitA[L293DShield::kTerminalCount] = {2, 1, 5, 0};
constexpr uint8_t kBitB[L293DShield::kTerminalCount] = {3, 4, 7, 6};

uint8_t maskOf(uint8_t n) {  // not bit(): Arduino.h defines that as a macro
  return static_cast<uint8_t>(1U << n);
}

}  // namespace

void L293DShield::begin() {
  // The register first, all bits off, then turn its outputs on: no motor is
  // ever told to run while the pins are set up.
  pinMode(kPinData, OUTPUT);
  pinMode(kPinClock, OUTPUT);
  pinMode(kPinLatch, OUTPUT);
  pinMode(kPinEnable, OUTPUT);
  writeDirections(0);
  digitalWrite(kPinEnable, LOW);

  for (uint8_t t = 0; t < kTerminalCount; ++t) {
    pinMode(kPwmPin[t], OUTPUT);
    analogWrite(kPwmPin[t], 0);
    m_duty[t] = 0;
    m_writtenDuty[t] = 0;
  }
  m_directionBits = 0;
  m_writtenDirectionBits = 0;
}

void L293DShield::set(uint8_t terminal, const MotorOutput &output) {
  if (terminal >= kTerminalCount) {
    return;
  }
  const uint8_t both = static_cast<uint8_t>(maskOf(kBitA[terminal]) | maskOf(kBitB[terminal]));
  uint8_t bits = static_cast<uint8_t>(m_directionBits & ~both);
  uint8_t duty = output.duty;
  switch (output.direction) {
    case Direction::Forward:
      bits = static_cast<uint8_t>(bits | maskOf(kBitA[terminal]));
      break;
    case Direction::Backward:
      bits = static_cast<uint8_t>(bits | maskOf(kBitB[terminal]));
      break;
    case Direction::Release:
    default:
      duty = 0;  // released means no drive at all, whatever the duty said
      break;
  }
  m_directionBits = bits;
  m_duty[terminal] = duty;
}

void L293DShield::apply() {
  if (m_directionBits != m_writtenDirectionBits) {
    writeDirections(m_directionBits);
    m_writtenDirectionBits = m_directionBits;
  }
  for (uint8_t t = 0; t < kTerminalCount; ++t) {
    if (m_duty[t] != m_writtenDuty[t]) {
      analogWrite(kPwmPin[t], m_duty[t]);
      m_writtenDuty[t] = m_duty[t];
    }
  }
}

void L293DShield::writeDirections(uint8_t bits) {
  // Bit 7 goes in first, so after eight clocks it sits on output 7. The
  // outputs keep the old byte until the latch rises.
  digitalWrite(kPinLatch, LOW);
  shiftOut(kPinData, kPinClock, MSBFIRST, bits);
  digitalWrite(kPinLatch, HIGH);
}

}  // namespace Heron::Motor
