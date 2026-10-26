// File        : main.cpp
// Author      : Chris Pane
// Description : The motor firmware's Arduino half: serial port, clock,
//               motor driver, Boot and Status. The decisions are in
//               MotorController.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The program that runs on the Arduino Uno. It connects the hardware to
// MotorController, which holds every decision and is tested on the host:
//
//   Serial (USB, /dev/arduino on the Pi)
//     -> FrameParser            bytes to checked frames
//     -> MotorController        what each command means, the watchdog, the ramp
//     -> toMotorOutput()        applied value to duty and direction, with the
//                               rig's direction and dead band
//     -> L293DShield            the pins, through the rig's terminal map
//   and back:
//     -> Status and Boot        encoded with Protocol.h, written to Serial
//
// Anything here can only be tested on the bench, so this file does as little
// as possible: read bytes, read the clock, hand the applied values to the
// driver, write frames, and gather the few facts only the chip knows (reset
// cause, free RAM, loop time).
//
// Nothing rig-specific is compiled in. The rig's numbers -- terminal map,
// directions, dead band, ramp -- arrive from the Pi in a Configure after
// every Boot, from the rig's profile (docs/rig-profiles.md). Until then the
// controller refuses Drive and every motor stays released. One build serves
// every rig of the platform: the bench rig and the mini robot alike.
//
// With the shield's PWR jumper off and the motor battery on EXT_PWR, a
// configured Drive turns wheels. docs/RESUME-motor.md.
//
// Build and flash
// ---------------
//   scripts/build_firmware.sh                  in a container; no host install
//   scripts/flash_firmware.sh <user>@<pi>      avrdude on the Pi, to /dev/arduino
//
// The firmware version and the platform ID come from build flags in
// platformio.ini, one environment per platform, so Boot reports what was
// actually flashed.
//
// The Uno's limits, which shape this file
// ---------------------------------------
// - 2 KB of RAM: no allocation, fixed buffers, and free RAM reported in Boot.
// - One hardware serial port, which is the Pi link: there is no debug
//   console. What the firmware has to say goes in Status.
// - Serial.write() blocks once its 64-byte transmit buffer is full. A
//   Status frame is 39 bytes, about 3.4 ms at 115200 baud, so one Status
//   per 100 ms leaves the buffer mostly empty.

#include <Arduino.h>
#include <avr/io.h>

#include "L293DShield.h"
#include "MotorController.h"
#include "MotorOutput.h"
#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"

#ifndef HERON_FW_MAJOR
#error "HERON_FW_MAJOR, HERON_FW_MINOR, HERON_FW_PATCH and HERON_PLATFORM come from platformio.ini"
#endif

// avr-libc's heap bounds, for freeRam(). C symbols, so declared outside any
// namespace.
extern "C" {
extern int __heap_start;
extern int *__brkval;
}

namespace {

using namespace Heron::Motor;

constexpr uint32_t kBaud = 115200;

FrameParser s_parser;
MotorController s_controller;
L293DShield s_shield;
uint8_t s_resetFlags = 0;
uint16_t s_longestLoopMicros = 0;
uint32_t s_lastLoopMicros = 0;

// Bytes between the top of the heap and the bottom of the stack: the margin
// left in the 2 KB. The usual AVR idiom; the symbols come from avr-libc, and
// their int types are its, hence int here.
uint16_t freeRam() {
  int top = 0;
  const int heapEnd = (__brkval == nullptr) ? reinterpret_cast<int>(&__heap_start)
                                            : reinterpret_cast<int>(__brkval);
  return static_cast<uint16_t>(reinterpret_cast<int>(&top) - heapEnd);
}

void sendFrame(const uint8_t *frame, size_t length) {
  Serial.write(frame, length);
}

void sendBoot() {
  BootInfo boot;
  boot.firmwareMajor = HERON_FW_MAJOR;
  boot.firmwareMinor = HERON_FW_MINOR;
  boot.firmwarePatch = HERON_FW_PATCH;
  boot.protocolVersion = kProtocolVersion;
  boot.platform = static_cast<Platform>(HERON_PLATFORM);
  boot.resetFlags = s_resetFlags;
  boot.uptimeMillis = millis();
  boot.freeRamBytes = freeRam();
  uint8_t frame[kMaxFrameLen];
  sendFrame(frame, encodeBoot(boot, frame, sizeof(frame)));
}

void sendStatus() {
  // Battery: no divider is fitted on the bench yet, so 0 (docs/motor-design.md
  // §2, pin A0).
  const Status status = s_controller.makeStatus(s_parser.getStats(), 0, s_longestLoopMicros);
  uint8_t frame[kMaxFrameLen];
  sendFrame(frame, encodeStatus(status, frame, sizeof(frame)));
  s_longestLoopMicros = 0;  // the field is "since the previous Status"
}

// The applied values, through the rig's terminal map, directions and dead
// band, onto the shield. Unconfigured, every applied value is 0, so every
// motor is released. apply() writes only what changed.
void driveMotors() {
  const MotorConfig &config = s_controller.getConfig();
  for (size_t i = 0; i < kMotorCount; ++i) {
    const bool reversed = ((config.reversed >> i) & 1U) != 0;
    // Configure's terminals are 1..4 (M1..M4); the shield counts from 0.
    s_shield.set(static_cast<uint8_t>(config.terminal[i] - 1),
                 toMotorOutput(s_controller.getApplied(i), reversed, config.minDutyForward[i],
                               config.minDutyBackward[i]));
  }
  s_shield.apply();
}

}  // namespace

void setup() {
  // The motor pins first: until this runs they float (L293DShield.h).
  s_shield.begin();

  // MCUSR holds why the chip last reset; clear it so the next reset reports
  // only its own cause. On the bench Uno this reads 0: its bootloader runs
  // first and clears it [ours, measured 2026-10-25]. Kept for a board
  // whose bootloader leaves it alone.
  s_resetFlags = MCUSR & (kResetPowerOn | kResetExternal | kResetBrownOut | kResetWatchdog);
  MCUSR = 0;

  Serial.begin(kBaud);
  sendBoot();
  s_lastLoopMicros = micros();
}

void loop() {
  const uint32_t now = millis();

  while (Serial.available() > 0) {
    s_parser.feed(static_cast<uint8_t>(Serial.read()),
                  [now](const Frame &frame) { s_controller.onFrame(frame, now); });
  }
  s_controller.update(now);
  driveMotors();

  if (s_controller.takeBootRequest()) {
    sendBoot();
  }
  if (s_controller.isStatusDue(now)) {
    sendStatus();
  }

  // The time since the previous pass, so it includes everything above.
  const uint32_t nowMicros = micros();
  const uint32_t elapsed = nowMicros - s_lastLoopMicros;
  s_lastLoopMicros = nowMicros;
  if (elapsed > s_longestLoopMicros) {
    s_longestLoopMicros = elapsed > 0xFFFF ? 0xFFFF : static_cast<uint16_t>(elapsed);
  }
}
