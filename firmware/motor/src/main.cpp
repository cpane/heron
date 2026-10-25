// File        : main.cpp
// Author      : Chris Pane
// Description : The motor firmware's Arduino half: serial port, clock,
//               Boot and Status. The decisions are in MotorController.
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
//     -> MotorController        what each command means, the watchdog
//     -> Status and Boot        encoded with Protocol.h, written to Serial
//
// Anything here can only be tested on the bench, so this file does as little
// as possible: read bytes, read the clock, write frames, and gather the few
// facts only the chip knows (reset cause, free RAM, loop time).
//
// Step 2 of the motor workstream: the link without the motors. Nothing here
// touches a motor pin, so a bug cannot turn a wheel. Step 3 adds the driver
// and drives controller.getApplied(). docs/RESUME-motor.md.
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

#include "MotorController.h"
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

}  // namespace

void setup() {
  // MCUSR holds why the chip last reset; clear it so the next reset reports
  // only its own cause. The Uno's bootloader runs first and may already
  // have cleared it, in which case this reads 0 [unverified on our board].
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
