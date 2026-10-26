// File        : MotorController.h
// Author      : Chris Pane
// Description : The motor firmware's logic: what each command does, the
//               watchdog, and the Status the Arduino reports. No Arduino code.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The Arduino's side of the motor link has two halves. main.cpp is the
// Arduino half: it owns the serial port, the clock and (from step 3) the
// motor driver. This class is the other half: the decisions. Given the
// frames that arrive and the time, it decides which wheel values should be
// applied, whether the motors run at all, and what the next Status says.
//
// It contains no Arduino code -- no Serial, no millis(), no pins -- so the
// same file is compiled twice: by PlatformIO into the firmware, and by CMake
// on the host, where cpp/tests/MotorControllerTest.cpp drives it with made-up
// frames and times. Every rule below is tested there, not on the bench.
//
// The rules
// ---------
//   Start-up      Stopped, reason Unconfigured: Drive is refused until a
//                 Configure is accepted.
//   Configure     The rig's calibration (MotorConfig): terminals, signs,
//                 ramp, dead band. Accepted only while stopped and only if
//                 isValidConfig(); then held, and its configChecksum() goes
//                 out in every Status. An unconfigured controller becomes
//                 stopped with reason Boot: ready, no Drive yet. Ignored
//                 otherwise, which the Pi sees as an unchanged checksum.
//   Drive         Refused while unconfigured (its sequence is still echoed).
//                 Otherwise running, with its four wheel values as the target (already
//                 clamped to -1000..1000 by decodeDrive()). Refreshes the
//                 watchdog. The applied values ramp toward the target.
//   Stop          Stopped at once, reason StopMessage, wheels zero: no ramp
//                 down, the motors are released and coast.
//   InfoRequest   Ask main.cpp to send a Boot (takeBootRequest()).
//   Watchdog      Running and no Drive for more than kWatchdogMs (250 ms):
//                 Stopped, reason Watchdog, and watchdogStops counts it. The
//                 target keeps the last commanded values, so Status shows
//                 what the motors were doing when the link went quiet. Like
//                 Stop, no ramp: the motors are released at once.
//
// Only Drive keeps the motors running. Stop, InfoRequest and Configure are commands
// too, and their sequence is echoed, but they do not feed the watchdog.
// A frame whose type is unknown, or whose payload is the wrong length, is
// ignored: the parser has already checked its checksum, so it can only come
// from a sender bug, not from noise.
//
// Target and applied
// ------------------
// "Target" is what the Pi last asked for; "applied" is what the motors
// should be getting right now. Both are in the protocol's units, -1000..1000,
// in protocol order, and both go out in every Status. They differ in two
// ways:
//
//   The ramp      While running, each applied value moves toward its target
//                 by at most the configured rampPerMs for every millisecond that passes, in
//                 update(). A step from 0 to 1000 at 4 per ms takes 250 ms;
//                 a reversal from +1000 to -1000 takes 500 ms, passing
//                 through zero. This limits the current surge a sudden
//                 change draws, which protects the L293D, and the jolt to
//                 the robot. rampPerMs 0 means no ramp: applied jumps to the
//                 target on the next update().
//   Stopping      Stop and the watchdog zero applied at once (above). Safety
//                 does not wait for a ramp.
//
// The applied value is still in wheel units, before any per-motor sign or
// dead band: those belong to the driver side, in MotorOutput.h, so Status
// reports what the protocol means rather than what the pins see.
//
// How main.cpp uses it
// --------------------
//   MotorController controller;
//   FrameParser parser;
//
//   void loop() {
//     const uint32_t now = millis();
//     while (Serial.available() > 0) {
//       parser.feed(static_cast<uint8_t>(Serial.read()),
//                   [&](const Frame &frame) { controller.onFrame(frame, now); });
//     }
//     controller.update(now);                  // the watchdog and the ramp
//     const MotorConfig &config = controller.getConfig();
//     for (size_t i = 0; i < kMotorCount; ++i) {
//       driver.set(config.terminal[i] - 1,
//                  toMotorOutput(controller.getApplied(i), ...));   // MotorOutput.h
//     }
//     if (controller.takeBootRequest()) { sendBoot(); }
//     if (controller.isStatusDue(now)) {
//       sendStatus(controller.makeStatus(parser.getStats(), battery, longestLoop));
//     }
//   }
//
// Time is a uint32_t of milliseconds, as millis() returns. It wraps after
// about 49 days; every comparison here subtracts, so the wrap is harmless.
//
// No C++ standard library: this is shared AVR code (CPP_CODING_STANDARD.md,
// "Shared code for the AVR"). The protocol is docs/motor-protocol.md; the
// firmware's layout is docs/motor-design.md §3.

#ifndef HERON_MOTOR_CONTROLLER_H
#define HERON_MOTOR_CONTROLLER_H

#include <stddef.h>
#include <stdint.h>

#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"

namespace Heron::Motor {

/** No Drive for longer than this, and the motors stop: about 12 frames. */
inline constexpr uint32_t kWatchdogMs = 250;

/** Status goes out this often: 10 Hz. */
inline constexpr uint32_t kStatusPeriodMs = 100;

/**
 * The firmware's decisions: commands in, wheel values and Status out.
 *
 * Thread Safety: Not thread-safe. The Arduino has one thread; on the host,
 * external synchronization is required.
 */
class MotorController {
  public:
    /**
     * Acts on one frame from the parser.
     *
     * @param frame A frame whose checksum and version are good
     * @param nowMs The time it arrived, in milliseconds
     */
    void onFrame(const Frame &frame, uint32_t nowMs);

    /**
     * Applies the watchdog, then moves the applied values along the ramp.
     * Call on every pass of the loop, after the frames that arrived in it.
     *
     * @param nowMs The time now, in milliseconds
     */
    void update(uint32_t nowMs);

    /**
     * @param nowMs The time now, in milliseconds
     * @return true once per kStatusPeriodMs, the first time at once. After a
     *         stall longer than a period it returns true once, not once for
     *         every period missed
     */
    bool isStatusDue(uint32_t nowMs);

    /** @return true once after each InfoRequest: send a Boot */
    bool takeBootRequest();

    /**
     * Builds the next Status from the controller's state and what only the
     * caller knows.
     *
     * @param link The parser's counters
     * @param batteryMillivolts The motor supply; 0 if no divider is fitted
     * @param longestLoopMicros The slowest loop pass since the last Status
     * @return The message, ready for encodeStatus()
     */
    [[nodiscard]] Status makeStatus(const ParserStats &link, uint16_t batteryMillivolts,
                                    uint16_t longestLoopMicros) const;

    [[nodiscard]] bool isRunning() const { return m_state == MotorState::Running; }

    /** @return true once a Configure has been accepted */
    [[nodiscard]] bool isConfigured() const { return m_configChecksum != 0; }

    /** @return The calibration held; MotorConfig's defaults while unconfigured */
    [[nodiscard]] const MotorConfig &getConfig() const { return m_config; }

    /**
     * @param motor 0..kMotorCount-1, in protocol order
     * @return What that motor should be getting now, -1000..1000
     */
    [[nodiscard]] int16_t getApplied(size_t motor) const { return m_applied[motor]; }

  private:
    void stop(StopReason reason);
    void ramp(uint32_t elapsedMs);
    void configure(const MotorConfig &config);

    MotorConfig m_config;
    uint16_t m_configChecksum{0};  // 0: unconfigured
    MotorState m_state{MotorState::Stopped};
    StopReason m_stopReason{StopReason::Unconfigured};
    uint8_t m_lastSequence{0};
    int16_t m_target[kMotorCount]{};
    int16_t m_applied[kMotorCount]{};
    uint32_t m_lastDriveMs{0};
    uint32_t m_lastUpdateMs{0};
    bool m_updateStarted{false};
    uint32_t m_nextStatusMs{0};
    bool m_statusStarted{false};
    bool m_bootRequested{false};
    uint16_t m_watchdogStops{0};
};

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_CONTROLLER_H
