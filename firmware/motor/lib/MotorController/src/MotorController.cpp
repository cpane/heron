// File        : MotorController.cpp
// Author      : Chris Pane
// Description : The motor firmware's logic: commands, the watchdog, Status.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "MotorController.h"

namespace Heron::Motor {

namespace {

// true if a is at or after b, across the millis() wrap: the difference read
// as signed is non-negative. Valid while the two are under 24 days apart.
bool isAtOrAfter(uint32_t a, uint32_t b) {
  return static_cast<int32_t>(a - b) >= 0;
}

// One applied value moved toward its target by at most maxStep.
int16_t stepToward(int16_t applied, int16_t target, int32_t maxStep) {
  const int32_t diff = static_cast<int32_t>(target) - applied;
  if (diff > maxStep) {
    return static_cast<int16_t>(applied + maxStep);
  }
  if (diff < -maxStep) {
    return static_cast<int16_t>(applied - maxStep);
  }
  return target;
}

}  // namespace

void MotorController::onFrame(const Frame &frame, uint32_t nowMs) {
  DriveCommand drive;
  StopCommand stopCmd;
  InfoRequest info;
  if (decodeDrive(frame, drive)) {
    m_lastSequence = drive.sequence;
    m_lastDriveMs = nowMs;
    if (m_state != MotorState::Running) {
      // Starting: the ramp runs from this Drive, not from the last update()
      // before it, so time spent stopped is not counted as ramp time.
      m_lastUpdateMs = nowMs;
      m_updateStarted = true;
    }
    m_state = MotorState::Running;
    m_stopReason = StopReason::None;
    for (size_t i = 0; i < kMotorCount; ++i) {
      m_target[i] = drive.wheels[i];  // applied follows in update(), on the ramp
    }
  } else if (decodeStop(frame, stopCmd)) {
    m_lastSequence = stopCmd.sequence;
    for (size_t i = 0; i < kMotorCount; ++i) {
      m_target[i] = 0;
    }
    stop(StopReason::StopMessage);
  } else if (decodeInfoRequest(frame, info)) {
    m_lastSequence = info.sequence;
    m_bootRequested = true;
  }
  // Anything else passed the checksum, so it is a sender bug, not noise:
  // ignored, as the header explains.
}

void MotorController::update(uint32_t nowMs) {
  if (m_state == MotorState::Running && nowMs - m_lastDriveMs > kWatchdogMs) {
    ++m_watchdogStops;
    stop(StopReason::Watchdog);  // the target keeps the last commanded values
  }

  const uint32_t elapsed = m_updateStarted ? nowMs - m_lastUpdateMs : 0;
  m_lastUpdateMs = nowMs;
  m_updateStarted = true;
  if (m_state == MotorState::Running) {
    ramp(elapsed);
  }
}

bool MotorController::isStatusDue(uint32_t nowMs) {
  if (!m_statusStarted) {
    m_statusStarted = true;
    m_nextStatusMs = nowMs + kStatusPeriodMs;
    return true;
  }
  if (!isAtOrAfter(nowMs, m_nextStatusMs)) {
    return false;
  }
  m_nextStatusMs += kStatusPeriodMs;
  if (isAtOrAfter(nowMs, m_nextStatusMs)) {
    // More than a whole period late: restart the schedule from now rather
    // than sending the missed ones back to back.
    m_nextStatusMs = nowMs + kStatusPeriodMs;
  }
  return true;
}

bool MotorController::takeBootRequest() {
  const bool requested = m_bootRequested;
  m_bootRequested = false;
  return requested;
}

Status MotorController::makeStatus(const ParserStats &link, uint16_t batteryMillivolts,
                                   uint16_t longestLoopMicros) const {
  Status status;
  status.lastSequence = m_lastSequence;
  status.state = m_state;
  status.stopReason = m_stopReason;
  for (size_t i = 0; i < kMotorCount; ++i) {
    status.target[i] = m_target[i];
    status.applied[i] = m_applied[i];
  }
  status.batteryMillivolts = batteryMillivolts;
  status.watchdogStops = m_watchdogStops;
  status.framesReceived = link.framesReceived;
  status.checksumErrors = link.checksumErrors;
  status.versionMismatches = link.versionMismatches;
  status.bytesDiscarded = link.bytesDiscarded;
  status.longestLoopMicros = longestLoopMicros;
  return status;
}

void MotorController::ramp(uint32_t elapsedMs) {
  // The widest move is -1000 to 1000. Capping the step there first keeps the
  // product below in range however long the loop stalled.
  const int32_t fullSwing = static_cast<int32_t>(kWheelMax) - kWheelMin;
  int32_t maxStep = fullSwing;
  if (m_ramp.rampPerMs != 0) {
    const uint32_t elapsed = elapsedMs > static_cast<uint32_t>(fullSwing)
                                 ? static_cast<uint32_t>(fullSwing)
                                 : elapsedMs;
    const uint32_t step = elapsed * m_ramp.rampPerMs;
    maxStep = step > static_cast<uint32_t>(fullSwing) ? fullSwing : static_cast<int32_t>(step);
  }
  for (size_t i = 0; i < kMotorCount; ++i) {
    m_applied[i] = stepToward(m_applied[i], m_target[i], maxStep);
  }
}

void MotorController::stop(StopReason reason) {
  m_state = MotorState::Stopped;
  m_stopReason = reason;
  for (size_t i = 0; i < kMotorCount; ++i) {
    m_applied[i] = 0;
  }
}

}  // namespace Heron::Motor
