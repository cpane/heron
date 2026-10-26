// File        : Drive.h
// Author      : Chris Pane
// Description : Remote sticks to a body velocity, and a body velocity to four
//               mecanum wheel values; with the arming rule that gates them.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The maths between the remote control and the motors, for driving Heron by
// hand. Every 20 ms the robot reads the remote, turns its sticks into how
// the robot should move, and turns that into four wheel values for Motors:
//
//   IBusFrame --sticksToVelocity()--> BodyVelocity --mix()--> four wheels
//   (RemoteControl)                   vx, vy, wz              (Motors::drive)
//
// Pure: no I/O, no time, no threads, so ctest checks every move
// (cpp/tests/DriveTest.cpp). heron_teleop runs the loop. The decisions
// behind it are docs/motor-design.md §4.
//
// The ideas behind it
// -------------------
//   Body velocity. How the whole robot should move, in REP 103: vx forward,
//   vy to the left, wz anticlockwise, each -1..1 of full drive. A holonomic
//   robot can do all three at once: drive, strafe and turn.
//
//   The sticks (FS-i6X, mode 2; docs/remote-ibus-protocol.md §4):
//     right stick up-down      CH2   vx: up is forward. Up reads LOW on our
//                                    transmitter (measured 2026-10-25), so
//                                    this axis is reversed: vx = -stick
//     right stick left-right   CH1   vy = -stick (right reads high; y is left)
//     left stick left-right    CH4   wz = -stick (right reads high, and turns
//                                    clockwise)
//   Which end of a stick reads high is the transmitter's setting, so each
//   axis has a reverse flag in StickConfig, set from what was measured.
//     left stick up-down       CH3   throttle, 0..1: a speed limit that
//                                    scales all three. It does not spring back
//   A dead zone around each stick's centre (30 counts, 3% of its travel)
//   keeps a stick resting slightly off centre from creeping.
//
//   Mixing, for mecanum wheels whose rollers form an X seen from above:
//     m1 = vx - vy - wz     front-left
//     m2 = vx + vy + wz     front-right
//     m3 = vx - vy + wz     back-right
//     m4 = vx + vy - wz     back-left
//   Combined moves can ask for more than full drive; then all four are
//   divided by the largest, which keeps the direction and gives up speed.
//   A wrong sign on vy or wz mirrors the robot without any error, so each
//   move is checked on the bench, wheel by wheel (docs/motor-design.md §4).
//
//   Arming. The remote's arm switch (SwA) and its link come to the robot as
//   RemoteSnapshot::isOk(). Because the throttle stays where it is left,
//   arming with it up would lurch the robot. ArmGate allows driving only
//   once the throttle has been seen at the bottom since the remote became
//   OK, the way drone flight controllers arm. Disarming, losing the radio
//   or a stale link closes it again.
//
// Usage
// -----
//   Heron::Drive::StickConfig sticks;            // our transmitter's defaults
//   Heron::Drive::ArmGate gate;
//
//   // every 20 ms
//   const auto r = remote.getLatest(now);
//   const double throttle = Heron::Drive::readThrottle(r.frame, sticks);
//   if (!gate.update(r.isOk(), throttle)) {
//     motors.stopMotors();                       // not armed, or throttle not yet down
//   } else {
//     motors.drive(Heron::Drive::mix(Heron::Drive::sticksToVelocity(r.frame, sticks)));
//   }

#ifndef HERON_DRIVE_DRIVE_H
#define HERON_DRIVE_DRIVE_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "heron/remote/IBusParser.h"
#include "heron/remote/Normalize.h"

namespace Heron::Drive {

/** How the robot should move, REP 103, each -1..1 of full drive. */
struct BodyVelocity {
  double vx{0.0};  // forward
  double vy{0.0};  // left
  double wz{0.0};  // anticlockwise
};

/** Four wheel values, -1..1, in protocol order: front-left, front-right,
 *  back-right, back-left. The same shape as Heron::Motor::WheelValues. */
using WheelValues = std::array<double, 4>;

/**
 * Which channel is which, and how sticks are read. The defaults are our
 * FS-i6X in mode 2 (docs/remote-ibus-protocol.md §4); slots are 0-based, so
 * CH1 is 0.
 */
struct StickConfig {
  std::size_t forwardChannel{1};   // CH2, right stick up-down
  std::size_t sideChannel{0};      // CH1, right stick left-right
  std::size_t turnChannel{3};      // CH4, left stick left-right
  std::size_t throttleChannel{2};  // CH3, left stick up-down
  /** Whether pushing a stick toward forward, left or anticlockwise reads
   *  LOW on its channel. Measured on our FS-i6X 2026-10-25 with
   *  heron_teleop: right stick up reads low on CH2; right stick left and
   *  left stick left read low on CH1 and CH4, as the sign flips below
   *  already expect for those two. */
  bool forwardReadsLow{true};
  bool leftReadsLow{true};
  bool anticlockwiseReadsLow{true};
  /** The three self-centring sticks: 1000..2000 about 1500, 30 counts of
   *  dead zone, 3% of the travel. */
  Heron::Remote::AxisConfig axis{1000, 1500, 2000, 30};
  std::int32_t throttleMin{1000};  // raw value at the bottom: throttle 0
  std::int32_t throttleMax{2000};  // raw value at the top: throttle 1
  /** Scales wz before mixing. 1.0 until tuned on the mini robot, where
   *  turning can be judged (decided 2026-10-25). */
  double turnGain{1.0};
};

/** @return The throttle, 0..1, from the frame */
[[nodiscard]] double readThrottle(const Heron::Remote::IBusFrame &frame,
                                  const StickConfig &config);

/**
 * The sticks as a body velocity, each axis scaled by the throttle.
 *
 * @throws std::invalid_argument if a channel slot is out of range, or the
 *         axis settings are invalid (normalizeAxis())
 */
[[nodiscard]] BodyVelocity sticksToVelocity(const Heron::Remote::IBusFrame &frame,
                                            const StickConfig &config);

/**
 * Mecanum mixing (X rollers): the body velocity as four wheel values. If
 * any would exceed 1 in size, all four are divided by the largest.
 *
 * @return Four values, each -1..1, in protocol order
 */
[[nodiscard]] WheelValues mix(const BodyVelocity &velocity);

/**
 * The arming rule: driving is allowed only once the throttle has been seen
 * at the bottom since the remote became OK. See the file header.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class ArmGate {
  public:
    /** At or below this the throttle counts as at the bottom: 5%. */
    static constexpr double kThrottleDown = 0.05;

    /**
     * Call once per control cycle.
     *
     * @param remoteOk RemoteSnapshot::isOk(): fresh, armed, not in failsafe
     * @param throttle readThrottle(), 0..1
     * @return true if the robot may drive this cycle
     */
    bool update(bool remoteOk, double throttle) noexcept {
      if (!remoteOk) {
        m_ready = false;
      } else if (throttle <= kThrottleDown) {
        m_ready = true;
      }
      return remoteOk && m_ready;
    }

    /** @return true while armed and ready: what the last update() returned */
    [[nodiscard]] bool isReady() const noexcept { return m_ready; }

  private:
    bool m_ready{false};
};

}  // namespace Heron::Drive

#endif  // HERON_DRIVE_DRIVE_H
