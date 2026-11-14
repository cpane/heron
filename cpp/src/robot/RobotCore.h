// File        : RobotCore.h
// Author      : Chris Pane
// Description : heron_robot's decisions, kept out of the program so they are
//               tested like the libraries' cores: the mode, the units, each
//               library's settings from the profile, and one cycle's choice.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// heron_robot (cpp/tools/HeronRobot.cpp) is heron_teleop grown up: the
// same remote, arming and mixing, plus the LiDAR, odometry, pose and the
// safety limiter, with SwC choosing the mode (docs/navigation-design.md §9,
// docs/navigation-cpp-design.md §7). The program owns the threads and the
// loop; every decision it makes is here, pure, so robot_test checks them on
// the Mac with no hardware:
//
//   SwC ----modeFromSwitch()----> Mode
//   sticks --sticksToVelocity()--> BodyVelocity (fractions of full drive)
//                                     |
//            Manual: as is ------------+------ Assisted: toTwist() -> the
//                                     |        limiter -> toBodyVelocity()
//                                     v
//                     decide() -> Decision: four wheel values, and why
//
// The modes (D5, design §9):
//
//   Manual    exactly heron_teleop: the sticks, mixed. No scan, no pose, no
//             limiter: it is the way out when anything else fails.
//   Assisted  the sticks through the limiter, which cuts any motion that
//             would bring the LiDAR's points inside the stop outline. Not
//             ready (a profile section missing, the blind spot not covered,
//             the LiDAR or OTOS not started) is stop, never Manual.
//   Navigate  the planner's motion through the limiter. Not built yet
//             (step 2): stop.
//
// Units (I2). The remote and the mixing work in fractions of full drive,
// -1..1; the limiter and the planner in m/s and rad/s. toTwist() and
// toBodyVelocity() are the one place the two meet, through [geometry]'s
// max_speed, min_speed and max_turn_rate. The bench rig's max_speed was
// measured driving forward (rig-calibration.md §8.6); sideways uses the
// same figure until a strafe speed is measured. The limiter then expects a
// strafe to go further than it does, which is the safe side.
//
// Speed is not in proportion to the command. The firmware lifts any wheel
// command above zero to the motor's dead band (min_duty), so on the floor
// the smallest command already moves the robot at min_speed; measured on
// the bench rig, 2026-11-14: 0.20, 0.39, 0.59 of full gave 0.19, 0.26,
// 0.30 m/s, where proportion said 0.08, 0.16, 0.25. So, per axis:
//
//   speed = 0 at 0, otherwise min_speed + (max_speed - min_speed) x |fraction|
//
// and the turn the same, its floor in the same proportion (not measured
// on its own). Back the other way, a speed under min_speed cannot be
// driven: it stops, unless upToMinSpeed, when it goes at min_speed, the
// least that moves. decide() asks for that only when the slow zone alone
// lowered the command; a cut stops. min_speed 0 is plain proportion.
//
// Usage, one cycle (HeronRobot.cpp has it in full):
//
//   const Mode mode = modeFromSwitch(swc, manualIsHigh);
//   const bool armed = gate.update(r.isOk(), Drive::readThrottle(r.frame, sticks));
//   const Drive::BodyVelocity stick = Drive::sticksToVelocity(r.frame, sticks);
//   AssistedInput assisted{&limiter, geometry, LimitInput{...}};
//   const Decision d = decide(mode, armed, stick, ready ? &assisted : nullptr);
//   armed ? motors.drive(d.wheels) : motors.stopMotors();

#ifndef HERON_ROBOT_ROBOT_CORE_H
#define HERON_ROBOT_ROBOT_CORE_H

#include <cstdint>
#include <optional>
#include <string>

#include "heron/drive/Drive.h"
#include "heron/geometry/Pose2.h"
#include "heron/lidar/Lidar.h"
#include "heron/limiter/Limiter.h"
#include "heron/odometry/OtosConfig.h"
#include "heron/profile/RigProfile.h"
#include "heron/remote/Normalize.h"

namespace Heron::Robot {

/** SwC's three positions (design §9). */
enum class Mode : std::uint8_t {
  Manual,
  Assisted,
  Navigate
};

/** SwC is CH9, slot 8 (motor-design.md §4, the i6X at 10 channels). */
inline constexpr std::size_t kModeChannel = 8;

/**
 * SwC's position as a mode: the middle is Assisted, one end Manual, the
 * other Navigate. Which end is up is not yet checked (design §18 item 4),
 * so the program takes it as a flag and prints the mode it reads.
 *
 * @param manualIsHigh true if SwC High is Manual
 */
[[nodiscard]] Mode modeFromSwitch(Heron::Remote::Switch3 swc, bool manualIsHigh) noexcept;

/** @return "MANUAL", "ASSISTED" or "NAVIGATE" */
[[nodiscard]] const char *toString(Mode mode) noexcept;

/** Fractions of full drive to m/s and rad/s, through min_speed and
 *  max_speed (vx and vy) and max_turn_rate (wz): header, "Units". */
[[nodiscard]] Geometry::Twist toTwist(const Drive::BodyVelocity &v,
                                      const Profile::GeometryProfile &g) noexcept;

/** m/s and rad/s back to fractions of full drive, each held to -1..1. A
 *  speed under min_speed (or the turn's floor) stops that axis, or with
 *  upToMinSpeed goes at the floor, the smallest fraction that moves. */
[[nodiscard]] Drive::BodyVelocity toBodyVelocity(const Geometry::Twist &t,
                                                 const Profile::GeometryProfile &g,
                                                 bool upToMinSpeed = false) noexcept;

// -- each library's settings, from the profile (I6) ---------------------------

/** The OTOS's mounting and scalars; the device and timings as the defaults. */
[[nodiscard]] Odometry::OtosConfig toOtosConfig(const Profile::OdometryProfile &o);

/** The LiDAR's housing yaw as the scan's mounting offset; the rest default.
 *  Its x and y are not the LiDAR's business: the limiter and perception
 *  place the points (toLimiterConfig()). */
[[nodiscard]] Lidar::LidarConfig toLidarConfig(const Profile::LidarProfile &l);

/**
 * The limiter's outline (length and width, halved), the LiDAR's place, the
 * margins and the brake.
 *
 * @throws std::invalid_argument if [geometry], [lidar] or [navigation] is
 *         missing; check assistedRefusal() first
 */
[[nodiscard]] Limiter::LimiterConfig toLimiterConfig(const Profile::RigProfile &profile);

/**
 * Whether this profile can run Assisted (§7.1): every one of [geometry],
 * [lidar], [odometry] and [navigation], and the LiDAR's blind circle inside
 * the stop outline (checkBlindSpot()).
 *
 * @return Empty if it can, otherwise why not, in a line for the status
 */
[[nodiscard]] std::string assistedRefusal(const Profile::RigProfile &profile);

// -- one cycle ----------------------------------------------------------------

/** What Assisted needs each cycle. Pointers are borrowed for the call. */
struct AssistedInput {
  const Limiter::Limiter *limiter{nullptr};
  Profile::GeometryProfile geometry{};
  /** The scan, the pose history and freshness, the measured velocity and
   *  now; its command is ignored: decide() puts the stick's there. */
  Limiter::LimitInput limit{};
};

/** Why the wheels are what they are this cycle. */
enum class Why : std::uint8_t {
  Disarmed,            // not armed, or the throttle not yet down: stopped
  Manual,              // the sticks, mixed
  Limited,             // Assisted: the limiter's answer; see Decision::limited
  AssistedUnavailable, // Assisted but not ready: stopped
  NavigateNotBuilt     // Navigate, step 2: stopped
};

/** @return "DISARMED", "MANUAL", "LIMITED", "ASSISTED_UNAVAILABLE" or
 *          "NAVIGATE_NOT_BUILT" */
[[nodiscard]] const char *toString(Why why) noexcept;

/** One cycle's choice. */
struct Decision {
  Drive::WheelValues wheels{};  // what to drive; all zero when stopped
  Drive::BodyVelocity body{};   // before mixing
  Why why{Why::Disarmed};
  std::optional<Limiter::LimitResult> limited;  // Assisted only
};

/**
 * One cycle, design §9's table. Manual reads nothing but the stick, so it
 * gives heron_teleop's wheels for the same frames (D5); Assisted runs the
 * stick through the limiter; Navigate stops until step 2.
 *
 * @param mode SwC's mode
 * @param armed ArmGate::update()'s answer this cycle
 * @param stick Drive::sticksToVelocity()'s answer this cycle
 * @param assisted What Assisted needs, or nullptr if it is not ready
 * @return The wheels and why
 */
[[nodiscard]] Decision decide(Mode mode, bool armed, const Drive::BodyVelocity &stick,
                              const AssistedInput *assisted);

}  // namespace Heron::Robot

#endif  // HERON_ROBOT_ROBOT_CORE_H
