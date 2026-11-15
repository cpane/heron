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
#include "heron/motion/MotionTable.h"
#include "heron/odometry/OtosConfig.h"
#include "heron/pose/PoseTracker.h"
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

/** How much faster than the table the limiter is told a command goes, so a
 *  table measured on a good day cannot make it late (design §19.2). */
inline constexpr double kTableMargin = 1.2;

/**
 * The active surface's motion table (design §19, P13): its points as
 * heron_motion's, mirrored as the surface says.
 *
 * @return no value without an active surface, or one with no points
 * @throws std::invalid_argument if its points form fewer than 2 columns
 */
[[nodiscard]] std::optional<Motion::MotionTable> toMotionTable(const Profile::RigProfile &profile);

/** The pose tracker's settings: plausibility (P16) from [geometry]'s
 *  max_speed, 1.5 times; the speed-jump check left off (its threshold
 *  needs the OTOS's 100 Hz noise, unmeasured). */
[[nodiscard]] Pose::PoseConfig toPoseConfig(const Profile::RigProfile &profile);

// -- one cycle ----------------------------------------------------------------

/**
 * P14, heading hold (I20). With the turn stick centred and the robot told
 * to translate, the turn rate that holds the heading taken when that
 * began: the heading error times a gain, less the drift the table
 * predicts, so the drift is cancelled before it shows. The turn stick
 * overrides it, and letting the stick go takes the new heading. Standing
 * still nothing is held, since a small turn alone would meet the motors'
 * dead band and lurch (the turn's floor, header "Units", is about 1.3
 * rad/s on the bench rig). A new pose epoch (I8) drops the heading held:
 * the old one is in another frame.
 *
 * Measured reason (design §19.1): a full strafe on the hardwood turned the
 * bench rig 8 to 12 degrees a second clockwise, and floor check 3 turned
 * it 19 degrees on the way to a box.
 *
 * The rate it returns is rad/s; holdFraction() turns it into the mixing's
 * wz for the translation actually driven.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class HeadingHold {
  public:
    struct Config {
      double gainPerS{2.0};      // rad/s commanded per radian of error
      double maxRateRadps{0.8};  // the most it turns, either way
    };

    HeadingHold() noexcept;  // Config's defaults
    explicit HeadingHold(Config config) noexcept;

    /**
     * One cycle.
     *
     * @param turnStickRadps the turn stick as a rate; non-zero overrides
     * @param translating whether the stick asks for any translation
     * @param headingRad the pose's heading, odom frame
     * @param driftRadps the table's predicted turn for this translation
     * @param epoch the pose's epoch; a change drops the heading held
     * @return the turn rate to command, rad/s: the stick's when it is
     *         used, 0 standing still
     */
    double update(double turnStickRadps, bool translating, double headingRad, double driftRadps,
                  std::uint64_t epoch) noexcept;

    /** Drops the heading held: on leaving Assisted or Navigate, and on
     *  disarming. */
    void reset() noexcept;

    /** @return whether a heading is held; targetRad() is then valid */
    [[nodiscard]] bool holding() const noexcept { return m_holding; }

    /** @return the heading held, odom frame */
    [[nodiscard]] double targetRad() const noexcept { return m_target; }

  private:
    Config m_config;
    bool m_holding{false};
    double m_target{0.0};
    std::uint64_t m_epoch{0};
};

/**
 * A turn rate as the mixing's wz while translating (P14). With every
 * wheel already turning, wz shifts them unevenly: each wheel's speed rises
 * (max_speed - min_speed) per unit of command, so a rad/s is
 * max_turn_rate x (1 - min_speed / max_speed) per unit of wz, not
 * max_turn_rate. Held to half the slowest wheel's translation, so no wheel
 * is pushed back toward the dead band or reversed; zero with no
 * translation (two wheels stand still on a 45 degree diagonal, so nothing
 * is held there either).
 */
[[nodiscard]] double holdFraction(double turnRadps, const Drive::BodyVelocity &translation,
                                  const Profile::GeometryProfile &g) noexcept;

/** What Assisted needs each cycle. Pointers are borrowed for the call. */
struct AssistedInput {
  const Limiter::Limiter *limiter{nullptr};
  Profile::GeometryProfile geometry{};
  /** The floor's motion table, or nullptr for the model of "Units". */
  const Motion::MotionTable *motion{nullptr};
  /** The scan, the pose history and freshness, the measured velocity and
   *  now; its command is ignored: decide() puts the stick's there. */
  Limiter::LimitInput limit{};
  /** Heading hold's turn rate, rad/s, in place of the turn stick (P14):
   *  set by the loop only with the turn stick centred and a translation
   *  asked for. The limiter is told it; the wheels get holdFraction() of
   *  it for the translation driven. */
  std::optional<double> holdRadps;
};

/** Why the wheels are what they are this cycle. */
enum class Why : std::uint8_t {
  Disarmed,            // not armed, or the throttle not yet down: stopped
  Manual,              // the sticks, mixed
  Limited,             // Assisted: the limiter's answer; see Decision::limited
  AssistedUnavailable, // Assisted but not ready: stopped
  PoseNotFresh,        // Assisted with the pose not fresh (P16): stopped
  NavigateNotBuilt     // Navigate, step 2: stopped
};

/** @return "DISARMED", "MANUAL", "LIMITED", "ASSISTED_UNAVAILABLE", "POSE_NOT_FRESH" or
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
