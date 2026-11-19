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
//   Navigate  the right stick sets a course (CourseKeeper); the planner
//             picks the command that follows it round obstacles, and that
//             command goes through Assisted's path as if it were the
//             stick: the limiter, the table, heading hold, CutHold
//             (navigateCommand()). Letting go stops at once.
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

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "heron/drive/Drive.h"
#include "heron/geometry/Pose2.h"
#include "heron/lidar/Lidar.h"
#include "heron/limiter/Limiter.h"
#include "heron/motion/MotionTable.h"
#include "heron/odometry/OtosConfig.h"
#include "heron/perception/Perception.h"
#include "heron/planner/CourseKeeper.h"
#include "heron/planner/Planner.h"
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

/** The map's settings: where the LiDAR sits ([lidar] x, y; its yaw is the
 *  scan's own, toLidarConfig()); the counts and timings as designed. */
[[nodiscard]] Perception::PerceptionConfig toPerceptionConfig(const Profile::LidarProfile &l);

/**
 * The planner's settings: the outline, stop_margin, max_speed, accel, and
 * the floor's table, so its candidates are commands (P15).
 *
 * @throws std::invalid_argument if [geometry] or [navigation] is missing;
 *         check assistedRefusal() first
 */
[[nodiscard]] Planner::PlannerConfig toPlannerConfig(const Profile::RigProfile &profile,
                                                     std::shared_ptr<const Motion::MotionTable> motion);

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

/**
 * "Commanded but not moving" (design §19.4 step 4, accepted 2026-11-15):
 * the wheels driven with a command that, from rest, should move the robot
 * at expectMps or more, while the OTOS reads under stillMps, for a whole
 * window, trips it. It catches the robot lifted (floor check 4: a gentle
 * lift raised TILT only and read exactly 0, so the pose stayed fresh and
 * the wheels turned in the air) and wheels spinning while stuck.
 *
 * "From rest" is MotionTable::predictFromRest(), the points as measured:
 * a command in the dead band, which cannot start a still robot, expects
 * almost nothing and never trips it. The numbers are from the floor
 * checks of 2026-11-14: from rest the bench rig reached 0.15 m/s within
 * 0.2 s; standing on the hardwood the OTOS read within 0.01 m/s; lifted,
 * exactly 0.
 *
 * Once tripped it stays tripped until the translation stick is centred,
 * so a lifted robot does not pulse its wheels every window. Assisted
 * stops while it is (Why::NotMoving). It is not the pose's freshness: the
 * pose tracker cannot see what the wheels were told (I21).
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class MotionWatch {
  public:
    struct Config {
      double expectMps{0.10};                    // the command should make at least this
      double stillMps{0.01};                     // the OTOS reads under this
      std::chrono::milliseconds window{500};     // both, this long
    };

    MotionWatch() noexcept;  // Config's defaults
    explicit MotionWatch(Config config) noexcept;

    /**
     * One cycle.
     *
     * @param expectedMps what last cycle's driven command makes from rest
     *        (predictFromRest()), speed over the floor; 0 without a table
     * @param measuredMps the OTOS's speed over the floor
     * @param translating whether the stick asks for any translation; once
     *        tripped, false clears it
     * @param now this cycle's time
     * @return whether it is tripped
     */
    bool update(double expectedMps, double measuredMps, bool translating,
                std::chrono::steady_clock::time_point now) noexcept;

    /** Clears it: on leaving Assisted or Navigate, and on disarming. */
    void reset() noexcept;

    [[nodiscard]] bool tripped() const noexcept { return m_tripped; }

  private:
    Config m_config;
    bool m_tripped{false};
    std::optional<std::chrono::steady_clock::time_point> m_since;
};

/**
 * "Driven and standing" (2026-11-18): the wheels told the same translation
 * for kFor while the pose moved under stillMps gives that command as dead
 * (Motion::DeadCommand). The table can say a command starts the robot when
 * on today's batteries it does not: on 2026-11-16 the limiter held a right
 * strafe at 0.5, which the table put at 0.11 m/s, for 9 s beside a box,
 * the wheels humming. MotionWatch did not trip: the OTOS read 0.03 to
 * 0.04 m/s standing (its phantom speed), over its 0.01, and stopping is
 * not the cure anyway. Once dead, the planner and decide() pass over that
 * size and below in that direction while the robot is nearly still, so a
 * larger size or another way is chosen.
 *
 * The pose's movement, not the OTOS's velocity: the pose read 0.00 m/s
 * through the 9 s. "The same translation" is within kSubstituteWidthRad
 * and 0.05 of size of where the window began; a change starts it again.
 * The largest size seen in the window is given. Full drive is never dead:
 * that is a lift or a wall, MotionWatch's and the limiter's business.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class StallWatch {
  public:
    struct Config {
      double stillMps{0.03};                  // the pose moved less than this
      std::chrono::milliseconds window{1000};  // with the same translation, this long
    };

    StallWatch() noexcept;  // Config's defaults
    explicit StallWatch(Config config) noexcept;

    /**
     * One cycle.
     *
     * @param driven the translation the wheels were told last cycle,
     *        fractions; zero: nothing driven
     * @param movedMps how fast the pose moved over the last
     *        Planner::kMovedWindow; below 0 when not known
     * @param now this cycle's time
     * @return a command just found dead, once per window
     */
    std::optional<Motion::DeadCommand> update(const Drive::BodyVelocity &driven, double movedMps,
                                              std::chrono::steady_clock::time_point now) noexcept;

    /** Starts the window afresh. */
    void reset() noexcept;

  private:
    Config m_config;
    std::optional<std::chrono::steady_clock::time_point> m_since;
    double m_directionRad{0.0};
    double m_firstSize{0.0};
    double m_largestSize{0.0};
};

/** Adds a dead command, merging it with one within kSubstituteWidthRad:
 *  the larger size, at the newer direction. */
void addDead(std::vector<Motion::DeadCommand> &dead, const Motion::DeadCommand &found);

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
  /** MotionWatch tripped: the wheels were told to move and nothing moved. */
  bool notMoving{false};
  /** CutHold::ceiling(): while holding, last cycle's translation.
   *  decide() drives nothing the table says makes more, Clear included
   *  (the table's path only; without a table there are no substitutes). */
  std::optional<Drive::BodyVelocity> cutCeiling;
  /** Commands seen not to start the robot (StallWatch): with the robot
   *  nearly still, no substitute is one of them. Borrowed. */
  std::span<const Motion::DeadCommand> dead;
};

/** Why the wheels are what they are this cycle. */
enum class Why : std::uint8_t {
  Disarmed,            // not armed, or the throttle not yet down: stopped
  Manual,              // the sticks, mixed
  Limited,             // Assisted: the limiter's answer; see Decision::limited
  AssistedUnavailable, // Assisted but not ready: stopped
  PoseNotFresh,        // Assisted with the pose not fresh (P16): stopped
  NotMoving            // Assisted, told to move and nothing moved (MotionWatch): stopped
};

/** @return "DISARMED", "MANUAL", "LIMITED", "ASSISTED_UNAVAILABLE", "POSE_NOT_FRESH"
 *          or "NOT_MOVING" */
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
 * stick through the limiter; Navigate runs the plan's command through the
 * same path (the loop passes navigateCommand() as the stick), so every
 * safeguard Assisted has, Navigate has.
 *
 * @param mode SwC's mode
 * @param armed ArmGate::update()'s answer this cycle
 * @param stick Drive::sticksToVelocity()'s answer this cycle; in Navigate,
 *        navigateCommand()'s
 * @param assisted What Assisted needs, or nullptr if it is not ready
 * @return The wheels and why
 */
[[nodiscard]] Decision decide(Mode mode, bool armed, const Drive::BodyVelocity &stick,
                              const AssistedInput *assisted);

/**
 * While the limiter cuts, the command only falls (design §19.4 step 4,
 * floor check 2 again, 2026-11-16). Driven at a box at full speed, the
 * limiter's answer changed cycle to cycle with the OTOS's speed noise
 * (about +-0.03 m/s forward and +-0.07 sideways at speed), so the command
 * jumped between nothing and something (0.50, 0, 0.29, 0, 0.39, 0.30, 0),
 * and once a substitute 15 degrees off asked for more than the cycle
 * before (0.75, then 0.97 -0.26). The stop was where it should be; the
 * wheels were pulsed.
 *
 * CutHold holds from the first Cut, keeping each cycle's translation as
 * the next one's ceiling (AssistedInput::cutCeiling), until the limiter
 * has been Clear for kReleaseCycles in a row, the translation stick is
 * centred, or it points more than kSubstituteWidthRad from where it did
 * when the hold began: pulling back to get away is never held to the way
 * in. One Clear is not enough: the same noise gives a single Clear between
 * Cuts (the replay in RobotTest), and a lower command than the limiter
 * allows is always safe. Use: ceiling() before decide(), update() after.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class CutHold {
  public:
    /** Clear cycles in a row that end a hold: 0.1 s at the loop's 20 ms. */
    static constexpr int kReleaseCycles = 5;

    /** @return the ceiling for this stick while holding, else nullopt */
    [[nodiscard]] std::optional<Drive::BodyVelocity> ceiling(const Drive::BodyVelocity &stick) const noexcept;
    /** What decide() gave for this stick, this cycle. */
    void update(const Drive::BodyVelocity &stick, const Decision &decision) noexcept;
    void reset() noexcept;
    [[nodiscard]] bool holding() const noexcept;

  private:
    [[nodiscard]] bool along(const Drive::BodyVelocity &stick) const noexcept;

    bool m_holding{false};
    double m_directionRad{0.0};  // the translation stick's, when the hold began
    int m_clears{0};             // Clear cycles in a row while holding
    Drive::BodyVelocity m_last{};
};

/**
 * Navigate's command (navigation-cpp-design.md §8.7, with the table of
 * §19): the plan's, in fractions of full drive, if it was made for this
 * course and is no older than kMaxPlanAge; otherwise none, so Heron stops.
 * With a table the plan's command is used as chosen (I19); without one its
 * speeds are turned back into fractions. The turn is the stick's now, not
 * the one the plan was made with; centred, heading hold keeps the heading.
 *
 * @param course CourseKeeper's answer this cycle; no value: the stick is
 *        centred, stop at once (D1), not 100 ms later in the planner
 * @param plan LocalPlanner's newest, or nullptr
 * @param stick this cycle's sticks; only the turn (wz) is used
 * @param withTable whether the planner had the floor's table
 */
[[nodiscard]] Drive::BodyVelocity navigateCommand(const std::optional<Planner::Course> &course,
                                                  const Planner::Plan *plan, const Drive::BodyVelocity &stick,
                                                  const Profile::GeometryProfile &g, bool withTable,
                                                  std::chrono::steady_clock::time_point now) noexcept;

/** A plan older than this is not driven (design §8.2). */
inline constexpr std::chrono::milliseconds kMaxPlanAge{300};

/**
 * BLOCKED on the status line (decided 2026-11-16, step 5's question 4):
 * the plan asks to move and the limiter has let none of it through for
 * longer than the window. The planner and the limiter are unchanged: this
 * only says so, so whoever drives sees why Heron waits.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class BlockedWatch {
  public:
    static constexpr std::chrono::milliseconds kWindow{500};

    /**
     * @param asked the plan's command translates
     * @param driven what decide() gave translates
     * @return whether to show BLOCKED
     */
    bool update(bool asked, bool driven, std::chrono::steady_clock::time_point now) noexcept;
    void reset() noexcept;
    [[nodiscard]] bool blocked() const noexcept { return m_blocked; }

  private:
    bool m_blocked{false};
    std::optional<std::chrono::steady_clock::time_point> m_since;
};

/**
 * How much the LiDAR's view changed between two scans, metres: the median,
 * over the kSectorDeg sectors with a return in both, of the change in the
 * nearest range in each. Standing still the A1M8 repeats a range to a
 * millimetre or two (RESUME-lidar.md's Inbox, 2026-11-14); carried, or
 * driven at a crawl over a second, the view moves by centimetres.
 *
 * @return the change; nullopt with fewer than kMinSectors in common
 */
[[nodiscard]] std::optional<double> scanChange(const Scan &from, const Scan &to);

/**
 * Whether the robot is still, by the LiDAR alone (2026-11-16): the OTOS
 * cannot say so while it is the thing in doubt. Still when every scan for
 * kFor has stayed within kChangeM of the first of them (scanChange()); a
 * scan that moved starts the count again from it. Comparing against that
 * first scan, not the last one, catches a slow drift.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class LidarStill {
  public:
    static constexpr double kChangeM{0.02};
    static constexpr std::chrono::milliseconds kFor{1000};

    /**
     * @param scan the newest scan; nullptr (none yet) is not still
     * @return whether the view has held for kFor, up to this scan
     */
    bool update(const std::shared_ptr<const Scan> &scan);
    [[nodiscard]] bool still() const noexcept { return m_still; }

  private:
    std::shared_ptr<const Scan> m_from;  // the first scan of the still run
    std::uint64_t m_lastSeq{0};
    bool m_still{false};
};

/**
 * When to restart the OTOS (2026-11-16): picked up while heron_robot ran,
 * the bench rig's OTOS raised TRACKING and read about -3 m/s with the robot
 * back on the floor and still, for over a minute, until the program ended;
 * Assisted and Navigate stop while the pose is not fresh, so the robot was
 * Manual only. Earlier lifts cleared in about 0.3 s. A restart of the
 * odometry recovers it (and starts a new epoch, I8: the map, the course and
 * heading hold start afresh).
 *
 * The OTOS calibrates its gyro as it starts, so it must restart on a still
 * robot. The first version took centred sticks for still, and the same day
 * restarted while the robot was carried: it came back with TILT and a
 * turn of -0.19 rad/s standing, the heading 11 deg/s adrift, and Navigate
 * veered left after it (navigate-fresh-2026-11-16.log). Now:
 *
 *   - Trouble: the pose not fresh for kNotFresh, or, the LiDAR saying still,
 *     the OTOS reading TILT or a turn over kSuspectRadps for kSuspect (a
 *     restart that calibrated on a moving robot).
 *   - Then restart once the LiDAR says still (LidarStill) and the sticks
 *     have been centred for kCentred, and not within kBetween of the last
 *     restart; a bad restart is trouble again, so it is retried.
 *
 * Thread Safety: Not thread-safe; one control loop owns it.
 */
class OtosRecovery {
  public:
    static constexpr std::chrono::milliseconds kNotFresh{2000};
    static constexpr std::chrono::milliseconds kSuspect{1000};
    static constexpr double kSuspectRadps{0.05};
    static constexpr std::chrono::milliseconds kCentred{1000};
    static constexpr std::chrono::milliseconds kBetween{5000};

    /** One cycle's readings. */
    struct Input {
      bool poseFresh{false};
      bool tilt{false};          // the OTOS's TILT warning
      double turnRadps{0.0};     // the OTOS's turn rate
      bool sticksCentred{false};
      bool still{false};         // LidarStill
    };

    /** @return whether to restart the OTOS now; true starts kBetween */
    bool update(const Input &in, std::chrono::steady_clock::time_point now) noexcept;

    /** @return why the last restart, for the status line */
    [[nodiscard]] const char *why() const noexcept { return m_why; }

  private:
    std::optional<std::chrono::steady_clock::time_point> m_notFreshSince;
    std::optional<std::chrono::steady_clock::time_point> m_suspectSince;
    std::optional<std::chrono::steady_clock::time_point> m_centredSince;
    std::optional<std::chrono::steady_clock::time_point> m_lastRestart;
    const char *m_why{""};
};

}  // namespace Heron::Robot

#endif  // HERON_ROBOT_ROBOT_CORE_H
