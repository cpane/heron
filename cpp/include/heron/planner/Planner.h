// File        : Planner.h
// Author      : Chris Pane
// Description : The local planner's core: from the obstacle map and the
//               course, the sideways and forward speed to drive now.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// In Navigate, ten times a second, the planner decides how Heron follows
// the course (CourseKeeper.h) and gets round what is in the way
// (docs/navigation-design.md §8, P9). It chooses only vx and vy: the turn
// rate is the stick's (D3, I16). Each call:
//
//   1. Can it plan? No course, no map, a map older than maxMapAge, a pose
//      not fresh, or the map, pose and course in different epochs: Stopped,
//      with the reason, and a zero twist.
//   2. A distance field (src/planner/DistanceField.h): for every map cell,
//      how far it is from the nearest cell Heron must keep clear of. Those
//      are the blocked cells, and the unknown ones within 0.6 m of Heron
//      (P6) other than those under Heron itself, which it cannot see.
//   3. Candidates: 11 x 11 speeds round the measured one, as far as
//      accelMps2 reaches in windowS, none faster than the course asks
//      (I13); plus the straight one, along the course as fast as reachable.
//   4. Each is imagined held for horizonS, checked every stepS against the
//      field, round the outline's edge. It is rejected if it comes nearer
//      than the stop margin plus the cell slack (I11), unless Heron is
//      already that near and it brings nothing nearer (I12); if it makes
//      less than 1 cm of progress along the course (I14); or if it ends
//      farther than maxOffLineM from the line, or than Heron is now (I14).
//   5. The rest are scored: clearance, progress, distance off the line at
//      the end, and change from the last choice. Clearance is scored along
//      each candidate's way out to lookAheadM, whatever its speed, so
//      slowing toward a box never scores as well as going round it. The
//      best is the plan: Following if it is the straight one and Heron is
//      on the line, otherwise Avoiding. None left: Blocked, and a zero
//      twist.
//
// With the floor's motion table (design §19, P15), step 3 changes: the
// candidates are commands, not speeds, because on the bench rig's hardwood
// a command does not make the motion its numbers suggest (a diagonal loses
// half its sideways part, a strafe needs 0.4 of full and turns the robot).
// Directions every 15 degrees, sizes 0.1 to 1.0 in 0.1, each imagined as
// the table predicts it, drift and all, plus the course's turn rate; kept
// if no faster than the course asks (I13) and within accelMps2 x windowS
// of the measured speed on each axis. A command short of the first size
// measured to move in its direction is never a candidate, rolling or
// still: the table lifts it to that size's motion (for the limiter), so it
// looks as good, and on 2026-11-16 Navigate chose left at 0.1 rolling and
// 1.0 nearly still, swinging between them round a box. Nearly still, a
// direction's smallest command that moves is kept whatever the window
// says: from rest the motors' dead band makes even the smallest command
// faster than the window reaches (forward 0.19 m/s, against 0.18), and
// without it Heron could never start. Rolling, it is not: left at 0.6
// makes 0.04 m/s, and kept past the window it was a near-stop the planner
// jumped to for a cycle or two in front of a box (2026-11-16).
// Nearly still (the OTOS's velocity, or the pose's movement over
// kMovedWindow, under kNearlyStillMps), a command must also start
// the robot from rest at kStartMps or more (MotionTable::predictFromRest(),
// the points as measured, stalls included): rolling, a small command keeps
// the robot going, but from rest it may not move it at all, and on
// 2026-11-16 Navigate slowed onto small left commands the robot could not
// start from, and stood with the stick held; then, with only "moves at
// all" required, onto right at 0.4, measured at 0.08 m/s, and stood again.
// It must also be at least kStartSize: the smallest measured points stop
// starting the robot as the battery runs down. kNearlyStillMps is well
// above a stop (0.15): at 0.05, Navigate round a box (2026-11-16) chose
// small commands while crawling, slowed to a stop, kicked with a large
// one, and went round in pulses; a crawl is now planned as a start.
// Nearly still, a direction's smallest such command is kept even when it is
// faster than the course asks (I13's one exception, as forward's dead band
// already makes 0.19 m/s of any stick): a slow course would otherwise
// leave nothing, and Heron would stand. The plan then carries the command
// chosen (I19), and Navigate drives that.
// "Straight" is any candidate whose predicted motion runs within 5 degrees
// of the course. Without a table, as above.
//
// The limiter's test (limiterBrakeMps2 set): the limiter sweeps each axis
// of a command alone, forward and sideways, for as long as it takes to
// stop from the measured speed plus its blind time, and cuts an axis whose
// sweep comes inside the stop margin (Limiter.h). A diagonal past a box
// can clear it while its forward part alone does not; at 0.42 m/s that
// sweep is about 0.37 m on the hardwood, and the box scenarios driven at
// full stick saw the limiter cut the planner's choice in 6 runs of 10
// (2026-11-16). So a candidate is also rejected if any axis alone, or the
// whole, told to the limiter limiterScale faster, would come inside the
// margin (I12's floor) within that time. The turn is the one the limiter
// is told: holding a heading (Course::holdHeadingRad), heading hold's rate
// plus the drift, which is the error times holdGainPerS; else the
// candidate's own. That turn is what made the remaining cuts: the left
// diagonal turns Heron 18 degrees, and hold swinging it back beside the
// box swept the body's corner into it.
//
// The cell slack, kCellSlackM: the map is 5 cm cells, so true clearance
// can fall short of what the field says by half a cell's diagonal for
// where the hit lies in its cell, as much again for reading the field at a
// cell's centre, and half the 2.5 cm spacing round the edge. The planner
// keeps that much more than the limiter, so its choices normally pass the
// limiter untouched; the limiter, on raw points, catches what the map gets
// wrong.
//
// Pure (P2): no clock, no thread. It remembers one thing between calls,
// the last choice, for the change score. Design:
// docs/navigation-cpp-design.md §8.
//
// Usage
// -----
//   Heron::Planner::Planner planner(config);          // from the profile (I6)
//   const Plan plan = planner.plan(PlanInput{map.get(), poseNow, epoch, fresh,
//                                            measured, course, now});
//   // plan.twist: vx, vy chosen, wz the course's; plan.state and reason say why
//   // with config.motion set: plan.commandVx, commandVy, the fractions to drive

#ifndef HERON_PLANNER_PLANNER_H
#define HERON_PLANNER_PLANNER_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "heron/geometry/Outline.h"
#include "heron/geometry/Pose2.h"
#include "heron/motion/MotionTable.h"
#include "heron/perception/ObstacleMap.h"
#include "heron/planner/CourseKeeper.h"

namespace Heron::Planner {

/** How short of the field's clearance the true one can be (I11): two half
 *  diagonals of a 5 cm cell and half the 2.5 cm spacing round the edge. */
inline constexpr double kCellSlackM = 0.0833;

/** Measured slower than this, m/s, Heron is nearly still: with a table,
 *  only commands that start it from rest are candidates (header). */
inline constexpr double kNearlyStillMps = 0.15;

/** The pose's movement over this long is the second test of nearly still:
 *  the OTOS's velocity can read a speed with the robot standing (2026-11-16:
 *  -0.07 m/s for seconds, the LiDAR's nearest point unchanged), and then
 *  only small commands, which cannot start it, were allowed. */
inline constexpr Duration kMovedWindow = std::chrono::milliseconds(500);

/** Nearly still, a candidate must start Heron from rest at this, m/s, or
 *  more (MotionTable::predictFromRest()): a command measured just above
 *  the dead band does not start it every time (2026-11-16: right at 0.4,
 *  measured 0.08 m/s, left it standing). The same speed as MotionWatch's
 *  expectMps. */
inline constexpr double kStartMps = 0.10;

/** Nearly still, a candidate must also be at least this size, fractions of
 *  full drive: the table's smallest points were measured with a fresh
 *  battery, and on 2026-11-16, after a day of runs, forward at 0.2
 *  (measured 0.19 m/s on 2026-11-14) only hummed. 0.4 is twice that. */
inline constexpr double kStartSize = 0.4;

/** Design §8.2's starting weights, tuned in the simulator (design §18, 3);
 *  offLine and change raised for full stick. At 1.0 and 0.5, the box
 *  scenarios at 0.42 m/s ended up to 0.91 m off the line 2 m past the box
 *  (6 runs of 30 over 0.5 m). At 2 and 2.5 (2026-11-16), within 0.47 m,
 *  but on the floor Navigate braked to 0.1 m/s before choosing a side.
 *  At 1.5 and 1 (2026-11-18): within 0.48 m, a run 15.8 s on average
 *  against 17.0, and at 0.3 m/s within 0.14 m [the floor unverified]. */
struct ScoreWeights {
  double clearance{1.0};
  double way{1.0};  // the free way ahead, out to lookAheadM
  double progress{2.0};
  double offLine{1.5};
  double change{1.0};
};

/** The planner's settings, filled by heron_robot from the profile (I6). */
struct PlannerConfig {
  Geometry::Outline outline{};  // [geometry] length and width, halved
  double stopMarginM{0.05};     // [navigation] stop_margin
  double maxSpeedMps{0.35};     // [geometry] max_speed
  double accelMps2{0.6};        // [geometry] accel
  double windowS{0.3};          // speeds reachable in this long
  int samples{11};              // per axis: 11 x 11 candidates
  double horizonS{1.5};         // each candidate held this long
  double stepS{0.05};           // checked this often
  double pathStepS{0.1};        // the published path's spacing
  double unknownRadiusM{0.6};   // unknown is blocked this near (P6)
  double clearanceCapM{0.5};    // more clearance scores no higher
  double preferredClearanceM{0.15};  // the free way ends nearer than this
  double lookAheadM{1.0};       // the free way is scored this far along, and
                                // progress toward the line's point this far ahead
  double maxOffLineM{1.0};      // I14
  double minProgressM{0.01};    // I14: less over the horizon is no progress
  double onLineM{0.05};         // within this of the line: Following
  Duration maxMapAge{std::chrono::milliseconds(500)};
  ScoreWeights weights{};
  /** The floor's motion table (P15): with it the candidates are commands,
   *  each imagined as the table predicts; nullptr: speeds, as before. */
  std::shared_ptr<const Motion::MotionTable> motion;
  double commandStepRad{15.0 * Geometry::kDegToRad};  // with a table: directions this far apart
  double commandSizeStep{0.1};                         // and sizes this far apart, up to 1
  /** The limiter's per-axis test, applied to each candidate (header):
   *  the brake, m/s^2, that sets the limiter's horizon; 0: off. */
  double limiterBrakeMps2{0.0};
  double limiterReactionS{0.25};  // its blind time: a scan's age and a revolution
  double limiterScale{1.0};       // what it is told, against the table's prediction
  double holdGainPerS{2.0};       // heading hold's (RobotCore.h), for the turn
  double holdMaxRadps{0.8};       // the limiter is told while it holds
};

enum class State : std::uint8_t {
  Following,  // on the course
  Avoiding,   // off it, to get round something
  Blocked,    // no candidate passes: stopped
  Stopped     // cannot plan: see Reason
};

enum class Reason : std::uint8_t {
  None,          // Following or Avoiding
  NothingSafe,   // Blocked: every candidate came too near or made no progress
  NoCourse,      // the stick is centred
  NoMap,         // no map yet
  StaleMap,      // the map is older than maxMapAge
  PoseNotFresh,
  OtherFrame     // the map, pose and course are not all in one epoch
};

/** One cycle's answer; never changed once published. */
struct Plan {
  Geometry::Twist twist{};            // vx, vy chosen; wz the course's (with a
                                      // table: as predicted, the drift added)
  double commandVx{0.0};              // with a table: the command chosen,
  double commandVy{0.0};              // fractions of full drive (I19); else 0
  std::vector<Geometry::Pose2> path;  // odom frame, every pathStepS, for telemetry
  State state{State::Stopped};
  Reason reason{Reason::NoCourse};
  std::uint64_t courseSeq{0};         // the course it was made for
  double clearanceM{0.0};             // the chosen path's, after the cell slack
  int candidatesTried{0};
  int candidatesPassed{0};
  bool nearlyStill{false};            // planned as from rest (header)
  double movedMps{-1.0};              // the input's, for the status line
  TimePoint time{};                   // when it was made
};

/** What one cycle reads. The map is borrowed for the call only. */
struct PlanInput {
  const Perception::ObstacleMap *map{nullptr};
  Geometry::Pose2 pose{};       // Heron now (PoseHistory::at(now)), odom frame
  std::uint64_t epoch{0};       // the pose's
  bool poseFresh{false};        // PoseSnapshot::fresh, and the lookup found it
  Geometry::Twist measured{};   // newest odometry velocity, robot frame
  double movedMps{-1.0};        // how fast the pose moved over the last
                                // kMovedWindow; below 0 when not known
  std::optional<Course> course;
  TimePoint now{};
};

/**
 * Thread Safety: a plain value. plan() changes the remembered last choice,
 * so one Planner belongs to one thread.
 */
class Planner {
  public:
    explicit Planner(const PlannerConfig &config);

    /**
     * Design §8.2, with I10 to I14.
     *
     * @param input The newest map, pose and course, and now
     * @return What to drive, and why
     */
    [[nodiscard]] Plan plan(const PlanInput &input);

    /** Forgets the last choice. */
    void reset() noexcept;

    [[nodiscard]] const PlannerConfig &config() const noexcept { return m_config; }

  private:
    PlannerConfig m_config;
    std::vector<Geometry::Vec2> m_edge;     // round the outline, robot frame
    std::optional<Geometry::Twist> m_last;  // for the change score
};

/** @return "FOLLOWING", "AVOIDING", "BLOCKED" or "STOPPED" */
[[nodiscard]] const char *toString(State state) noexcept;

/** @return "NONE", "NOTHING_SAFE", "NO_COURSE", "NO_MAP", "STALE_MAP",
 *          "POSE_NOT_FRESH" or "OTHER_FRAME" */
[[nodiscard]] const char *toString(Reason reason) noexcept;

}  // namespace Heron::Planner

#endif  // HERON_PLANNER_PLANNER_H
