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

#ifndef HERON_PLANNER_PLANNER_H
#define HERON_PLANNER_PLANNER_H

#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "heron/geometry/Outline.h"
#include "heron/geometry/Pose2.h"
#include "heron/perception/ObstacleMap.h"
#include "heron/planner/CourseKeeper.h"

namespace Heron::Planner {

/** How short of the field's clearance the true one can be (I11): two half
 *  diagonals of a 5 cm cell and half the 2.5 cm spacing round the edge. */
inline constexpr double kCellSlackM = 0.0833;

/** Design §8.2's starting weights; tuned in the simulator (design §18, 3). */
struct ScoreWeights {
  double clearance{1.0};
  double way{1.0};  // the free way ahead, out to lookAheadM
  double progress{2.0};
  double offLine{1.0};
  double change{0.5};
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
  Geometry::Twist twist{};            // vx, vy chosen; wz the course's
  std::vector<Geometry::Pose2> path;  // odom frame, every pathStepS, for telemetry
  State state{State::Stopped};
  Reason reason{Reason::NoCourse};
  std::uint64_t courseSeq{0};         // the course it was made for
  double clearanceM{0.0};             // the chosen path's, after the cell slack
  int candidatesTried{0};
  int candidatesPassed{0};
  TimePoint time{};                   // when it was made
};

/** What one cycle reads. The map is borrowed for the call only. */
struct PlanInput {
  const Perception::ObstacleMap *map{nullptr};
  Geometry::Pose2 pose{};       // Heron now (PoseHistory::at(now)), odom frame
  std::uint64_t epoch{0};       // the pose's
  bool poseFresh{false};        // PoseSnapshot::fresh, and the lookup found it
  Geometry::Twist measured{};   // newest odometry velocity, robot frame
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
