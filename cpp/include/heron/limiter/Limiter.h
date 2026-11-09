// File        : Limiter.h
// Author      : Chris Pane
// Description : The safety limiter: the last check between a command and the
//               motors, against the newest raw LiDAR scan.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// In Assisted and Navigate, every 20 ms, the control loop hands the limiter
// the command (the stick's, or the planner's) and gets back what may be
// driven (docs/navigation-design.md §7). It reads the newest scan's raw
// points, never the obstacle map, so a bug in perception or the planner
// cannot drive Heron into what the LiDAR sees (P7, P8). Manual is never
// limited: it is the way out when the LiDAR fails.
//
// Each call:
//
//   1. Can it see? No scan, a scan older than maxScanAge, or a revolution
//      not fully observed: stop (design §7.4).
//   2. Where are the points now? Each is placed in the robot frame at its
//      own time, then moved by Heron's motion since (pose history), so it
//      is where it is now. With the pose not fresh they are not moved, and
//      the reaction time doubles instead.
//   3. How far ahead to look: T, the time to stop from the measured speed
//      plus the time the limiter is blind, the scan's age and one more
//      revolution: T = v / brake + reaction.
//   4. The rule (design §7.3), against the outline grown by stopMargin:
//        - each of vx, vy and wz alone: kept whole if its sweep stays clear
//          for T; otherwise cut to the largest value that does; and slowed
//          in proportion when something lies within slowMargin along it;
//        - then the cut motion together, cut as a whole if it still
//          touches;
//        - something already inside the stop margin (rule 4): every point
//          in the margin stops only the axes that would bring it nearer
//          than the nearest anything is now, so backing off and sliding
//          along a wall stay allowed; the points outside the margin are
//          swept against the body grown by that nearest distance, not by
//          the margin, so the rest of a wall may come in beside as Heron
//          slides. How, why, what it costs, and the alternative:
//          docs/navigation-cpp-design.md §6.2.
//      Per axis is what lets Heron keep sliding along a wall while the
//      part of the motion into it stops.
//
// Distances to the outline are measured as Outline's grown() grows it:
// the larger of the gaps along x and along y, so a point is inside the
// stop outline exactly when its distance is stopMargin or less.
//
// Why "cut to the largest value" is exact, not searched for: scaling a
// twist leaves the path the outline sweeps unchanged and only changes how
// fast it is travelled. If the full command touches a point at time t < T,
// the same command scaled by t / T reaches that point at exactly T, and
// nothing sooner. So every scale is one sweep of the points.
//
// The slow zone is measured along the path too: the distance travelled
// before contact, against slowMargin. Distance for a turn is the angle
// times the outline's half-diagonal, the farthest a corner moves.
//
// Pure: no clock, no thread, and nothing kept between calls, so the same
// input always gives the same answer. Design:
// docs/navigation-cpp-design.md §6.
//
// Usage
// -----
//   Heron::Limiter::Limiter limiter(config);         // from the profile (I6)
//   const auto [history, epoch] = tracker.copyHistory();
//   const PoseSnapshot pose = tracker.getLatest();
//   const auto scan = lidar.getLatest();
//   const LimitResult r = limiter.limit(LimitInput{
//       stickTwist, scan.get(), &history, pose.fresh, pose.newest.velocity, now});
//   drive(r.command);                                // and report r.reason

#ifndef HERON_LIMITER_LIMITER_H
#define HERON_LIMITER_LIMITER_H

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "heron/geometry/Outline.h"
#include "heron/geometry/Pose2.h"
#include "heron/lidar/Scan.h"
#include "heron/pose/PoseHistory.h"

namespace Heron::Limiter {

/** The A1M8 returns nothing nearer than this, measured [ours]: 126.8 mm. */
inline constexpr double kBlindRadiusM = 0.1268;

/** The limiter's settings, filled by heron_robot from the profile (I6). */
struct LimiterConfig {
  Geometry::Outline outline{};          // [geometry] length and width, halved
  Geometry::Vec2 lidarOffset{};         // [lidar] x, y: the spin axis, robot frame
  double stopMarginM{0.05};             // nothing comes nearer the outline than this
  double slowMarginM{0.25};             // within this along the motion: slower
  double brakeMps2{1.0};                // [geometry] brake
  Duration maxScanAge{std::chrono::milliseconds(300)};
  Duration scanPeriod{std::chrono::milliseconds(137)};  // one revolution
  Geometry::SweepStep step{};
};

/** Why the command came back as it did, most serious first in each group. */
enum class Reason : std::uint8_t {
  Clear,       // passed unchanged
  Cut,         // an axis, or the whole, cut to stay clear for T
  Slowed,      // only the slow zone lowered it
  BackingOff,  // something is inside the stop outline; only motion away passes
  NoScan,      // stopped: no scan yet
  Stale,       // stopped: the newest scan is older than maxScanAge
  NotCovered,  // stopped: the revolution was not fully observed
  Unobserved   // an axis stopped: its direction was not looked at
};

/** One call's inputs. The pointers are borrowed for the call only. */
struct LimitInput {
  Geometry::Twist command{};               // m/s, rad/s, robot frame
  const Heron::Scan *scan{nullptr};        // the newest, or nullptr
  const Pose::PoseHistory *history{nullptr};
  bool poseFresh{false};                   // PoseSnapshot::fresh
  Geometry::Twist measured{};              // newest odometry velocity
  TimePoint now{};
};

/** One call's answer. */
struct LimitResult {
  Geometry::Twist command{};               // what may be driven
  Geometry::Twist scale{1.0, 1.0, 1.0};    // per axis, 0..1, of the command
  Reason reason{Reason::Clear};
  std::optional<Geometry::Vec2> nearest;   // the point nearest the outline, robot frame, now
  double horizonS{0.0};                    // T, as used
};

/**
 * Thread Safety: limit() is const and keeps nothing; one Limiter may be
 * called from any thread.
 */
class Limiter {
  public:
    explicit Limiter(const LimiterConfig &config);

    /**
     * Design §7.3 and §7.4.
     *
     * @param input The command, the newest scan and pose, and now
     * @return What may be driven, and why
     */
    [[nodiscard]] LimitResult limit(const LimitInput &input) const;

    [[nodiscard]] const LimiterConfig &config() const noexcept { return m_config; }

  private:
    LimiterConfig m_config;
};

/**
 * The profile-time check of design §7.4: the LiDAR's blind circle,
 * kBlindRadiusM round its spin axis, must lie inside the stop outline.
 * Anything the LiDAR cannot see is then already inside Heron's margin.
 *
 * @return An empty string if it does, otherwise why not
 */
[[nodiscard]] std::string checkBlindSpot(const LimiterConfig &config);

/** @return "CLEAR", "CUT", "SLOWED", "BACKING_OFF", "NO_SCAN", "STALE",
 *          "NOT_COVERED" or "UNOBSERVED" */
[[nodiscard]] const char *toString(Reason reason) noexcept;

}  // namespace Heron::Limiter

#endif  // HERON_LIMITER_LIMITER_H
