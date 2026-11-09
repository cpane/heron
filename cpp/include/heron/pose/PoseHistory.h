// File        : PoseHistory.h
// Author      : Chris Pane
// Description : The last two seconds of Heron's pose, and where it was at
//               any moment in them: the core of heron_pose.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The OTOS reports Heron's pose about 100 times a second. Perception and
// the limiter need it at other times: each LiDAR point was measured at its
// own moment, somewhere in a 133 ms revolution, and placing every point at
// the pose of that moment is what keeps a wall straight while Heron moves
// (deskewing, docs/navigation-design.md §3.3). So the readings are kept,
// each at its own time, and at() answers "where was Heron at t?":
//
//   - between two readings, interpolated: x and y in a straight line, the
//     heading the short way round, so 179 to -179 degrees passes through
//     180, not through 0;
//   - up to 50 ms after the newest, extrapolated with its velocity, so a
//     lookup at "now" works between two readings;
//   - otherwise it fails, with the reason. A failure is never a zero pose:
//     (0, 0, 0) is a real pose, where Heron started (I5).
//
// A fixed ring of 256 readings, 2.56 s at 100 a second; a revolution is
// 133 ms, so a whole scan always fits. Pure: no clock, no thread, no lock.
// PoseTracker holds one under its lock and hands out copies, about 14 KB
// each (I7). Design: docs/navigation-cpp-design.md §4.1.
//
// Usage
// -----
//   using namespace std::chrono_literals;
//   Heron::Pose::PoseHistory history;
//   history.add(PoseSample{t0, Pose2{0.0, 0.0, 0.0}, Twist{0.2, 0.0, 0.0}});
//   history.add(PoseSample{t0 + 10ms, Pose2{0.002, 0.0, 0.0}, Twist{0.2, 0.0, 0.0}});
//   const PoseLookup p = history.at(t0 + 5ms);     // ok, x = 0.001
//   const PoseLookup q = history.at(t0 + 40ms);    // ok, extrapolated: x = 0.008
//   const PoseLookup r = history.at(t0 + 70ms);    // !ok, PastExtrapolation

#ifndef HERON_POSE_POSE_HISTORY_H
#define HERON_POSE_POSE_HISTORY_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "heron/geometry/Pose2.h"
#include "heron/lidar/Scan.h"

namespace Heron::Pose {

/** One odometry reading, at its own time. */
struct PoseSample {
  TimePoint time{};
  Geometry::Pose2 pose{};      // robot frame in the odom frame
  Geometry::Twist velocity{};  // robot frame
};

/** Why a lookup failed. */
enum class LookupFailure : std::uint8_t {
  None,              // it did not
  Empty,             // no reading yet
  BeforeOldest,      // earlier than the oldest reading kept
  PastExtrapolation  // more than kMaxExtrapolation after the newest
};

/** A pose at a moment, or why there is none. */
struct PoseLookup {
  bool ok{false};
  LookupFailure why{LookupFailure::Empty};
  Geometry::Pose2 pose{};      // meaningful only when ok
};

/**
 * The last readings, a fixed ring of kCapacity, oldest overwritten first.
 *
 * Thread Safety: a plain value, not thread-safe. PoseTracker holds one
 * under its lock and hands out copies.
 */
class PoseHistory {
  public:
    static constexpr std::size_t kCapacity = 256;
    static constexpr Duration kMaxExtrapolation = std::chrono::milliseconds(50);

    /**
     * Adds a reading. One not later than the newest is ignored, so the ring
     * stays in time order.
     *
     * @param sample The reading, at its own time
     */
    void add(const PoseSample &sample) noexcept;

    /** Forgets every reading. */
    void clear() noexcept;

    /**
     * Where Heron was at t.
     *
     * @param t A moment, steady_clock
     * @return The pose, interpolated between the readings around t, heading
     *         the short way round; or, up to kMaxExtrapolation past the
     *         newest, extrapolated with its velocity; or a failure
     */
    [[nodiscard]] PoseLookup at(TimePoint t) const noexcept;

    /** @return The newest reading, or none */
    [[nodiscard]] std::optional<PoseSample> newest() const noexcept;

    /** @return The oldest reading kept, or none */
    [[nodiscard]] std::optional<PoseSample> oldest() const noexcept;

    /** @return How many readings it holds, at most kCapacity */
    [[nodiscard]] std::size_t size() const noexcept { return m_count; }

  private:
    /** The i-th reading kept, 0 the oldest. */
    [[nodiscard]] const PoseSample &nth(std::size_t i) const noexcept;

    std::array<PoseSample, kCapacity> m_ring{};
    std::size_t m_next{0};   // where the next reading goes
    std::size_t m_count{0};
};

}  // namespace Heron::Pose

#endif  // HERON_POSE_POSE_HISTORY_H
