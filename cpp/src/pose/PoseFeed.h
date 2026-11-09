// File        : PoseFeed.h
// Author      : Chris Pane
// Description : What PoseTracker does with each odometry reading, without
//               its thread or clock: the epoch, the gap and freshness.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// PoseTracker's decisions, as a core (docs/navigation-design.md §4.1, P2):
// given one Odometry snapshot and the time it was asked for, update the
// history, the epoch and the snapshot. PoseTracker calls update() from its
// thread under its lock; pose_test calls it with made-up snapshots and
// times, so a restart or a gap is one call, not a wait.
//
// Private to heron_pose: robot code uses PoseTracker.
//
// Usage
// -----
//   PoseFeed feed(PoseConfig{});
//   feed.update(odometry.getLatest(now), now);
//   feed.snapshot().epoch;   // 1 after the first reading

#ifndef HERON_POSE_POSE_FEED_H
#define HERON_POSE_POSE_FEED_H

#include <cstdint>

#include "heron/odometry/Odometry.h"
#include "heron/pose/PoseHistory.h"
#include "heron/pose/PoseTracker.h"

namespace Heron::Pose {

/**
 * Thread Safety: Not thread-safe; PoseTracker holds one under its lock.
 */
class PoseFeed {
  public:
    explicit PoseFeed(PoseConfig config) : m_config(config) {}

    /**
     * Takes one poll's snapshot.
     *
     * @param o Odometry's newest reading, freshness judged at now
     * @param now When it was asked for, steady_clock
     */
    void update(const Odometry::OdometrySnapshot &o, TimePoint now);

    [[nodiscard]] const PoseSnapshot &snapshot() const noexcept { return m_snapshot; }
    [[nodiscard]] const PoseHistory &history() const noexcept { return m_history; }
    [[nodiscard]] std::uint64_t epoch() const noexcept { return m_epoch; }

  private:
    PoseConfig m_config;
    PoseHistory m_history;
    PoseSnapshot m_snapshot;
    std::uint64_t m_epoch{0};
    std::uint64_t m_lastSeq{0};
    std::uint64_t m_lastGap{0};  // readings missed before the newest
};

}  // namespace Heron::Pose

#endif  // HERON_POSE_POSE_FEED_H
