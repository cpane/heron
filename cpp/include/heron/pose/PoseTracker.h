// File        : PoseTracker.h
// Author      : Chris Pane
// Description : Heron's pose for the robot: a thread copying each odometry
//               reading into a PoseHistory, read from any thread.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The thin wrapper round PoseHistory (docs/navigation-design.md §4.1, P2):
// it owns the thread and reads the clock; the decisions are in the core.
// Every 10 ms it asks Odometry for its newest reading, and a new one (a new
// seq) goes into the history at its own time, the poll's time minus the
// reading's age. Shaped like Odometry and Lidar: a reader thread, and
// getLatest() that never waits for long.
//
// Two things it adds to what Odometry says:
//
//   - The epoch: which odom frame the poses are in (I8). Odometry's seq
//     starts again from 1 when it restarts, and its frame with it, so a
//     seq lower than the last means every stored pose is meaningless: the
//     history is cleared and the epoch goes up. Perception compares epochs
//     to know when to clear its map. 0 before the first reading.
//   - Freshness over the history: not fresh when Odometry is not OK, or
//     when more than maxSeqGap readings were missed between two polls.
//
// It assumes it polls at least once while Odometry restarts and counts
// back up past its old seq; at 10 ms against a 0.65 s start-up, it does.
//
// The velocity is passed on as the robot-frame twist PoseSample promises.
// Whether the OTOS gives it in its own frame or the odom frame is not
// measured yet (docs/navigation-design.md §18); if the odom frame, it is
// turned here, with rotate(v, -heading), once N1 has checked it.
//
// Design: docs/navigation-cpp-design.md §4.2.
//
// Usage
// -----
//   Heron::Odometry::Odometry odometry(otosConfig);
//   odometry.start();                              // still and level
//   Heron::Pose::PoseTracker tracker(odometry);
//   tracker.start();
//   const PoseSnapshot now = tracker.getLatest();  // the newest pose
//   const auto [history, epoch] = tracker.copyHistory();
//   const PoseLookup then = history.at(scan.timeOf(bearing));

#ifndef HERON_POSE_POSE_TRACKER_H
#define HERON_POSE_POSE_TRACKER_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "heron/odometry/Odometry.h"
#include "heron/pose/PoseHistory.h"

namespace Heron::Pose {

/** How the tracker polls. */
struct PoseConfig {
  Duration pollPeriod{std::chrono::milliseconds(10)};
  std::uint64_t maxSeqGap{10};  // more readings missed than this: not fresh
};

/** The newest pose, as of the call. */
struct PoseSnapshot {
  bool fresh{false};            // Odometry OK and no gap over maxSeqGap
  std::uint64_t epoch{0};       // which odom frame (I8); 0 before the first
  PoseSample newest{};
  bool tiltWarning{false};      // passed on, not acted on
  bool trackingWarning{false};
  std::string reason;           // why not fresh, for the robot channel
};

/**
 * Heron's pose, from the odometry library, by a thread of its own.
 *
 * Thread Safety: getLatest() and copyHistory() may be called from any
 * thread at any rate and never block for long. start() and stop() are for
 * one owning thread. The Odometry must outlive the tracker.
 */
class PoseTracker {
  public:
    /**
     * @param odometry Read by the thread; started or not, the tracker waits
     * @param config How often to poll, and the gap that is not fresh
     */
    explicit PoseTracker(const Odometry::Odometry &odometry, PoseConfig config = {});

    /** Stops the thread. */
    ~PoseTracker();

    PoseTracker(const PoseTracker &) = delete;
    PoseTracker &operator=(const PoseTracker &) = delete;

    /**
     * Polls once, then starts the thread. A no-op while running.
     *
     * @return true; the type leaves room for a tracker that can fail
     */
    bool start();

    /** Stops the thread; returns within about one poll. Safe to repeat. */
    void stop();

    /** @return The newest pose, freshness judged at the last poll */
    [[nodiscard]] PoseSnapshot getLatest() const;

    /** @return A copy of the history and its epoch, taken together under
     *          the lock, so the poses are all in that epoch's frame */
    [[nodiscard]] std::pair<PoseHistory, std::uint64_t> copyHistory() const;

  private:
    struct Impl;  // the thread, the lock and the core live here
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Heron::Pose

#endif  // HERON_POSE_POSE_TRACKER_H
