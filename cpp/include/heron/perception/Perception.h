// File        : Perception.h
// Author      : Chris Pane
// Description : The obstacle map for the robot: a thread adding each new
//               LiDAR scan to a copy of the map, and publishing the copy.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The thin wrapper round addScan() (docs/navigation-design.md §4.1, P2): it
// owns the thread and reads the clock. Every 10 ms it asks the LiDAR for
// its newest scan; a new one (a new seq) is added to a copy of the last
// map, with the pose history as PoseTracker holds it, and the copy is
// published. The planner keeps reading the map it has meanwhile; a
// published map is never changed (design §6.4).
//
// A scan addScan() skips (stale, not covered, no pose) publishes nothing,
// unless it opened a new epoch: then the reset map is published, since the
// last one is in a frame that no longer exists.
//
// Shaped like Lidar and PoseTracker: a thread, and getLatest() that never
// waits for long. Design: docs/navigation-cpp-design.md §5.2.
//
// Usage
// -----
//   Heron::Perception::Perception perception(lidar, tracker, config);
//   perception.start();
//   if (const auto map = perception.getLatest()) {
//     map->grid.classify(Vec2{1.0, 0.0});      // Blocked, Free or Unknown
//   }

#ifndef HERON_PERCEPTION_PERCEPTION_H
#define HERON_PERCEPTION_PERCEPTION_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "heron/lidar/Lidar.h"
#include "heron/perception/ObstacleGrid.h"
#include "heron/perception/ObstacleMap.h"
#include "heron/pose/PoseTracker.h"

namespace Heron::Perception {

/** The map's settings, filled by heron_robot from the profile (I6). */
struct PerceptionConfig {
  LidarMount mount{};
  GridConfig grid{};
  Duration maxScanAge{kMaxScanAge};
  Duration pollPeriod{std::chrono::milliseconds(10)};
};

/** Counters, for the robot's health reporting. */
struct PerceptionHealth {
  std::uint64_t scansAdded{0};    // Added or NewFrame
  std::uint64_t scansSkipped{0};  // Stale, NotCovered or NoPose
  std::uint64_t mapResets{0};     // a new epoch reset the map
  ScanOutcome lastOutcome{ScanOutcome::Added};
  std::string lastError;          // why the last skipped scan was skipped
};

/**
 * The obstacle map, built from the LiDAR and the pose by a thread of its own.
 *
 * Thread Safety: getLatest() and getHealth() may be called from any thread
 * at any rate and never block for long. start() and stop() are for one
 * owning thread. The Lidar and the PoseTracker must outlive it.
 */
class Perception {
  public:
    /**
     * @param lidar Its newest scan is read each poll; started or not
     * @param pose Its history is copied for each new scan
     * @param config Where the LiDAR sits, the counts and the timings
     */
    Perception(const Lidar::Lidar &lidar, const Pose::PoseTracker &pose, PerceptionConfig config);

    /** Stops the thread. */
    ~Perception();

    Perception(const Perception &) = delete;
    Perception &operator=(const Perception &) = delete;

    /**
     * Starts the thread. A no-op while running.
     *
     * @return true; the type leaves room for a start that can fail
     */
    bool start();

    /** Stops the thread; returns within about one poll and a scan. Safe to
     *  repeat. */
    void stop();

    /** @return The newest map, or nullptr before the first */
    [[nodiscard]] std::shared_ptr<const ObstacleMap> getLatest() const;

    /** @return A snapshot of the counters */
    [[nodiscard]] PerceptionHealth getHealth() const;

  private:
    struct Impl;  // the thread, the lock and the newest map live here
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Heron::Perception

#endif  // HERON_PERCEPTION_PERCEPTION_H
