// File        : ObstacleMap.h
// Author      : Chris Pane
// Description : One obstacle map as Perception publishes it, and addScan():
//               a LiDAR revolution placed into it, each point at its own
//               time.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The core of heron_perception (docs/navigation-design.md §6.2). addScan()
// takes one Scan and the pose history, and for each point:
//
//   1. finds where Heron was when that point was measured,
//      history.at(scan.timeOf(bearing)), so a scan taken while turning
//      lands straight (deskewing, design §3.3);
//   2. places the LiDAR and the point in the odom frame, from the pose and
//      the LiDAR's mounting offset;
//   3. adds the beam to the grid: passes along it, a hit at its end.
//
// Before any of that it checks the scan, as the LiDAR's consumer rules
// say (docs/lidar-cpp-design.md §7.1): not older than maxScanAge, the
// whole revolution observed. And it looks up every point's pose first: if
// one is missing, the scan is skipped whole, so a map never holds half a
// scan.
//
// The Scan holds valid returns only, so a no-return clears nothing, as it
// must: glass and dark surfaces return nothing and are still there.
//
// Epochs (I8): the map remembers the odom frame it is in. A history from
// another epoch means odometry restarted; every cell is in a frame that no
// longer exists, so the map is reset first and the result says NewFrame.
// The reset holds even when the scan is then skipped.
//
// The bearing's direction is the LiDAR's own: Scan's points are already
// turned by the housing's yaw (the LiDAR's mountingOffsetRad, P4), so only
// the spin axis's position is needed here.
//
// Pure: the time is an argument. Design: docs/navigation-cpp-design.md §5.1.
//
// Usage
// -----
//   ObstacleMap map{ObstacleGrid(gridConfig)};
//   const auto [history, epoch] = tracker.copyHistory();
//   const ScanOutcome outcome =
//       addScan(map, *scan, history, epoch, LidarMount{{0.06, 0.0}}, Clock::now());
//   if (outcome == ScanOutcome::Added || outcome == ScanOutcome::NewFrame) {
//     map.grid.classify(Vec2{1.5, 1.93});   // Blocked, after a second scan
//   }

#ifndef HERON_PERCEPTION_OBSTACLE_MAP_H
#define HERON_PERCEPTION_OBSTACLE_MAP_H

#include <chrono>
#include <cstdint>

#include "heron/geometry/Pose2.h"
#include "heron/lidar/Scan.h"
#include "heron/perception/ObstacleGrid.h"
#include "heron/pose/PoseHistory.h"

namespace Heron::Perception {

/** One map, as Perception publishes it: never changed once published. */
struct ObstacleMap {
  ObstacleGrid grid;
  std::uint64_t epoch{0};          // the odom frame it is in (I8); 0: none yet
  std::uint64_t scanSeq{0};        // the newest scan in it
  TimePoint scanEnd{};
  Geometry::Pose2 poseAtScanEnd{};
};

/** What addScan() did. */
enum class ScanOutcome : std::uint8_t {
  Added,       // the scan is in the map
  Stale,       // older than maxScanAge: skipped
  NotCovered,  // part of the revolution not observed: skipped
  NoPose,      // a point's pose, or the scan end's, not in the history: skipped
  NewFrame     // a new epoch: the map was reset, and the scan is in it
};

/** Where the LiDAR's spin axis is, robot frame: the profile's [lidar] x, y.
 *  Its yaw stays in the LiDAR library, which turns the bearings (P4). */
struct LidarMount {
  Geometry::Vec2 offset{};
};

/** The LiDAR's freshness rule, as the map applies it: about two revolutions. */
inline constexpr Duration kMaxScanAge = std::chrono::milliseconds(300);

/**
 * Adds one scan to a map (design §6.2).
 *
 * @param map The map to add to; reset first when epoch differs from its own
 * @param scan The revolution
 * @param history Heron's poses, in epoch's frame
 * @param epoch The history's epoch, from PoseTracker::copyHistory()
 * @param mount Where the LiDAR sits on the robot
 * @param now The time to judge the scan's age at, steady_clock
 * @param maxScanAge Older scans are skipped
 * @return Added or NewFrame when the scan is in the map; otherwise why not
 */
ScanOutcome addScan(ObstacleMap &map, const Heron::Scan &scan, const Pose::PoseHistory &history,
                    std::uint64_t epoch, const LidarMount &mount, TimePoint now,
                    Duration maxScanAge = kMaxScanAge);

/** @return "ADDED", "STALE", "NOT_COVERED", "NO_POSE" or "NEW_FRAME" */
[[nodiscard]] const char *toString(ScanOutcome outcome) noexcept;

}  // namespace Heron::Perception

#endif  // HERON_PERCEPTION_OBSTACLE_MAP_H
