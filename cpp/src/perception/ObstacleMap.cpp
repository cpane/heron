// File        : ObstacleMap.cpp
// Author      : Chris Pane
// Description : addScan(): one revolution into the obstacle map.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/perception/ObstacleMap.h"

#include <vector>

namespace Heron::Perception {

namespace {

/** One beam, placed: where the LiDAR was and where the point was. */
struct Beam {
  Geometry::Vec2 from;
  Geometry::Vec2 to;
};

}  // namespace

ScanOutcome addScan(ObstacleMap &map, const Heron::Scan &scan, const Pose::PoseHistory &history,
                    std::uint64_t epoch, const LidarMount &mount, TimePoint now,
                    Duration maxScanAge) {
  // A new frame first: the old cells mean nothing in it, whatever happens
  // to this scan.
  const bool newFrame = epoch != map.epoch;
  if (newFrame) {
    const std::optional<Pose::PoseSample> newest = history.newest();
    map.grid.reset(newest ? Geometry::Vec2{newest->pose.x, newest->pose.y} : Geometry::Vec2{});
    map.epoch = epoch;
    map.scanSeq = 0;
    map.scanEnd = TimePoint{};
    map.poseAtScanEnd = Geometry::Pose2{};
  }

  if (now - scan.tEnd > maxScanAge) {
    return ScanOutcome::Stale;
  }
  if (!scan.isCoverageOk()) {
    return ScanOutcome::NotCovered;
  }

  const Pose::PoseLookup atEnd = history.at(scan.tEnd);
  if (!atEnd.ok) {
    return ScanOutcome::NoPose;
  }
  // Every pose before any cell changes, so a missing one leaves the map as
  // it was. About 200 points indoors; the vector is the scan's only
  // allocation.
  std::vector<Beam> beams;
  beams.reserve(scan.points.size());
  for (const Heron::Point &point : scan.points) {
    const Pose::PoseLookup then = history.at(scan.timeOf(point.bearingRad));
    if (!then.ok) {
      return ScanOutcome::NoPose;
    }
    const Geometry::Vec2 inRobot =
        Geometry::polarToRobot(point.bearingRad, point.rangeM, mount.offset);
    beams.push_back(Beam{Geometry::toParent(then.pose, mount.offset),
                         Geometry::toParent(then.pose, inRobot)});
  }

  map.grid.follow(Geometry::Vec2{atEnd.pose.x, atEnd.pose.y});
  for (const Beam &beam : beams) {
    map.grid.addRay(beam.from, beam.to, true);
  }
  map.scanSeq = scan.seq;
  map.scanEnd = scan.tEnd;
  map.poseAtScanEnd = atEnd.pose;
  return newFrame ? ScanOutcome::NewFrame : ScanOutcome::Added;
}

const char *toString(ScanOutcome outcome) noexcept {
  switch (outcome) {
    case ScanOutcome::Added:
      return "ADDED";
    case ScanOutcome::Stale:
      return "STALE";
    case ScanOutcome::NotCovered:
      return "NOT_COVERED";
    case ScanOutcome::NoPose:
      return "NO_POSE";
    case ScanOutcome::NewFrame:
      return "NEW_FRAME";
  }
  return "UNKNOWN";
}

}  // namespace Heron::Perception
