// File        : World.h
// Author      : Chris Pane
// Description : The simulator's world: walls and boxes as line segments, and
//               the ray cast the simulated LiDAR measures them with.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// heron_nav_sim lets the navigation libraries be tested on the Mac with no
// robot (docs/navigation-design.md §12, docs/navigation-cpp-design.md §3).
// Three pieces, each a header here:
//
//   World   what is in the room, in the odom frame
//   Base    how Heron moves when told to: mecanum, accel limits, a lag
//   Lidar   the A1M8 as measured, turning while Heron moves, so its scans
//           carry the same skew a real one does
//
// No threads and no clock: a test owns the time and moves it on, calling
// Base::advance() and Lidar::advance() with the same `now`.
//
// The world is flat line segments: a wall is one, a box four. Nothing is
// round and nothing moves; that is enough for the limiter, perception and
// planner tests, and a moving box is a later addition when one needs it.
//
// Usage
// -----
//   Heron::NavSim::World world;
//   world.addBox(Vec2{0.0, 0.0}, 4.0, 4.0);       // a 4 m square room, seen from inside
//   world.addBox(Vec2{1.2, 0.0}, 0.3, 0.3);       // a box in it
//   world.cast(Vec2{0.0, 0.0}, 0.0, 12.0);        // 1.05: the box's near face

#ifndef HERON_NAV_SIM_WORLD_H
#define HERON_NAV_SIM_WORLD_H

#include <optional>
#include <vector>

#include "heron/geometry/Pose2.h"

namespace Heron::NavSim {

/** A wall, or one side of a box, from a to b, odom frame. */
struct Segment {
  Geometry::Vec2 a;
  Geometry::Vec2 b;
};

/** Walls and boxes, as line segments, in the odom frame. */
struct World {
  std::vector<Segment> segments;

  /**
   * Adds the four sides of a rectangle with its sides along the axes:
   * length along x, width along y. Seen from inside, it is a room.
   */
  void addBox(Geometry::Vec2 centre, double length, double width);

  /**
   * The first hit along a ray.
   *
   * @param from Where the ray starts, odom frame
   * @param angleRad Its direction in the odom frame, anticlockwise from +x
   * @param maxRangeM How far to look
   * @return The distance to the nearest segment, or no value within maxRangeM
   */
  [[nodiscard]] std::optional<double> cast(Geometry::Vec2 from, double angleRad,
                                           double maxRangeM) const;
};

}  // namespace Heron::NavSim

#endif  // HERON_NAV_SIM_WORLD_H
