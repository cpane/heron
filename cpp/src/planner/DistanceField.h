// File        : DistanceField.h
// Author      : Chris Pane
// Description : For every cell of the obstacle grid, how far it is from the
//               nearest cell Heron must keep clear of. Private to the planner.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The planner checks about 121 candidate motions a cycle, each at 30 steps
// round 34 points of the outline's edge. Asking the map "how near is the
// nearest obstacle?" at each of those points is what makes that cheap: the
// answer for every cell is worked out once a cycle, here, and each point
// then costs four array reads, interpolated (docs/navigation-cpp-design.md
// §8.4, I10).
//
// Which cells to keep clear of is keepClear(): the blocked ones, and the
// unknown ones near Heron (P6) except those under Heron itself. The
// distances are exact Euclidean ones between cell centres, by the
// two-pass transform of Felzenszwalb and Huttenlocher ("Distance
// Transforms of Sampled Functions", 2012): the squared distance along each
// row first, then down each column, each a lower envelope of parabolas.
// 6,400 cells, about 0.05 ms on the Mac.
//
// Pure: no clock, no thread. Tested against brute force by planner_test.

#ifndef HERON_PLANNER_DISTANCE_FIELD_H
#define HERON_PLANNER_DISTANCE_FIELD_H

#include <array>

#include "heron/geometry/Outline.h"
#include "heron/geometry/Pose2.h"
#include "heron/perception/ObstacleGrid.h"

namespace Heron::Planner {

/** One flag per grid cell, laid out as ObstacleGrid::counts(): (ix, iy) at
 *  iy * 80 + ix. */
using CellFlags = std::array<bool, Perception::kGridCells * Perception::kGridCells>;

/**
 * The cells Heron must keep clear of: every Blocked cell, and every Unknown
 * one whose centre is within unknownRadiusM of Heron's position, except
 * those whose centre lies inside `under` placed at Heron's pose: what the
 * LiDAR cannot see, its blind circle and the robot's own body.
 */
[[nodiscard]] CellFlags keepClear(const Perception::ObstacleGrid &grid, const Geometry::Pose2 &pose,
                                  const Geometry::Outline &under, double unknownRadiusM);

/**
 * Distances to the nearest flagged cell, in metres, between cell centres.
 *
 * Thread Safety: a plain value.
 */
class DistanceField {
  public:
    /** With no flagged cell at all, every distance is this. */
    static constexpr double kFar = 1000.0;

    /**
     * @param flags The cells to measure from
     * @param origin The corner of cell (0, 0), odom frame (ObstacleGrid::origin())
     */
    DistanceField(const CellFlags &flags, Geometry::Vec2 origin);

    /** @return The distance at p, interpolated between the four nearest
     *          cell centres. Outside the grid, the edge's less how far
     *          outside p is */
    [[nodiscard]] double at(Geometry::Vec2 p) const noexcept;

    /** @return The distance at one cell */
    [[nodiscard]] double atCell(int ix, int iy) const noexcept;

  private:
    Geometry::Vec2 m_origin;
    std::array<double, Perception::kGridCells * Perception::kGridCells> m_metres{};
};

}  // namespace Heron::Planner

#endif  // HERON_PLANNER_DISTANCE_FIELD_H
