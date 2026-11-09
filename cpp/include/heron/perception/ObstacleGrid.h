// File        : ObstacleGrid.h
// Author      : Chris Pane
// Description : The rolling obstacle grid: 4 x 4 m of 5 cm cells round
//               Heron, each a count of hits and passes, in the odom frame.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// What the planner sees the room as (docs/navigation-design.md §6). Each
// LiDAR point is a beam from the LiDAR to something: the cell it ends in
// holds something (a hit, +3), and every cell it crossed on the way holds
// nothing (a pass, -1). The counts saturate, at +12 and -8, so a wall seen
// for a minute still clears in a few scans once it moves, and a cell
// counts as
//
//   Blocked  at +3 or more: one hit on an unknown cell; a cell seen free
//            (at -2 or below) takes two
//   Free     at -2 or less: two beams through it, and nothing since
//   Unknown  anything else, never seen included. Unknown is not free (P6)
//
// The grid is in the odom frame, so obstacles stay put while Heron turns
// and slides, and rolls with Heron: when Heron is more than 0.5 m from its
// centre, it shifts by whole cells to put Heron back in the middle; cells
// shifted out are forgotten and cells shifted in are unknown. Cell edges
// sit on multiples of 5 cm in the odom frame, whatever the shifts, so a
// shift never blurs what the cells hold.
//
// Callers hand it points in the odom frame, never cell indices; cells are
// in reach through cellOf() and counts(), for tests and the telemetry.
//
// Pure: no clock, no thread. 6.4 KB of counts, copied whole per scan by
// Perception (design §6.4). Design: docs/navigation-cpp-design.md §5.1.
//
// Usage
// -----
//   ObstacleGrid grid;                         // the starting counts, §6.1
//   grid.reset(Vec2{0.0, 0.0});
//   grid.follow(robotAt);                      // each scan, before its rays
//   grid.addRay(lidarAt, pointAt, true);       // a beam that hit something
//   grid.classify(pointAt);                    // Blocked: +3
//   grid.classify(halfwayThere);               // Unknown: one pass is -1...
//   grid.addRay(lidarAt, pointAt, true);
//   grid.classify(halfwayThere);               // ...Free after the second

#ifndef HERON_PERCEPTION_OBSTACLE_GRID_H
#define HERON_PERCEPTION_OBSTACLE_GRID_H

#include <array>
#include <cstdint>
#include <optional>

#include "heron/geometry/Pose2.h"

namespace Heron::Perception {

inline constexpr int kGridCells = 80;    // each way
inline constexpr double kCellM = 0.05;

/** The counts of design §6.1; starting values, to be tuned (P5). */
struct GridConfig {
  std::int8_t hit{3};         // added to the cell a beam ends in
  std::int8_t hitMax{12};
  std::int8_t pass{-1};       // added to each cell a beam crosses
  std::int8_t passMin{-8};
  std::int8_t blockedAt{3};   // this or more: Blocked
  std::int8_t freeAt{-2};     // this or less: Free
  double recentreM{0.5};      // Heron this far from the centre: shift
};

enum class Cell : std::uint8_t { Unknown, Free, Blocked };

/** A cell of the grid: column ix along x, row iy along y, each 0 to 79. */
struct CellIndex {
  int ix{0};
  int iy{0};
};

/**
 * 80 x 80 signed counts, rolling with Heron, in the odom frame.
 *
 * Thread Safety: a plain value, not thread-safe. Perception copies it per
 * scan and publishes the copy, never changed after.
 */
class ObstacleGrid {
  public:
    using Counts = std::array<std::int8_t, kGridCells * kGridCells>;

    /** An unknown grid centred on the odom frame's origin. */
    explicit ObstacleGrid(GridConfig config = {});

    /**
     * Forgets everything: every cell unknown, the grid centred on the cell
     * holding `centre`, its edges on multiples of 5 cm.
     */
    void reset(Geometry::Vec2 centre) noexcept;

    /**
     * Shifts by whole cells, when `robot` is more than recentreM from the
     * centre, so that the robot's cell is the centre's; cells shifted in
     * are unknown (design §6.3). Otherwise does nothing.
     */
    void follow(Geometry::Vec2 robot) noexcept;

    /**
     * One beam: lowers each cell on the line from `from` to `to`, `from`'s
     * included and `to`'s not, then raises `to`'s if it is a hit. Either
     * end may lie outside the grid; only the cells inside change.
     *
     * @param from Where the beam left, the LiDAR's spin axis, odom frame
     * @param to Where it ended, odom frame
     * @param hit true if something was there, which raises `to`'s cell;
     *            false leaves that cell as it is
     */
    void addRay(Geometry::Vec2 from, Geometry::Vec2 to, bool hit) noexcept;

    /** @return What the cell holding p holds; outside the grid, Unknown */
    [[nodiscard]] Cell classify(Geometry::Vec2 p) const noexcept;

    /** @return The cell holding p, or none outside the grid */
    [[nodiscard]] std::optional<CellIndex> cellOf(Geometry::Vec2 p) const noexcept;

    /** @return The centre of a cell, odom frame */
    [[nodiscard]] Geometry::Vec2 centreOf(CellIndex cell) const noexcept;

    /** @return The corner of cell (0, 0), the grid's lowest x and y */
    [[nodiscard]] Geometry::Vec2 origin() const noexcept;

    /** @return The grid's centre, where follow() measures from */
    [[nodiscard]] Geometry::Vec2 centre() const noexcept;

    /** @return One cell's count */
    [[nodiscard]] std::int8_t count(CellIndex cell) const noexcept;

    /** @return Every count, row by row: cell (ix, iy) at iy * 80 + ix */
    [[nodiscard]] const Counts &counts() const noexcept { return m_counts; }

    [[nodiscard]] const GridConfig &config() const noexcept { return m_config; }

  private:
    /** A cell of the whole floor, in 5 cm steps from the odom origin. */
    struct FloorCell {
      std::int64_t x{0};
      std::int64_t y{0};
    };

    [[nodiscard]] static FloorCell floorCellOf(Geometry::Vec2 p) noexcept;
    [[nodiscard]] std::int8_t *at(FloorCell c) noexcept;  // nullptr outside
    [[nodiscard]] Cell classifyCount(std::int8_t count) const noexcept;

    GridConfig m_config;
    FloorCell m_origin{};  // the floor cell of grid cell (0, 0)
    Counts m_counts{};
};

}  // namespace Heron::Perception

#endif  // HERON_PERCEPTION_OBSTACLE_GRID_H
