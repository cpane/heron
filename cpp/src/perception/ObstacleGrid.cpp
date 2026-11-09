// File        : ObstacleGrid.cpp
// Author      : Chris Pane
// Description : The rolling obstacle grid: cells, beams and the shift.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/perception/ObstacleGrid.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace Heron::Perception {

namespace {

constexpr int kHalf = kGridCells / 2;

// Cells per metre, multiplied rather than divided by kCellM: 0.05 is not
// exact in binary, and 2.0 / 0.05 floors to 39, putting a point on a cell
// edge into the cell below. 1 / 0.05 rounds to exactly 20, and 2.0 * 20
// is exactly 40.
constexpr double kCellsPerM = 1.0 / kCellM;

// A point no finite cell holds would make floorCellOf() undefined.
bool isFinite(Geometry::Vec2 p) noexcept {
  return std::isfinite(p.x) && std::isfinite(p.y);
}

std::int8_t clampCount(int value, int low, int high) noexcept {
  return static_cast<std::int8_t>(std::clamp(value, low, high));
}

}  // namespace

ObstacleGrid::ObstacleGrid(GridConfig config) : m_config(config) {
  reset(Geometry::Vec2{});
}

ObstacleGrid::FloorCell ObstacleGrid::floorCellOf(Geometry::Vec2 p) noexcept {
  return FloorCell{static_cast<std::int64_t>(std::floor(p.x * kCellsPerM)),
                   static_cast<std::int64_t>(std::floor(p.y * kCellsPerM))};
}

void ObstacleGrid::reset(Geometry::Vec2 centre) noexcept {
  const FloorCell c = isFinite(centre) ? floorCellOf(centre) : FloorCell{};
  m_origin = FloorCell{c.x - kHalf, c.y - kHalf};
  m_counts.fill(0);
}

void ObstacleGrid::follow(Geometry::Vec2 robot) noexcept {
  if (!isFinite(robot) || Geometry::norm(robot - centre()) <= m_config.recentreM) {
    return;
  }
  const FloorCell c = floorCellOf(robot);
  const FloorCell to{c.x - kHalf, c.y - kHalf};
  const std::int64_t dx = to.x - m_origin.x;
  const std::int64_t dy = to.y - m_origin.y;
  Counts shifted{};
  if (std::llabs(dx) < kGridCells && std::llabs(dy) < kGridCells) {
    for (int iy = 0; iy < kGridCells; ++iy) {
      const std::int64_t oy = iy + dy;
      if (oy < 0 || oy >= kGridCells) {
        continue;
      }
      for (int ix = 0; ix < kGridCells; ++ix) {
        const std::int64_t ox = ix + dx;
        if (ox >= 0 && ox < kGridCells) {
          shifted[static_cast<std::size_t>(iy * kGridCells + ix)] =
              m_counts[static_cast<std::size_t>(oy * kGridCells + ox)];
        }
      }
    }
  }
  m_origin = to;
  m_counts = shifted;
}

std::int8_t *ObstacleGrid::at(FloorCell c) noexcept {
  const std::int64_t ix = c.x - m_origin.x;
  const std::int64_t iy = c.y - m_origin.y;
  if (ix < 0 || ix >= kGridCells || iy < 0 || iy >= kGridCells) {
    return nullptr;
  }
  return &m_counts[static_cast<std::size_t>(iy * kGridCells + ix)];
}

void ObstacleGrid::addRay(Geometry::Vec2 from, Geometry::Vec2 to, bool hit) noexcept {
  if (!isFinite(from) || !isFinite(to)) {
    return;
  }
  // Bresenham's line between the two floor cells: every cell the line
  // steps through once, by whole cells, with no floating point on the way.
  const FloorCell a = floorCellOf(from);
  const FloorCell b = floorCellOf(to);
  const std::int64_t dx = std::llabs(b.x - a.x);
  const std::int64_t dy = -std::llabs(b.y - a.y);
  const std::int64_t sx = a.x < b.x ? 1 : -1;
  const std::int64_t sy = a.y < b.y ? 1 : -1;
  std::int64_t error = dx + dy;
  FloorCell c = a;
  while (c.x != b.x || c.y != b.y) {
    if (std::int8_t *count = at(c)) {
      *count = clampCount(*count + m_config.pass, m_config.passMin, m_config.hitMax);
    }
    const std::int64_t twice = 2 * error;
    if (twice >= dy) {
      error += dy;
      c.x += sx;
    }
    if (twice <= dx) {
      error += dx;
      c.y += sy;
    }
  }
  if (hit) {
    if (std::int8_t *count = at(b)) {
      *count = clampCount(*count + m_config.hit, m_config.passMin, m_config.hitMax);
    }
  }
}

Cell ObstacleGrid::classifyCount(std::int8_t count) const noexcept {
  if (count >= m_config.blockedAt) {
    return Cell::Blocked;
  }
  if (count <= m_config.freeAt) {
    return Cell::Free;
  }
  return Cell::Unknown;
}

Cell ObstacleGrid::classify(Geometry::Vec2 p) const noexcept {
  const std::optional<CellIndex> cell = cellOf(p);
  return cell ? classifyCount(count(*cell)) : Cell::Unknown;
}

std::optional<CellIndex> ObstacleGrid::cellOf(Geometry::Vec2 p) const noexcept {
  if (!isFinite(p)) {
    return std::nullopt;
  }
  const FloorCell c = floorCellOf(p);
  const std::int64_t ix = c.x - m_origin.x;
  const std::int64_t iy = c.y - m_origin.y;
  if (ix < 0 || ix >= kGridCells || iy < 0 || iy >= kGridCells) {
    return std::nullopt;
  }
  return CellIndex{static_cast<int>(ix), static_cast<int>(iy)};
}

Geometry::Vec2 ObstacleGrid::centreOf(CellIndex cell) const noexcept {
  return Geometry::Vec2{(static_cast<double>(m_origin.x + cell.ix) + 0.5) * kCellM,
                        (static_cast<double>(m_origin.y + cell.iy) + 0.5) * kCellM};
}

Geometry::Vec2 ObstacleGrid::origin() const noexcept {
  return Geometry::Vec2{static_cast<double>(m_origin.x) * kCellM,
                        static_cast<double>(m_origin.y) * kCellM};
}

Geometry::Vec2 ObstacleGrid::centre() const noexcept {
  return Geometry::Vec2{static_cast<double>(m_origin.x + kHalf) * kCellM,
                        static_cast<double>(m_origin.y + kHalf) * kCellM};
}

std::int8_t ObstacleGrid::count(CellIndex cell) const noexcept {
  return m_counts[static_cast<std::size_t>(cell.iy * kGridCells + cell.ix)];
}

}  // namespace Heron::Perception
