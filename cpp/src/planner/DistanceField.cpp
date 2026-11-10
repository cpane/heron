// File        : DistanceField.cpp
// Author      : Chris Pane
// Description : The planner's distance field: keepClear() and the transform.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "planner/DistanceField.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Heron::Planner {

namespace {

constexpr int kN = Perception::kGridCells;
constexpr double kCellsPerM = 20.0;  // multiplied, not divided: see ObstacleGrid
constexpr double kInf = std::numeric_limits<double>::infinity();

using Row = std::array<double, static_cast<std::size_t>(kN)>;

std::size_t index(int ix, int iy) noexcept {
  return static_cast<std::size_t>(iy * kN + ix);
}

std::size_t slot(int i) noexcept {
  return static_cast<std::size_t>(i);
}

/**
 * One row's squared distance transform (Felzenszwalb and Huttenlocher):
 * d[q] = min over p of (q - p)^2 + f[p], the lower envelope of the
 * parabolas rooted at each p. f is 0 at a flagged cell and infinity
 * elsewhere, or a column's results from the rows.
 */
void transform1d(const Row &f, Row &d) {
  std::array<int, static_cast<std::size_t>(kN)> v{};        // the parabolas in the envelope
  std::array<double, static_cast<std::size_t>(kN) + 1> z{};  // where each takes over
  const auto meet = [&f](int q, int p) {
    return ((f[slot(q)] + q * q) - (f[slot(p)] + p * p)) / (2.0 * q - 2.0 * p);
  };
  int k = -1;
  for (int q = 0; q < kN; ++q) {
    if (f[slot(q)] == kInf) {
      continue;  // no parabola: nothing is measured from here
    }
    while (k >= 0 && meet(q, v[slot(k)]) <= z[slot(k)]) {
      --k;
    }
    ++k;
    v[slot(k)] = q;
    z[slot(k)] = k == 0 ? -kInf : meet(q, v[slot(k - 1)]);
    z[slot(k + 1)] = kInf;
  }
  if (k < 0) {
    d.fill(kInf);
    return;
  }
  int j = 0;
  for (int q = 0; q < kN; ++q) {
    while (z[slot(j + 1)] < q) {
      ++j;
    }
    const double dq = q - v[slot(j)];
    d[slot(q)] = dq * dq + f[slot(v[slot(j)])];
  }
}

}  // namespace

CellFlags keepClear(const Perception::ObstacleGrid &grid, const Geometry::Pose2 &pose,
                    const Geometry::Outline &under, double unknownRadiusM) {
  CellFlags flags{};
  const Geometry::Vec2 at{pose.x, pose.y};
  const double r2 = unknownRadiusM * unknownRadiusM;
  for (int iy = 0; iy < kN; ++iy) {
    for (int ix = 0; ix < kN; ++ix) {
      const Perception::CellIndex cell{ix, iy};
      const Geometry::Vec2 c = grid.centreOf(cell);
      const Perception::Cell what = grid.classify(c);
      bool keep = what == Perception::Cell::Blocked;
      if (what == Perception::Cell::Unknown) {
        const Geometry::Vec2 d = c - at;
        keep = d.x * d.x + d.y * d.y <= r2 &&
               !Geometry::contains(under, Geometry::toChild(pose, c));
      }
      flags[index(ix, iy)] = keep;
    }
  }
  return flags;
}

DistanceField::DistanceField(const CellFlags &flags, Geometry::Vec2 origin) : m_origin(origin) {
  // Rows first (along x), then columns (along y), in squared cells.
  std::array<double, static_cast<std::size_t>(kN * kN)> rows{};
  Row f{};
  Row d{};
  for (int iy = 0; iy < kN; ++iy) {
    for (int ix = 0; ix < kN; ++ix) {
      f[slot(ix)] = flags[index(ix, iy)] ? 0.0 : kInf;
    }
    transform1d(f, d);
    for (int ix = 0; ix < kN; ++ix) {
      rows[index(ix, iy)] = d[slot(ix)];
    }
  }
  for (int ix = 0; ix < kN; ++ix) {
    for (int iy = 0; iy < kN; ++iy) {
      f[slot(iy)] = rows[index(ix, iy)];
    }
    transform1d(f, d);
    for (int iy = 0; iy < kN; ++iy) {
      m_metres[index(ix, iy)] = d[slot(iy)] == kInf ? kFar : std::sqrt(d[slot(iy)]) * Perception::kCellM;
    }
  }
}

double DistanceField::at(Geometry::Vec2 p) const noexcept {
  // In cells, with cell centres on whole numbers: between four centres the
  // value is interpolated, so it falls as soon as a point moves toward
  // something, not only when it crosses into the next cell (I12 needs
  // that). The distance is 1-Lipschitz, so the interpolation is out by no
  // more than half a cell's diagonal, as a nearest-cell read would be.
  const double u = (p.x - m_origin.x) * kCellsPerM - 0.5;
  const double v = (p.y - m_origin.y) * kCellsPerM - 0.5;
  if (!std::isfinite(u) || !std::isfinite(v)) {
    return 0.0;
  }
  const double last = static_cast<double>(kN - 1);
  const double cu = std::clamp(u, 0.0, last);
  const double cv = std::clamp(v, 0.0, last);
  const int i = std::min(static_cast<int>(cu), kN - 2);
  const int j = std::min(static_cast<int>(cv), kN - 2);
  const double tu = cu - i;
  const double tv = cv - j;
  const double value = (1.0 - tv) * ((1.0 - tu) * atCell(i, j) + tu * atCell(i + 1, j)) +
                       tv * ((1.0 - tu) * atCell(i, j + 1) + tu * atCell(i + 1, j + 1));
  // Outside the grid: the distance can fall by at most how far p is from it.
  const double outside = std::hypot(u - cu, v - cv) * Perception::kCellM;
  return std::max(0.0, value - outside);
}

double DistanceField::atCell(int ix, int iy) const noexcept {
  return m_metres[index(ix, iy)];
}

}  // namespace Heron::Planner
