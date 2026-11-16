// File        : MotionTable.cpp
// Author      : Chris Pane
// Description : The measured motion table: columns, interpolation, and the
//               two inverse lookups.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/motion/MotionTable.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace Heron::Motion {

namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Within this of a want's axis, a prediction counts as not going past it:
// the OTOS's noise on a settled speed is about this.
constexpr double kAxisToleranceMps = 0.005;

Geometry::Twist blend(const Geometry::Twist &a, const Geometry::Twist &b, double t) {
  return Geometry::Twist{a.vx + (b.vx - a.vx) * t, a.vy + (b.vy - a.vy) * t,
                         a.wz + (b.wz - a.wz) * t};
}

// The mean of directions, taken on the circle, so -179 and 179 average to 180.
double meanDirection(const std::vector<MotionPoint> &points) {
  double x = 0.0;
  double y = 0.0;
  for (const MotionPoint &p : points) {
    x += std::cos(p.directionRad);
    y += std::sin(p.directionRad);
  }
  return std::atan2(y, x);
}

// How far b lies counterclockwise of a, in [0, 2 pi).
double ccwFrom(double a, double b) {
  double d = std::fmod(b - a, kTwoPi);
  return d < 0.0 ? d + kTwoPi : d;
}

// Stalls lifted (header): the point after a column's last stall, if one
// follows it; otherwise the first, and the column reads as measured.
bool isStall(const MotionPoint &p) {
  return std::hypot(p.made.vx, p.made.vy) < kStallMps;
}

std::size_t pointAfterStalls(const std::vector<MotionPoint> &points) {
  std::size_t first = 0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    if (isStall(points[i])) {
      first = i + 1;
    }
  }
  return isStall(points[first]) ? 0 : first;
}

bool within(double v, double want) {
  return v >= std::min(0.0, want) - kAxisToleranceMps && v <= std::max(0.0, want) + kAxisToleranceMps;
}

}  // namespace

MotionTable::MotionTable(std::vector<MotionPoint> points, bool mirror) {
  for (MotionPoint &p : points) {
    if (!(p.size > 0.0)) {
      throw std::invalid_argument("Heron::Motion::MotionTable() : a point's size must be above 0");
    }
    p.directionRad = std::atan2(std::sin(p.directionRad), std::cos(p.directionRad));
  }
  std::sort(points.begin(), points.end(),
            [](const MotionPoint &a, const MotionPoint &b) { return a.directionRad < b.directionRad; });

  // Columns: a point joins the open column while within kColumnWidthRad of
  // the point before it, so a run of directions a few degrees apart (the
  // hardwood's right, -75 to -90) is one column, and two 15 apart are two.
  std::vector<std::vector<MotionPoint>> groups;
  for (const MotionPoint &p : points) {
    if (groups.empty() || p.directionRad - groups.back().back().directionRad > kColumnWidthRad) {
      groups.emplace_back();
    }
    groups.back().push_back(p);
  }
  // Across +-180: the last column and the first may be one.
  if (groups.size() > 1 &&
      ccwFrom(groups.back().back().directionRad, groups.front().front().directionRad) <= kColumnWidthRad) {
    groups.front().insert(groups.front().end(), groups.back().begin(), groups.back().end());
    groups.pop_back();
  }

  for (std::vector<MotionPoint> &g : groups) {
    Column column;
    column.directionRad = meanDirection(g);
    std::sort(g.begin(), g.end(), [](const MotionPoint &a, const MotionPoint &b) { return a.size < b.size; });
    // Two runs at one size: their mean.
    for (const MotionPoint &p : g) {
      if (!column.points.empty() && std::abs(column.points.back().size - p.size) < 1e-6) {
        column.points.back().made = blend(column.points.back().made, p.made, 0.5);
      } else {
        column.points.push_back(p);
      }
    }
    column.firstRead = pointAfterStalls(column.points);
    m_columns.push_back(std::move(column));
  }

  if (mirror) {
    const std::vector<Column> measured = m_columns;
    for (const Column &c : measured) {
      const double opposite = std::atan2(std::sin(-c.directionRad), std::cos(-c.directionRad));
      const bool onAxis = std::abs(std::remainder(c.directionRad - opposite, kTwoPi)) <= kColumnWidthRad;
      const bool taken = std::any_of(m_columns.begin(), m_columns.end(), [&](const Column &o) {
        return std::abs(std::remainder(o.directionRad - opposite, kTwoPi)) <= kColumnWidthRad;
      });
      if (onAxis || taken) {
        continue;
      }
      Column m;
      m.directionRad = opposite;
      for (MotionPoint p : c.points) {
        p.directionRad = opposite;
        p.made.vy = -p.made.vy;
        p.made.wz = -p.made.wz;
        m.points.push_back(p);
      }
      m.firstRead = c.firstRead;
      m_columns.push_back(std::move(m));
    }
  }

  std::sort(m_columns.begin(), m_columns.end(),
            [](const Column &a, const Column &b) { return a.directionRad < b.directionRad; });
  if (m_columns.size() < 2) {
    throw std::invalid_argument("Heron::Motion::MotionTable() : the points form fewer than 2 columns");
  }
}

Geometry::Twist MotionTable::inColumn(const Column &column, double size, bool liftStalls) const {
  const std::vector<MotionPoint> &p = column.points;
  const std::size_t first = liftStalls ? column.firstRead : 0;
  if (!liftStalls && size < p[column.firstRead].size - 1e-9) {
    return Geometry::Twist{};  // from rest, short of a size seen to move: nothing (header)
  }
  if (size <= p[first].size) {
    return p[first].made;  // the dead band, or a stall, lifts any command to it
  }
  if (size >= p.back().size) {
    return p.back().made;
  }
  for (std::size_t i = first + 1; i < p.size(); ++i) {
    if (size <= p[i].size) {
      return blend(p[i - 1].made, p[i].made, (size - p[i - 1].size) / (p[i].size - p[i - 1].size));
    }
  }
  return p.back().made;
}

Geometry::Twist MotionTable::predict(double vxFraction, double vyFraction) const {
  return read(vxFraction, vyFraction, true);
}

Geometry::Twist MotionTable::predictFromRest(double vxFraction, double vyFraction) const {
  return read(vxFraction, vyFraction, false);
}

Geometry::Twist MotionTable::read(double vxFraction, double vyFraction, bool liftStalls) const {
  const double size = std::hypot(vxFraction, vyFraction);
  if (size < 1e-9) {
    return Geometry::Twist{};
  }
  const double direction = std::atan2(vyFraction, vxFraction);
  // The column at or clockwise of the direction, and the next one round.
  std::size_t lo = m_columns.size() - 1;
  for (std::size_t i = 0; i < m_columns.size(); ++i) {
    if (m_columns[i].directionRad <= direction) {
      lo = i;
    }
  }
  const std::size_t hi = (lo + 1) % m_columns.size();
  const double span = ccwFrom(m_columns[lo].directionRad, m_columns[hi].directionRad);
  const double t = span > 1e-12 ? ccwFrom(m_columns[lo].directionRad, direction) / span : 0.0;
  return blend(inColumn(m_columns[lo], size, liftStalls), inColumn(m_columns[hi], size, liftStalls),
               std::clamp(t, 0.0, 1.0));
}

std::optional<std::pair<double, double>> MotionTable::commandFor(double directionRad,
                                                                 double speedMps) const {
  const double cx = std::cos(directionRad);
  const double cy = std::sin(directionRad);
  std::optional<double> best;
  for (int k = 1; k <= 200; ++k) {
    const double s = 0.005 * k;
    const Geometry::Twist t = predict(cx * s, cy * s);
    if (std::hypot(t.vx, t.vy) > speedMps + 1e-9) {
      break;  // never faster than asked: stop at the first size past it
    }
    best = s;
  }
  if (!best) {
    return std::nullopt;
  }
  return std::make_pair(cx * *best, cy * *best);
}

std::optional<std::pair<double, double>> MotionTable::nearestCommand(Geometry::Vec2 want, double aroundRad,
                                                                     double withinRad) const {
  if (std::hypot(want.x, want.y) < 1e-3) {
    return std::nullopt;
  }
  constexpr double kStepRad = 5.0 * Geometry::kDegToRad;
  const int steps = static_cast<int>(std::floor(withinRad / kStepRad + 1e-9));
  std::optional<std::pair<double, double>> best;
  double bestMiss = 0.0;
  for (int d = -steps; d <= steps; ++d) {
    const double direction = aroundRad + d * kStepRad;
    for (int k = 1; k <= 20; ++k) {
      const double s = 0.05 * k;
      const double cx = std::cos(direction) * s;
      const double cy = std::sin(direction) * s;
      const Geometry::Twist t = predict(cx, cy);
      if (std::hypot(t.vx, t.vy) < 0.01 || !within(t.vx, want.x) || !within(t.vy, want.y)) {
        continue;
      }
      const double miss = std::hypot(t.vx - want.x, t.vy - want.y);
      if (!best || miss < bestMiss) {
        best = std::make_pair(cx, cy);
        bestMiss = miss;
      }
    }
  }
  return best;
}

double MotionTable::leastSpeed(double directionRad) const {
  const Geometry::Twist t = predict(std::cos(directionRad) * 1e-3, std::sin(directionRad) * 1e-3);
  return std::hypot(t.vx, t.vy);
}

}  // namespace Heron::Motion
