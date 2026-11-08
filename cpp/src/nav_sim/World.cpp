// File        : World.cpp
// Author      : Chris Pane
// Description : The simulator's world: boxes and the ray cast.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "nav_sim/World.h"

#include <cmath>

namespace Heron::NavSim {

using Geometry::Vec2;

namespace {

/** The 2D cross product: positive when b is anticlockwise of a. */
double cross(Vec2 a, Vec2 b) noexcept {
  return a.x * b.y - a.y * b.x;
}

}  // namespace

void World::addBox(Vec2 centre, double length, double width) {
  const double hx = length / 2.0;
  const double hy = width / 2.0;
  const Vec2 c1 = centre + Vec2{hx, hy};
  const Vec2 c2 = centre + Vec2{-hx, hy};
  const Vec2 c3 = centre + Vec2{-hx, -hy};
  const Vec2 c4 = centre + Vec2{hx, -hy};
  segments.push_back(Segment{c1, c2});
  segments.push_back(Segment{c2, c3});
  segments.push_back(Segment{c3, c4});
  segments.push_back(Segment{c4, c1});
}

std::optional<double> World::cast(Vec2 from, double angleRad, double maxRangeM) const {
  // The ray from + t d meets the segment a + u (b - a) where both agree:
  // solve for t (along the ray, >= 0) and u (along the segment, 0..1).
  const Vec2 d{std::cos(angleRad), std::sin(angleRad)};
  std::optional<double> nearest;
  for (const Segment &s : segments) {
    const Vec2 e = s.b - s.a;
    const double denom = cross(d, e);
    if (std::abs(denom) < 1e-12) {
      continue;  // parallel: a ray along a wall does not see it
    }
    const Vec2 w = s.a - from;
    const double t = cross(w, e) / denom;
    const double u = cross(w, d) / denom;
    if (t < 0.0 || u < 0.0 || u > 1.0 || t > maxRangeM) {
      continue;
    }
    if (!nearest || t < *nearest) {
      nearest = t;
    }
  }
  return nearest;
}

}  // namespace Heron::NavSim
