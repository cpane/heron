// File        : Outline.cpp
// Author      : Chris Pane
// Description : The robot's rectangle and the sweep: firstContact().
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/geometry/Outline.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace Heron::Geometry {

namespace {

// Slower than this, an axis counts as not moving: 1 um/s.
constexpr double kStill = 1e-6;

// A cap on the steps of one sweep, so a tiny or bad SweepStep cannot loop
// for ever. The default step needs 20 for the limiter's horizon.
constexpr std::int64_t kMaxSteps = 100000;

constexpr double kForever = std::numeric_limits<double>::infinity();

/** The times when |p - v t| <= half on one axis, as [lo, hi]; lo > hi if never. */
void axisWindow(double p, double v, double half, double &lo, double &hi) noexcept {
  if (std::abs(v) < kStill) {
    const bool inside = std::abs(p) <= half;
    lo = inside ? -kForever : kForever;
    hi = inside ? kForever : -kForever;
    return;
  }
  const double t1 = (p - half) / v;
  const double t2 = (p + half) / v;
  lo = std::min(t1, t2);
  hi = std::max(t1, t2);
}

/** Pure translation at robot-frame velocity v, over [0, maxT]: design §7.2. */
std::optional<double> translationContact(const Outline &outline, Vec2 p, Vec2 v,
                                         double maxT) noexcept {
  double xLo = 0.0;
  double xHi = 0.0;
  double yLo = 0.0;
  double yHi = 0.0;
  axisWindow(p.x, v.x, outline.halfLength, xLo, xHi);
  axisWindow(p.y, v.y, outline.halfWidth, yLo, yHi);
  const double lo = std::max({xLo, yLo, 0.0});
  const double hi = std::min({xHi, yHi, maxT});
  if (lo > hi) {
    return std::nullopt;
  }
  return lo;
}

}  // namespace

Outline grown(const Outline &outline, double marginM) noexcept {
  return Outline{outline.halfLength + marginM, outline.halfWidth + marginM};
}

bool contains(const Outline &outline, Vec2 p) noexcept {
  return std::abs(p.x) <= outline.halfLength && std::abs(p.y) <= outline.halfWidth;
}

std::optional<double> firstContact(const Outline &outline, Vec2 point, const Twist &twist,
                                   double horizonS, const SweepStep &step) {
  return Sweep(outline, twist, horizonS, step).contact(point);
}

Sweep::Sweep(const Outline &outline, const Twist &twist, double horizonS, const SweepStep &step)
    : m_outline(outline), m_velocity{twist.vx, twist.vy}, m_horizonS(horizonS) {
  if (!(horizonS > 0.0) || twist.wz == 0.0) {
    return;
  }
  // Each step at most maxStepM along the path and maxStepRad of turn.
  double dt = step.maxStepRad / std::abs(twist.wz);
  const double speed = norm(m_velocity);
  if (speed > kStill) {
    dt = std::min(dt, step.maxStepM / speed);
  }
  if (!(dt > 0.0)) {
    m_horizonS = 0.0;  // a bad step: nothing can be swept
    return;
  }
  // Capped as a double, before the cast: a huge quotient would overflow it.
  const double wanted = std::min(static_cast<double>(kMaxSteps), std::ceil(horizonS / dt));
  const auto steps = static_cast<std::int64_t>(wanted);
  m_dt = horizonS / static_cast<double>(steps);
  m_turning = true;
  m_steps.reserve(static_cast<std::size_t>(steps));
  for (std::int64_t k = 0; k < steps; ++k) {
    const Pose2 at = integrate(Pose2{}, twist, m_dt * static_cast<double>(k));
    m_steps.push_back(StepPose{at.x, at.y, std::cos(at.heading), std::sin(at.heading)});
  }
}

std::optional<double> Sweep::contact(Vec2 point, double withinS) const noexcept {
  if (contains(m_outline, point)) {
    return 0.0;
  }
  const double limit = std::min(withinS, m_horizonS);
  if (!(limit > 0.0)) {
    return std::nullopt;
  }
  if (!m_turning) {
    return translationContact(m_outline, point, m_velocity, limit);
  }
  for (std::size_t k = 0; k < m_steps.size(); ++k) {
    const double t0 = m_dt * static_cast<double>(k);
    if (t0 > limit) {
      break;
    }
    // Where the point is, seen from where the robot is at the step's start:
    // toChild(), with the step's cosine and sine already known.
    const StepPose &s = m_steps[k];
    const double dx = point.x - s.x;
    const double dy = point.y - s.y;
    const Vec2 seen{s.cosH * dx + s.sinH * dy, -s.sinH * dx + s.cosH * dy};
    const double window = std::min(m_dt, limit - t0);
    if (const auto hit = translationContact(m_outline, seen, m_velocity, window)) {
      return t0 + *hit;
    }
  }
  return std::nullopt;
}

}  // namespace Heron::Geometry
