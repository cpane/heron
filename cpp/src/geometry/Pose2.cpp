// File        : Pose2.cpp
// Author      : Chris Pane
// Description : Points, poses and twists: the frame moves of Pose2.h.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/geometry/Pose2.h"

#include <cmath>
#include <numbers>

namespace Heron::Geometry {

namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Below this turn in one integrate() call, the arc is treated as a straight
// line: sin(t)/t and (1 - cos t)/t lose their digits as t goes to 0, and
// the straight line is then exact to far below a micrometre.
constexpr double kStraightRad = 1e-9;

}  // namespace

Vec2 operator+(Vec2 a, Vec2 b) noexcept {
  return Vec2{a.x + b.x, a.y + b.y};
}

Vec2 operator-(Vec2 a, Vec2 b) noexcept {
  return Vec2{a.x - b.x, a.y - b.y};
}

Vec2 operator*(double k, Vec2 v) noexcept {
  return Vec2{k * v.x, k * v.y};
}

double norm(Vec2 v) noexcept {
  return std::hypot(v.x, v.y);
}

Vec2 rotate(Vec2 v, double angleRad) noexcept {
  const double c = std::cos(angleRad);
  const double s = std::sin(angleRad);
  return Vec2{c * v.x - s * v.y, s * v.x + c * v.y};
}

double wrapAngle(double rad) noexcept {
  // Most angles are in range already, and fmod is slow next to a compare:
  // it was the largest single cost of a limiter call.
  if (rad >= -kPi && rad < kPi) {
    return rad;
  }
  // As wrapBearing(): std::remainder would return +pi for +pi, so fold by
  // hand and send the top end to the bottom.
  double r = std::fmod(rad + kPi, kTwoPi);
  if (r < 0.0) {
    r += kTwoPi;
  }
  r -= kPi;
  return r >= kPi ? -kPi : r;
}

Vec2 toParent(const Pose2 &pose, Vec2 p) noexcept {
  return Vec2{pose.x, pose.y} + rotate(p, pose.heading);
}

Vec2 toChild(const Pose2 &pose, Vec2 p) noexcept {
  return rotate(p - Vec2{pose.x, pose.y}, -pose.heading);
}

Pose2 compose(const Pose2 &a, const Pose2 &b) noexcept {
  const Vec2 origin = toParent(a, Vec2{b.x, b.y});
  return Pose2{origin.x, origin.y, wrapAngle(a.heading + b.heading)};
}

Pose2 inverse(const Pose2 &pose) noexcept {
  const Vec2 origin = rotate(Vec2{-pose.x, -pose.y}, -pose.heading);
  return Pose2{origin.x, origin.y, wrapAngle(-pose.heading)};
}

Pose2 relative(const Pose2 &from, const Pose2 &to) noexcept {
  return compose(inverse(from), to);
}

Pose2 integrate(const Pose2 &start, const Twist &twist, double dtS) noexcept {
  // The motion in the start's own frame, then composed onto the start. A
  // constant body twist turning at wz traces a circle; integrating the
  // body velocity, rotated by the heading reached at each instant, gives
  //   dx = (vx sin t - vy (1 - cos t)) / wz
  //   dy = (vx (1 - cos t) + vy sin t) / wz       with t = wz dtS.
  const double turn = twist.wz * dtS;
  Vec2 moved{twist.vx * dtS, twist.vy * dtS};
  if (std::abs(turn) >= kStraightRad) {
    const double s = std::sin(turn) / twist.wz;
    const double c = (1.0 - std::cos(turn)) / twist.wz;
    moved = Vec2{twist.vx * s - twist.vy * c, twist.vx * c + twist.vy * s};
  }
  return compose(start, Pose2{moved.x, moved.y, wrapAngle(turn)});
}

Vec2 polarToRobot(double bearingRad, double rangeM, Vec2 sensorOffset) noexcept {
  return sensorOffset + Vec2{rangeM * std::cos(bearingRad), rangeM * std::sin(bearingRad)};
}

}  // namespace Heron::Geometry
