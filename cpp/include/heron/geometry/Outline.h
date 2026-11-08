// File        : Outline.h
// Author      : Chris Pane
// Description : The robot's rectangle, and when it will touch a point as the
//               robot moves.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The limiter and the planner ask one question of every LiDAR point: if
// Heron keeps moving like this, does its body hit the point, and when?
// (docs/navigation-design.md §7.2.) The body is a rectangle, the outline,
// centred on the robot frame's origin with its sides along the axes:
// [geometry] length and width from the rig's profile, halved.
//
// The answer is firstContact(). Work in the robot frame at time 0, where
// the point is fixed in the world and the robot moves:
//
//   - Pure translation. The point slides toward the robot at -v, and the
//     outline holds it while |px - vx t| <= a and |py - vy t| <= b. Each
//     condition is an interval of t; contact starts where both overlap.
//     Exact, a few operations a point.
//   - With rotation. The motion is cut into steps of at most 2 cm and 2
//     degrees (SweepStep); each step is tested as a translation from where
//     the robot is at its start, so a corner passing over the point in the
//     middle of a step is still caught.
//
// The outline is centred because the profile gives only a length and a
// width. A robot whose LiDAR sticks out at one end gets an outline long
// enough to cover it (docs/navigation-cpp-design.md §2.2).
//
// Pure: no I/O, no time, no threads. Tested by cpp/tests/GeometryTest.cpp.
//
// Usage
// -----
//   using namespace Heron::Geometry;
//   const Outline body{0.12, 0.10};                     // 0.24 x 0.20 m
//   const Twist ahead{0.25, 0.0, 0.0};                  // 0.25 m/s forward
//   firstContact(body, Vec2{0.426, 0.0}, ahead, 3.0);   // 1.224 s
//   firstContact(body, Vec2{0.426, 0.5}, ahead, 3.0);   // no value: it passes
//   firstContact(grown(body, 0.05), ...);               // with a margin

#ifndef HERON_GEOMETRY_OUTLINE_H
#define HERON_GEOMETRY_OUTLINE_H

#include <optional>

#include "heron/geometry/Pose2.h"

namespace Heron::Geometry {

/** The robot's footprint: a rectangle centred on the robot frame's origin,
 *  its sides along the axes. [geometry] length and width, halved. */
struct Outline {
  double halfLength{0.0};  // along x
  double halfWidth{0.0};   // along y
};

/** @return The outline with marginM added all round */
[[nodiscard]] Outline grown(const Outline &outline, double marginM) noexcept;

/** @return true if p, in the robot frame, is inside or on the edge */
[[nodiscard]] bool contains(const Outline &outline, Vec2 p) noexcept;

/** How finely a motion with rotation is stepped (design §7.2). Both must
 *  be above zero. */
struct SweepStep {
  double maxStepM{0.02};
  double maxStepRad{0.0349};  // 2 degrees
};

/**
 * The first time the outline touches a fixed point while the robot moves
 * at a constant twist, the point given in the robot frame at time 0.
 *
 * Pure translation is solved exactly (the overlap of two intervals);
 * with rotation the motion is stepped by `step`, and each step solved as
 * a translation.
 *
 * @return Seconds until contact: 0 if the point is already inside, no
 *         value if it stays clear for horizonS
 */
[[nodiscard]] std::optional<double> firstContact(const Outline &outline, Vec2 point,
                                                 const Twist &twist, double horizonS,
                                                 const SweepStep &step = {}) noexcept;

}  // namespace Heron::Geometry

#endif  // HERON_GEOMETRY_OUTLINE_H
