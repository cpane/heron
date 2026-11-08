// File        : Pose2.h
// Author      : Chris Pane
// Description : Points, poses and twists on the floor, and the moves between
//               frames: the maths every navigation library shares.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// Navigation works in three frames (docs/navigation-design.md §3): the
// LiDAR's, the robot's (ROS: base_link) and the odom frame (ROS: odom),
// where the OTOS keeps Heron's pose since start-up. A frame is an origin
// and two axes; moving a point between two frames is a shift plus a
// rotation. This header is those moves, once, so the pose, perception,
// limiter, planner and simulator libraries cannot each get a sign wrong
// in their own way.
//
//   Vec2    a point or a direction in some frame, m
//   Pose2   a child frame in a parent frame: where its origin is and which
//           way its x axis points. Heron's pose is the robot frame in the
//           odom frame
//   Twist   how fast, in the robot frame: m/s forward, m/s left, rad/s
//           anticlockwise
//
// The 2 means planar: x, y and one heading (ROS Pose2D, GTSAM Pose2). It
// also keeps the type apart from the Heron::Pose namespace of heron_pose.
// Every length is in metres and every angle in radians, x forward, y left,
// angles anticlockwise (REP 103). Pose2 and Twist are both three numbers,
// so they are separate types and the compiler stops one being passed for
// the other (docs/navigation-cpp-design.md, I4).
//
// Pure: no I/O, no time, no threads. Tested by cpp/tests/GeometryTest.cpp.
//
// Usage
// -----
// The box of design §3.3 and §3.4, seen at 30 degrees and 1.00 m by a
// LiDAR mounted 6 cm ahead of the robot's centre, with Heron at (2, 1)
// facing +y:
//
//   using namespace Heron::Geometry;
//   const Vec2 inRobot = polarToRobot(30.0 * kDegToRad, 1.0, Vec2{0.06, 0.0});
//                                                         // (0.926, 0.500)
//   const Pose2 heron{2.0, 1.0, 90.0 * kDegToRad};
//   const Vec2 onFloor = toParent(heron, inRobot);         // (1.500, 1.926)
//
//   // Half a metre ahead later, where is the box now, seen from Heron?
//   const Pose2 later = integrate(heron, Twist{0.25, 0.0, 0.0}, 2.0);
//   const Vec2 nowSeen = toChild(later, onFloor);          // (0.426, 0.500)

#ifndef HERON_GEOMETRY_POSE2_H
#define HERON_GEOMETRY_POSE2_H

namespace Heron::Geometry {

/** A point or a direction in some frame, metres. */
struct Vec2 {
  double x{0.0};
  double y{0.0};
};

[[nodiscard]] Vec2 operator+(Vec2 a, Vec2 b) noexcept;
[[nodiscard]] Vec2 operator-(Vec2 a, Vec2 b) noexcept;
[[nodiscard]] Vec2 operator*(double k, Vec2 v) noexcept;
/** @return The length of v */
[[nodiscard]] double norm(Vec2 v) noexcept;
/** @return v turned anticlockwise by angleRad */
[[nodiscard]] Vec2 rotate(Vec2 v, double angleRad) noexcept;

inline constexpr double kDegToRad = 0.017453292519943295;

/**
 * Folds an angle into [-pi, +pi): wrapBearing()'s double twin (Scan.h,
 * decision 1). The half-open end matters: +pi comes back as -pi.
 */
[[nodiscard]] double wrapAngle(double rad) noexcept;

/**
 * The pose of a child frame in a parent frame: where its origin is and
 * which way its x axis points. m, m, rad. Heron's pose is the robot frame
 * in the odom frame (design §3.1).
 */
struct Pose2 {
  double x{0.0};
  double y{0.0};
  double heading{0.0};  // wrapped to [-pi, +pi)
};

/**
 * A child-frame point, in the parent frame: design §3.3, step 2.
 * Rotate by the heading, then shift by the origin.
 */
[[nodiscard]] Vec2 toParent(const Pose2 &pose, Vec2 p) noexcept;

/**
 * A parent-frame point, in the child frame: design §3.4's reverse step.
 * Shift back by the origin, then rotate back by the heading.
 */
[[nodiscard]] Vec2 toChild(const Pose2 &pose, Vec2 p) noexcept;

/**
 * b given in a's frame, expressed in a's parent: if a is the robot in the
 * odom frame and b the LiDAR on the robot, compose(a, b) is the LiDAR in
 * the odom frame.
 */
[[nodiscard]] Pose2 compose(const Pose2 &a, const Pose2 &b) noexcept;

/** The parent frame, seen from the child: compose(pose, inverse(pose)) is zero. */
[[nodiscard]] Pose2 inverse(const Pose2 &pose) noexcept;

/** Where `to` is, seen from `from`: compose(inverse(from), to). */
[[nodiscard]] Pose2 relative(const Pose2 &from, const Pose2 &to) noexcept;

/** How fast, in the robot frame: m/s forward, m/s left, rad/s anticlockwise. */
struct Twist {
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/**
 * Where a constant twist, held for dtS seconds, takes a pose: the exact
 * arc, not a straight-line step. The twist is in the robot frame, so a
 * robot driving forward while turning follows a circle.
 */
[[nodiscard]] Pose2 integrate(const Pose2 &start, const Twist &twist, double dtS) noexcept;

/**
 * A LiDAR reading in the robot frame: design §3.3, step 1. The bearing is
 * already in the robot frame's directions (Scan's points are); the sensor
 * sits at sensorOffset from the robot frame's origin.
 *
 * Takes plain numbers, not a Heron::Point, so geometry does not depend on
 * the LiDAR's types.
 */
[[nodiscard]] Vec2 polarToRobot(double bearingRad, double rangeM, Vec2 sensorOffset) noexcept;

}  // namespace Heron::Geometry

#endif  // HERON_GEOMETRY_POSE2_H
