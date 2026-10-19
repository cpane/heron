// File        : Planar.h
// Author      : Chris Pane
// Description : x, y and a heading: a pose on the floor, or its rates.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// One aggregate for every planar triple the odometry library hands out, so
// pose, velocity, acceleration and the mounting offset read alike. The units
// depend on what it holds, and every use names them. Design:
// docs/odometry-cpp-design.md §2.

#ifndef HERON_ODOMETRY_PLANAR_H
#define HERON_ODOMETRY_PLANAR_H

namespace Heron::Odometry {

/**
 * x and y with a heading: m, m, rad for a pose or an offset; m/s, m/s,
 * rad/s for a velocity; m/s^2, m/s^2, rad/s^2 for an acceleration.
 */
struct Planar {
  double x{0.0};
  double y{0.0};
  double h{0.0};
};

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_PLANAR_H
