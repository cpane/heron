// File        : MotionTest.cpp
// Author      : Chris Pane
// Description : heron_motion: the table's columns, interpolation, mirror
//               and inverse lookups, against tables made by hand.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The motion_test row of docs/navigation-cpp-design.md §7.5: columns from
// points within 10 degrees, across +-180 included; linear in size within a
// column, the smallest below it and the largest above; linear in angle
// between columns; the mirror; refused tables; commandFor() never faster
// than asked; nearestCommand() never past what it is asked for on either
// axis. Last, the bench rig's hardwood points (design §19.2's example),
// read back. Same style as the other tests: no framework.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "heron/motion/MotionTable.h"

namespace {

using Heron::Geometry::kDegToRad;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Motion::MotionPoint;
using Heron::Motion::MotionTable;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b, double tolerance = 1e-9) {
  return std::abs(a - b) <= tolerance;
}

bool same(const Twist &a, const Twist &b, double tolerance = 1e-9) {
  return near(a.vx, b.vx, tolerance) && near(a.vy, b.vy, tolerance) && near(a.wz, b.wz, tolerance);
}

MotionPoint point(double directionDeg, double size, double vx, double vy, double wz = 0.0) {
  return MotionPoint{directionDeg * kDegToRad, size, Twist{vx, vy, wz}};
}

// Forward and right, each at 0.5 and 1.0: forward 0.2 then 0.4 m/s; right
// 0.1 then 0.3 m/s, turning -0.2 rad/s at both.
std::vector<MotionPoint> forwardAndRight() {
  return {point(0, 0.5, 0.2, 0.0), point(0, 1.0, 0.4, 0.0), point(-90, 0.5, 0.0, -0.1, -0.2),
          point(-90, 1.0, 0.0, -0.3, -0.2)};
}

// The bench rig's hardwood, 2026-11-14 (design §19.2), not mirrored.
std::vector<MotionPoint> hardwood() {
  return {point(0, 0.2, 0.19, 0, 0.04),      point(0, 0.4, 0.27, 0, 0.03),
          point(0, 0.6, 0.31, 0, 0.05),      point(0, 0.8, 0.34, 0, 0.02),
          point(0, 1.0, 0.39, 0, 0),         point(-15, 0.2, 0.20, 0, 0.03),
          point(-15, 0.4, 0.28, 0, -0.03),   point(-15, 0.6, 0.33, -0.01, -0.02),
          point(-15, 0.8, 0.35, -0.02, -0.05), point(-15, 1.0, 0.38, 0, 0.02),
          point(-45, 0.3, 0.01, 0, 0),       point(-45, 0.55, 0.12, -0.10, 0.01),
          point(-45, 0.85, 0.15, -0.11, -0.09), point(-45, 1.0, 0.14, -0.14, -0.07),
          point(-85, 0.2, 0, 0, 0),          point(-75, 0.4, 0.05, -0.06, -0.18),
          point(-80, 0.6, 0.04, -0.15, -0.20), point(-90, 0.8, 0.01, -0.21, -0.13),
          point(-90, 1.0, -0.02, -0.27, 0.05), point(90, 0.6, 0.01, 0.04, 0.07),
          point(90, 1.0, -0.02, 0.25, 0),    point(180, 0.6, -0.31, 0, -0.08)};
}

}  // namespace

int main() {
  std::printf("Columns (I18)\n");
  {
    const MotionTable plain(forwardAndRight(), false);
    check("forward and right: 2 columns", plain.columns() == 2);
    const MotionTable spread({point(0, 1.0, 0.4, 0), point(-85, 0.5, 0, -0.1), point(-80, 0.6, 0, -0.15),
                              point(-90, 1.0, 0, -0.3)},
                             false);
    check("right at -80, -85, -90: one column", spread.columns() == 2);
    const MotionTable apart({point(0, 1.0, 0.4, 0), point(-15, 1.0, 0.38, 0), point(-90, 1.0, 0, -0.3)}, false);
    check("0 and -15: two columns", apart.columns() == 3);
    const MotionTable wrap({point(0, 1.0, 0.4, 0), point(178, 0.6, -0.3, 0), point(-178, 1.0, -0.4, 0)}, false);
    check("178 and -178: one column, across +-180", wrap.columns() == 2);
    bool threw = false;
    try {
      MotionTable({point(0, 0.5, 0.2, 0), point(5, 1.0, 0.4, 0)}, false);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check("one column only: refused", threw);
    threw = false;
    try {
      MotionTable({point(0, 0.0, 0.2, 0), point(-90, 1.0, 0, -0.3)}, false);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check("a size of 0: refused", threw);
  }

  std::printf("predict()\n");
  {
    const MotionTable t(forwardAndRight(), false);
    check("zero: zero", same(t.predict(0.0, 0.0), Twist{}));
    check("on a point: the point", same(t.predict(1.0, 0.0), Twist{0.4, 0.0, 0.0}));
    check("between sizes: linear (0.75 forward, 0.3 m/s)", same(t.predict(0.75, 0.0), Twist{0.3, 0.0, 0.0}));
    check("below the smallest: the smallest (the dead band)", same(t.predict(0.1, 0.0), Twist{0.2, 0.0, 0.0}));
    check("above the largest: the largest", same(t.predict(1.4, 0.0), Twist{0.4, 0.0, 0.0}));
    const double s = 1.0 / std::sqrt(2.0);
    check("45 deg right at size 1: halfway between the columns",
          same(t.predict(s, -s), Twist{0.2, -0.15, -0.1}, 1e-9));
    check("right at 0.5: its point, the turn kept", same(t.predict(0.0, -0.5), Twist{0.0, -0.1, -0.2}));
    // With only forward and right, left lies in the 270 deg gap from
    // forward round to right, a third of the way along it.
    const Twist left = t.predict(0.0, 1.0);
    check("left with no left column: between right and forward, round the back",
          near(left.vx, 0.4 * 2.0 / 3.0, 1e-9) && near(left.vy, -0.3 / 3.0, 1e-9));
  }

  std::printf("The mirror\n");
  {
    const MotionTable m(forwardAndRight(), true);
    check("forward and right, mirrored: left added, 3 columns", m.columns() == 3);
    check("left at 1.0: right's, vy and wz negated", same(m.predict(0.0, 1.0), Twist{0.0, 0.3, 0.2}));
    check("forward is on the axis: not doubled", same(m.predict(1.0, 0.0), Twist{0.4, 0.0, 0.0}));
    std::vector<MotionPoint> withLeft = forwardAndRight();
    withLeft.push_back(point(90, 1.0, 0.0, 0.05));
    const MotionTable own(withLeft, true);
    check("a measured left column is kept, not mirrored over", own.columns() == 3 &&
                                                                 same(own.predict(0.0, 1.0), Twist{0.0, 0.05, 0.0}));
  }

  std::printf("commandFor() and leastSpeed()\n");
  {
    const MotionTable t(forwardAndRight(), false);
    const auto c = t.commandFor(0.0, 0.3);
    check("forward at 0.3 m/s: size 0.75 (to the 0.005 step)", c && near(c->first, 0.75, 0.005) &&
                                                                    near(c->second, 0.0));
    const Twist made = t.predict(c->first, c->second);
    check("  and never faster than asked", made.vx <= 0.3 + 1e-9);
    check("forward at 0.1 m/s, under the least (0.2): none", !t.commandFor(0.0, 0.1));
    check("forward at 1 m/s, over the most: the largest size", t.commandFor(0.0, 1.0) &&
                                                                    near(t.commandFor(0.0, 1.0)->first, 1.0));
    check("least speed forward 0.2, right 0.1", near(t.leastSpeed(0.0), 0.2) &&
                                                    near(t.leastSpeed(-90.0 * kDegToRad), 0.1));
  }

  std::printf("nearestCommand()\n");
  {
    const MotionTable t(forwardAndRight(), false);
    const auto c = t.nearestCommand(Vec2{0.0, -0.2});
    const Twist made = c ? t.predict(c->first, c->second) : Twist{};
    std::printf("        want right 0.2: command (%.3f, %.3f) makes (%.3f, %.3f)\n", c ? c->first : 0.0,
                c ? c->second : 0.0, made.vx, made.vy);
    check("want right 0.2: a strafe that makes it, nothing forward", c && near(made.vy, -0.2, 0.011) &&
                                                                        std::abs(made.vx) <= 0.005);
    check("want nothing: none (stop)", !t.nearestCommand(Vec2{0.0, 0.0}));
    check("want 0.05 forward, under the least (0.2): none", !t.nearestCommand(Vec2{0.05, 0.0}));

    const MotionTable hw(hardwood(), false);
    check("hardwood: pure right is not on offer (its strafe creeps forward 0.04)",
          !hw.nearestCommand(Vec2{0.0, -0.15}) ||
              std::abs(hw.predict(hw.nearestCommand(Vec2{0.0, -0.15})->first,
                                  hw.nearestCommand(Vec2{0.0, -0.15})->second).vx) <= 0.005);
    const auto fr = hw.nearestCommand(Vec2{0.2, -0.15});
    const Twist frMade = fr ? hw.predict(fr->first, fr->second) : Twist{};
    std::printf("        hardwood, want (0.20, -0.15): command (%.3f, %.3f) makes (%.3f, %.3f)\n",
                fr ? fr->first : 0.0, fr ? fr->second : 0.0, frMade.vx, frMade.vy);
    check("hardwood, want forward-right: within it on both axes", fr && frMade.vx <= 0.205 &&
                                                                     frMade.vx >= -0.005 && frMade.vy >= -0.155 &&
                                                                     frMade.vy <= 0.005);
  }

  std::printf("The bench rig's hardwood (design §19.2)\n");
  {
    const MotionTable hw(hardwood(), false);
    std::printf("        %zu columns\n", hw.columns());
    check("6 columns: 0, -15, -45, right, left, back", hw.columns() == 6);
    check("forward at 0.6: 0.31 m/s", same(hw.predict(0.6, 0.0), Twist{0.31, 0.0, 0.05}));
    check("forward at 0.1: 0.19, the floor", near(hw.predict(0.1, 0.0).vx, 0.19));
    const Twist right = hw.predict(0.0, -0.6);
    std::printf("        strafe right 0.6: (%.3f, %.3f, %.3f)\n", right.vx, right.vy, right.wz);
    check("strafe right at 0.6: about 0.15 right, turning right", right.vy < -0.12 && right.vy > -0.18 &&
                                                                     right.wz < -0.1);
    check("strafe right at 0.2: nothing", std::hypot(hw.predict(0.0, -0.2).vx, hw.predict(0.0, -0.2).vy) < 0.02);
    check("left at 0.6: 0.04, not right's mirror", near(hw.predict(0.0, 0.6).vy, 0.04, 1e-9));
  }

  std::printf("\n%s: %d failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
