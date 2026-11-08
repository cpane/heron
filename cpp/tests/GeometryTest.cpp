// File        : GeometryTest.cpp
// Author      : Chris Pane
// Description : Frames, poses, twists and the sweep: heron_geometry.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The worked example of docs/navigation-cpp-design.md §2.3 (the box of
// design §3.3 and §3.4) first, then each function against numbers worked
// by hand, then the stepped sweep against a brute-force search with steps
// a hundred times finer. Same style as the other tests: no framework.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <optional>

#include "heron/geometry/Outline.h"
#include "heron/geometry/Pose2.h"

namespace {

using namespace Heron::Geometry;

constexpr double kPi = std::numbers::pi;

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

bool near(Vec2 a, Vec2 b, double tolerance = 1e-9) {
  return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance);
}

// Headings compared the short way round, so -pi and +pi agree.
bool near(const Pose2 &a, const Pose2 &b, double tolerance = 1e-9) {
  return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
         std::abs(wrapAngle(a.heading - b.heading)) <= tolerance;
}

bool isNear(std::optional<double> t, double expected, double tolerance = 1e-9) {
  return t.has_value() && near(*t, expected, tolerance);
}

// The sweep by brute force: the robot's pose at every fine step, and the
// first step at which the point is inside the outline.
std::optional<double> bruteContact(const Outline &outline, Vec2 point, const Twist &twist,
                                   double horizonS, double dtS) {
  const auto steps = static_cast<std::int64_t>(std::ceil(horizonS / dtS));
  for (std::int64_t k = 0; k <= steps; ++k) {
    const double t = std::min(horizonS, dtS * static_cast<double>(k));
    if (contains(outline, toChild(integrate(Pose2{}, twist, t), point))) {
      return t;
    }
  }
  return std::nullopt;
}

}  // namespace

int main() {
  std::printf("The worked example (design §3.3, §3.4)\n");
  {
    const Vec2 lidarOffset{0.06, 0.0};
    const Vec2 inRobot = polarToRobot(30.0 * kDegToRad, 1.0, lidarOffset);
    check("seen at 30 deg, 1.00 m: (0.926, 0.500) in the robot frame",
          near(inRobot, Vec2{0.06 + std::sqrt(3.0) / 2.0, 0.5}));
    const Pose2 heron{2.0, 1.0, 90.0 * kDegToRad};
    const Vec2 onFloor = toParent(heron, inRobot);
    check("on the floor: (1.500, 1.926) in the odom frame",
          near(onFloor, Vec2{1.5, 1.0 + 0.06 + std::sqrt(3.0) / 2.0}));
    const Pose2 later = integrate(heron, Twist{0.25, 0.0, 0.0}, 2.0);
    check("0.5 m ahead: Heron at (2.0, 1.5, 90 deg)", near(later, Pose2{2.0, 1.5, kPi / 2.0}));
    const Vec2 nowSeen = toChild(later, onFloor);
    check("the box now: (0.426, 0.500) in the robot frame",
          near(nowSeen, Vec2{0.06 + std::sqrt(3.0) / 2.0 - 0.5, 0.5}));
    const Pose2 turned{2.0, 1.5, kPi};
    check("after turning left: (0.500, -0.426)",
          near(toChild(turned, onFloor), Vec2{0.5, -nowSeen.x}));

    const Outline body{0.12, 0.10};
    const Twist ahead{0.25, 0.0, 0.0};
    check("0.5 m to the left, driving on: never touched",
          !firstContact(body, nowSeen, ahead, 3.0).has_value());
    check("straight ahead at 0.426 m: met at 1.224 s",
          isNear(firstContact(body, Vec2{nowSeen.x, 0.0}, ahead, 3.0),
                 (nowSeen.x - 0.12) / 0.25));
  }

  std::printf("Angles\n");
  {
    check("wrap +pi is -pi (half-open)", wrapAngle(kPi) == -kPi);
    check("wrap -pi stays -pi", wrapAngle(-kPi) == -kPi);
    check("wrap 0 stays 0", wrapAngle(0.0) == 0.0);
    check("wrap 3pi/2 is -pi/2", near(wrapAngle(1.5 * kPi), -0.5 * kPi));
    check("wrap -3pi/2 is +pi/2", near(wrapAngle(-1.5 * kPi), 0.5 * kPi));
    check("wrap 7pi is -pi", near(std::abs(wrapAngle(7.0 * kPi)), kPi));
    check("wrap 100.5 turns is half a turn", near(std::abs(wrapAngle(201.0 * kPi)), kPi, 1e-6));
    check("wrap stays in [-pi, +pi) over a sweep", [] {
      for (double a = -20.0; a <= 20.0; a += 0.001) {
        const double w = wrapAngle(a);
        if (w < -kPi || w >= kPi || std::abs(std::sin(w) - std::sin(a)) > 1e-9 ||
            std::abs(std::cos(w) - std::cos(a)) > 1e-9) {
          return false;
        }
      }
      return true;
    }());
    check("rotate (1, 0) by 90 deg is (0, 1)", near(rotate(Vec2{1.0, 0.0}, kPi / 2.0), Vec2{0.0, 1.0}));
    check("norm (3, 4) is 5", near(norm(Vec2{3.0, 4.0}), 5.0));
  }

  std::printf("Frames\n");
  {
    const Pose2 a{1.0, 2.0, 0.7};
    const Pose2 b{-0.3, 0.4, -2.9};
    const Vec2 p{0.25, -1.5};
    check("toChild undoes toParent", near(toChild(a, toParent(a, p)), p));
    check("toParent undoes toChild", near(toParent(a, toChild(a, p)), p));
    check("a then its inverse is no move", near(compose(a, inverse(a)), Pose2{}));
    check("the inverse then a is no move", near(compose(inverse(a), a), Pose2{}));
    check("relative(a, compose(a, b)) is b", near(relative(a, compose(a, b)), b));
    check("compose moves points as the two moves in turn",
          near(toParent(compose(a, b), p), toParent(a, toParent(b, p))));
    check("compose wraps the heading: 3 + 0.5 rad is 3.5 - 2pi",
          near(compose(Pose2{0.0, 0.0, 3.0}, Pose2{0.0, 0.0, 0.5}).heading, 3.5 - 2.0 * kPi));
    // The LiDAR 6 cm ahead on a robot at (2, 1) facing +y is at (2, 1.06).
    check("compose: a mount ahead on a robot facing +y is up the y axis",
          near(compose(Pose2{2.0, 1.0, kPi / 2.0}, Pose2{0.06, 0.0, 0.0}),
               Pose2{2.0, 1.06, kPi / 2.0}));
    check("polarToRobot: straight behind, 0.5 m, sensor at the origin",
          near(polarToRobot(-kPi, 0.5, Vec2{}), Vec2{-0.5, 0.0}));
  }

  std::printf("Integrating a twist\n");
  {
    check("no time, no move", near(integrate(Pose2{1.0, 2.0, 0.3}, Twist{1.0, 1.0, 1.0}, 0.0),
                                   Pose2{1.0, 2.0, 0.3}));
    check("forward 1 m/s for 1 s facing +y: up y",
          near(integrate(Pose2{1.0, 2.0, kPi / 2.0}, Twist{1.0, 0.0, 0.0}, 1.0),
               Pose2{1.0, 3.0, kPi / 2.0}));
    check("strafing left facing +x: up y",
          near(integrate(Pose2{}, Twist{0.0, 0.5, 0.0}, 2.0), Pose2{0.0, 1.0, 0.0}));
    check("spinning on the spot: turns only",
          near(integrate(Pose2{}, Twist{0.0, 0.0, 1.0}, 1.0), Pose2{0.0, 0.0, 1.0}));
    // Forward 0.5 m/s turning pi/4 rad/s for 2 s: a quarter circle of
    // radius r = v / wz to the left, ending at (r, r) facing +y.
    const double r = 0.5 / (kPi / 4.0);
    check("forward and turning left: a quarter circle to (r, r, 90 deg)",
          near(integrate(Pose2{}, Twist{0.5, 0.0, kPi / 4.0}, 2.0), Pose2{r, r, kPi / 2.0}));
    check("strafing left and turning left: a quarter circle to (-r, r, 90 deg)",
          near(integrate(Pose2{}, Twist{0.0, 0.5, kPi / 4.0}, 2.0), Pose2{-r, r, kPi / 2.0}));
    check("forward and turning right: a quarter circle to (r, -r, -90 deg)",
          near(integrate(Pose2{}, Twist{0.5, 0.0, -kPi / 4.0}, 2.0),
               Pose2{r, -r, -kPi / 2.0}));
    check("a full circle comes back to the start",
          near(integrate(Pose2{0.5, -0.5, 1.0}, Twist{1.0, 0.3, 1.0}, 2.0 * kPi),
               Pose2{0.5, -0.5, 1.0}));
    check("an arc in one call equals the same arc in a thousand", [] {
      const Twist twist{0.3, -0.1, 0.8};
      Pose2 stepped{0.2, 0.1, -0.4};
      for (std::int32_t i = 0; i < 1000; ++i) {
        stepped = integrate(stepped, twist, 0.003);
      }
      return near(stepped, integrate(Pose2{0.2, 0.1, -0.4}, twist, 3.0), 1e-9);
    }());
    check("a tiny turn rate gives the straight line",
          near(integrate(Pose2{}, Twist{1.0, 0.0, 1e-12}, 1.0), Pose2{1.0, 0.0, 0.0}, 1e-9));
  }

  std::printf("The outline\n");
  {
    const Outline body{0.12, 0.10};
    check("the centre is inside", contains(body, Vec2{}));
    check("a corner is inside (edges included)", contains(body, Vec2{0.12, -0.10}));
    check("just past the front is outside", !contains(body, Vec2{0.1201, 0.0}));
    check("just past the side is outside", !contains(body, Vec2{0.0, -0.1001}));
    const Outline big = grown(body, 0.05);
    check("grown by 5 cm all round", near(big.halfLength, 0.17) && near(big.halfWidth, 0.15));
  }

  std::printf("The sweep, translation (exact)\n");
  {
    const Outline body{0.12, 0.10};
    check("already inside: 0", isNear(firstContact(body, Vec2{0.05, 0.05}, Twist{}, 1.0), 0.0));
    check("already inside, even with no horizon",
          isNear(firstContact(body, Vec2{0.05, 0.05}, Twist{}, 0.0), 0.0));
    check("standing still, outside: never",
          !firstContact(body, Vec2{0.3, 0.0}, Twist{}, 10.0).has_value());
    check("ahead, driving forward: front edge meets it at (0.5 - 0.12) / 0.25",
          isNear(firstContact(body, Vec2{0.5, 0.0}, Twist{0.25, 0.0, 0.0}, 3.0), 1.52));
    check("ahead, but the horizon ends first: never",
          !firstContact(body, Vec2{0.5, 0.0}, Twist{0.25, 0.0, 0.0}, 1.5).has_value());
    check("ahead, horizon exactly at contact: met",
          isNear(firstContact(body, Vec2{0.5, 0.0}, Twist{0.25, 0.0, 0.0}, 1.52), 1.52));
    check("ahead, driving backward: never",
          !firstContact(body, Vec2{0.5, 0.0}, Twist{-0.25, 0.0, 0.0}, 10.0).has_value());
    check("behind, driving backward: rear edge meets it",
          isNear(firstContact(body, Vec2{-0.5, 0.0}, Twist{-0.25, 0.0, 0.0}, 3.0), 1.52));
    check("to the left, strafing left: side meets it at (0.5 - 0.10) / 0.25",
          isNear(firstContact(body, Vec2{0.0, 0.5}, Twist{0.0, 0.25, 0.0}, 3.0), 1.6));
    check("to the right, strafing left: never",
          !firstContact(body, Vec2{0.0, -0.5}, Twist{0.0, 0.25, 0.0}, 10.0).has_value());
    // Diagonal: x window 1.9 to 3.1 s, y window 2.0 to 3.0 s; overlap from 2.0.
    check("diagonal toward a point ahead-left: the later window opens at 2.0 s",
          isNear(firstContact(body, Vec2{0.5, 0.5}, Twist{0.2, 0.2, 0.0}, 5.0), 2.0));
    // Forward only, a point off the corner: the windows never overlap.
    check("ahead but just outside the side: driving forward passes it",
          !firstContact(body, Vec2{0.5, 0.1001}, Twist{0.25, 0.0, 0.0}, 10.0).has_value());
    check("ahead and just on the side line: touched",
          isNear(firstContact(body, Vec2{0.5, 0.10}, Twist{0.25, 0.0, 0.0}, 10.0), 1.52));
  }

  std::printf("The sweep, with rotation (stepped)\n");
  {
    const Outline body{0.12, 0.10};
    const SweepStep step;
    // Spinning anticlockwise at 1 rad/s, a point 0.13 m to the left: outside
    // the 0.10 m half-width, inside the corner's 0.156 m reach. In the robot
    // frame it swings clockwise from 90 deg and enters through the left
    // side, where 0.13 sin(phi) = 0.10.
    const double exact = kPi / 2.0 - std::asin(0.10 / 0.13);
    const auto spin = firstContact(body, Vec2{0.0, 0.13}, Twist{0.0, 0.0, 1.0}, 3.0, step);
    check("spinning, a point the corners reach: met within one 2 deg step, never early",
          spin.has_value() && *spin >= exact - 1e-9 && *spin <= exact + step.maxStepRad + 1e-9);
    check("spinning, a point beyond the corners' reach: never",
          !firstContact(body, Vec2{0.0, 0.16}, Twist{0.0, 0.0, 1.0}, 10.0, step).has_value());
    check("spinning, already inside: 0",
          isNear(firstContact(body, Vec2{0.1, 0.0}, Twist{0.0, 0.0, 1.0}, 1.0, step), 0.0));

    // Against brute force, steps of 0.1 ms. Stepping tests each 2 cm or 2 deg
    // as a translation, so it may differ from the true arc by about a step.
    // Each motion is a circle about a centre c, radius v / wz; a point at
    // distance d from c is met if the outline spans d there (worked by hand).
    struct Case {
      const char *what;
      Vec2 point;
      Twist twist;
      bool meets;
    };
    const Case cases[] = {
        // c (0, 0.5), r 0.5; d 0.5: on the path of the centre.
        {"forward, turning left: a point on the arc", Vec2{0.4, 0.2}, Twist{0.25, 0.0, 0.5}, true},
        // The mirror image.
        {"forward, turning right: a point on the arc", Vec2{0.4, -0.2}, Twist{0.25, 0.0, -0.5},
         true},
        // c (-0.5, 0), r 0.5; d 0.5.
        {"strafing left, turning left: a point on the arc", Vec2{-0.2, 0.4},
         Twist{0.0, 0.2, 0.4}, true},
        // c (0, -1/3), r 1/3; d 0.35, inside the outline's 0.23 to 0.45.
        {"backward, turning left: a point on the arc", Vec2{-0.3, -0.15},
         Twist{-0.2, 0.0, 0.6}, true},
        // c (0, 0.5); d 0.63, beyond the outer corners' 0.61: passes outside.
        {"forward, turning left: a point outside the arc", Vec2{0.6, 0.3},
         Twist{0.25, 0.0, 0.5}, false},
        // c (-0.3, 0), r 0.3; d 0.8, beyond 0.42.
        {"strafing left, turning left: a point the other way", Vec2{0.5, 0.0},
         Twist{0.0, 0.3, 1.0}, false},
    };
    for (const Case &c : cases) {
      const auto swept = firstContact(body, c.point, c.twist, 4.0, step);
      const auto brute = bruteContact(body, c.point, c.twist, 4.0, 1e-4);
      const double speed = std::hypot(c.twist.vx, c.twist.vy);
      const double oneStep =
          std::min(step.maxStepRad / std::abs(c.twist.wz), step.maxStepM / speed);
      const bool agree = swept.has_value() == c.meets && brute.has_value() == c.meets &&
                         (!c.meets || near(*swept, *brute, oneStep));
      check(c.what, agree);
      if (!agree) {
        std::printf("        swept %s %.4f, brute %s %.4f\n", swept ? "hit" : "none",
                    swept.value_or(-1.0), brute ? "hit" : "none", brute.value_or(-1.0));
      }
    }
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
