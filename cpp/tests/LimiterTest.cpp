// File        : LimiterTest.cpp
// Author      : Chris Pane
// Description : heron_limiter: each rule of design §7.3 and row of §7.4,
//               against walls and boxes placed by hand.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The N4 row of docs/navigation-cpp-design.md §9 and design §12.1: each
// rule of §7.3 (clear, cut by axis, slow zone, already too close) and each
// row of §7.4 (no scan, stale, not covered, unobserved, pose not fresh, the
// blind spot); sliding along a wall keeps the slide; backing off is
// allowed. The figures each check expects are worked by hand in its
// comment. Last, the time one call takes, printed. Same style as the other
// tests: no framework.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include "heron/limiter/Limiter.h"

namespace {

using namespace std::chrono_literals;
using namespace Heron::Limiter;
using Heron::Duration;
using Heron::Scan;
using Heron::TimePoint;
using Heron::Geometry::Outline;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Pose::PoseHistory;
using Heron::Pose::PoseSample;

constexpr double kPi = std::numbers::pi;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b, double tolerance) {
  return std::abs(a - b) <= tolerance;
}

bool same(const Twist &a, const Twist &b, double tolerance = 1e-9) {
  return near(a.vx, b.vx, tolerance) && near(a.vy, b.vy, tolerance) &&
         near(a.wz, b.wz, tolerance);
}

// A body 0.30 x 0.24 m with the LiDAR 6 cm ahead: the stop outline is then
// 0.20 x 0.17 m each way, and the blind circle fits inside it.
LimiterConfig testConfig() {
  LimiterConfig config;
  config.outline = Outline{0.15, 0.12};
  config.lidarOffset = Vec2{0.06, 0.0};
  return config;
}

// A point where the robot frame says, as the LiDAR at `lidar` reports it.
Heron::Point seen(Vec2 p, Vec2 lidar = Vec2{0.06, 0.0}) {
  const Vec2 d = p - lidar;
  return Heron::Point{static_cast<float>(std::atan2(d.y, d.x)),
                      static_cast<float>(Heron::Geometry::norm(d))};
}

// A healthy scan of these points, ending at `end`, one 133 ms revolution.
Scan scanOf(const std::vector<Vec2> &where, TimePoint end) {
  Scan scan;
  scan.seq = 1;
  scan.tEnd = end;
  scan.tStart = end - 133ms;
  scan.sweepStartRad = 3.0f;
  scan.rawSampleCount = 520;
  scan.worstSamplingGapRad = 8.0f * static_cast<float>(kPi / 180.0);
  for (const Vec2 &p : where) {
    scan.points.push_back(seen(p));
  }
  std::sort(scan.points.begin(), scan.points.end(),
            [](const Heron::Point &a, const Heron::Point &b) { return a.bearingRad < b.bearingRad; });
  return scan;
}

// A wall of points 1 cm apart, from a to b.
std::vector<Vec2> wall(Vec2 a, Vec2 b) {
  std::vector<Vec2> points;
  const double length = Heron::Geometry::norm(b - a);
  const auto n = static_cast<std::int32_t>(std::round(length / 0.01));
  for (std::int32_t i = 0; i <= n; ++i) {
    points.push_back(a + (static_cast<double>(i) / n) * (b - a));
  }
  return points;
}

// Heron standing at the origin from well before to after t.
PoseHistory still(TimePoint t) {
  PoseHistory history;
  history.add(PoseSample{t - 1s, Pose2{}, Twist{}});
  history.add(PoseSample{t + 10ms, Pose2{}, Twist{}});
  return history;
}

}  // namespace

int main() {
  const Limiter limiter(testConfig());
  const TimePoint t0{100s};
  const PoseHistory history = still(t0);
  const Twist anything{0.3, 0.1, 0.5};

  std::printf("When it cannot see (design §7.4)\n");
  {
    LimitResult r = limiter.limit(LimitInput{anything, nullptr, &history, true, Twist{}, t0});
    check("no scan: stop, NoScan", same(r.command, Twist{}) && r.reason == Reason::NoScan);
    const Scan empty = scanOf({}, t0);
    r = limiter.limit(LimitInput{anything, &empty, &history, true, Twist{}, t0 + 300ms});
    check("a scan 300 ms old: still used", r.reason == Reason::Clear);
    r = limiter.limit(LimitInput{anything, &empty, &history, true, Twist{}, t0 + 301ms});
    check("301 ms old: stop, Stale", same(r.command, Twist{}) && r.reason == Reason::Stale);
    Scan holed = empty;
    holed.worstSamplingGapRad = 0.3f;
    r = limiter.limit(LimitInput{anything, &holed, &history, true, Twist{}, t0});
    check("a 17 deg hole: stop, NotCovered",
          same(r.command, Twist{}) && r.reason == Reason::NotCovered);

    // An arc ahead not looked at (with a healthy worst gap, which the real
    // adapter would not produce: so the rule is tested on its own).
    Scan blind = empty;
    blind.unobserved.push_back(Heron::Arc{-0.3f, 0.6f});
    r = limiter.limit(LimitInput{Twist{0.3, 0.2, 0.5}, &blind, &history, true, Twist{}, t0});
    check("ahead unobserved: forward and turning stop, sideways kept, Unobserved",
          same(r.command, Twist{0.0, 0.2, 0.0}) && r.reason == Reason::Unobserved);
    r = limiter.limit(LimitInput{Twist{-0.3, 0.0, 0.0}, &blind, &history, true, Twist{}, t0});
    check("  backward, away from it: kept", same(r.command, Twist{-0.3, 0.0, 0.0}));
  }

  std::printf("Clear, cut and slowed: a wall ahead at 0.3 m/s (design §7.3)\n");
  {
    // T = 0.3 m/s / 1 m/s^2 + 0 + 0.137 s = 0.437 s. The stop outline's
    // front is at x = 0.20.
    const Twist ahead{0.3, 0.0, 0.0};
    const Scan far = scanOf(wall(Vec2{1.2, -0.5}, Vec2{1.2, 0.5}), t0);
    LimitResult r = limiter.limit(LimitInput{ahead, &far, &history, true, ahead, t0});
    std::printf("        T = %.3f s\n", r.horizonS);
    check("T is the time to stop plus a revolution", near(r.horizonS, 0.437, 1e-9));
    check("at 1.2 m: Clear, unchanged", same(r.command, ahead) && r.reason == Reason::Clear);
    // Every wall point within the body's width is equally near: any one.
    check("  the nearest point reported: on the wall, in front of the body",
          r.nearest && near(r.nearest->x, 1.2, 1e-5) && std::abs(r.nearest->y) <= 0.12 + 1e-5);

    // 0.20 m from the stop outline: contact at 0.667 s, after T; but within
    // the 0.25 m slow zone: 0.20 / 0.25 = 0.8 of the speed.
    const Scan mid = scanOf(wall(Vec2{0.4, -0.5}, Vec2{0.4, 0.5}), t0);
    r = limiter.limit(LimitInput{ahead, &mid, &history, true, ahead, t0});
    std::printf("        at 0.40 m: vx %.4f, %s\n", r.command.vx, toString(r.reason));
    check("at 0.40 m: Slowed to 0.8, 0.24 m/s",
          near(r.command.vx, 0.24, 1e-4) && r.reason == Reason::Slowed);

    // 0.10 m: contact at 0.333 s, before T: cut to 0.333 / 0.437 = 0.763;
    // the slow zone asks 0.10 / 0.25 = 0.4, the lower.
    const Scan close = scanOf(wall(Vec2{0.3, -0.5}, Vec2{0.3, 0.5}), t0);
    r = limiter.limit(LimitInput{ahead, &close, &history, true, ahead, t0});
    std::printf("        at 0.30 m: vx %.4f, %s\n", r.command.vx, toString(r.reason));
    check("at 0.30 m: Cut, to 0.4, 0.12 m/s",
          near(r.command.vx, 0.12, 1e-4) && r.reason == Reason::Cut && near(r.scale.vx, 0.4, 1e-4));

    // Faster, so T is longer: 0.6 / 1 + 0.137 = 0.737 s; contact at 0.333 /
    // 2 = 0.1667 s: cut to 0.226; slow 0.4.
    const Twist fast{0.6, 0.0, 0.0};
    r = limiter.limit(LimitInput{fast, &close, &history, true, fast, t0});
    std::printf("        at 0.30 m, 0.6 m/s: vx %.4f, scale %.4f\n", r.command.vx, r.scale.vx);
    check("at 0.6 m/s: cut to 0.1667 / 0.737 of it, 0.136 m/s",
          near(r.scale.vx, 0.1667 / 0.737, 1e-3) && r.reason == Reason::Cut);

    const Twist away{-0.3, 0.0, 0.0};
    r = limiter.limit(LimitInput{away, &close, &history, true, Twist{}, t0});
    check("the same wall, driving away: Clear", same(r.command, away) && r.reason == Reason::Clear);
  }

  std::printf("Sliding along a wall keeps the slide (design §7.3)\n");
  {
    // A wall on the left at y = 0.22, 5 cm outside the stop outline.
    // Forward along it never touches; sideways into it touches at 0.25 s
    // and is within the slow zone, 0.05 / 0.25 = 0.2: 0.04 m/s. Together,
    // (0.3, 0.04) reaches it at 1.25 s, after T: kept.
    const Scan left = scanOf(wall(Vec2{-1.0, 0.22}, Vec2{1.0, 0.22}), t0);
    const Twist slide{0.3, 0.2, 0.0};
    const LimitResult r = limiter.limit(LimitInput{slide, &left, &history, true, slide, t0});
    std::printf("        (0.30, 0.20) -> (%.4f, %.4f), %s\n", r.command.vx, r.command.vy,
                toString(r.reason));
    check("forward along it: kept whole, 0.3", near(r.command.vx, 0.3, 1e-12));
    check("sideways into it: cut to 0.04", near(r.command.vy, 0.04, 1e-4) && r.reason == Reason::Cut);
  }

  std::printf("Already too close: backing off and sliding allowed (design §7.3, rule 4)\n");
  {
    // A wall 3 cm in front of the body, 2 cm inside the stop margin.
    const Scan inMargin = scanOf(wall(Vec2{0.18, -0.5}, Vec2{0.18, 0.5}), t0);
    LimitResult r =
        limiter.limit(LimitInput{Twist{0.2, 0.0, 0.0}, &inMargin, &history, true, Twist{}, t0});
    check("forward into it: stopped, BackingOff",
          same(r.command, Twist{}) && r.reason == Reason::BackingOff);
    r = limiter.limit(LimitInput{Twist{-0.2, 0.0, 0.0}, &inMargin, &history, true, Twist{}, t0});
    check("backing off: whole", same(r.command, Twist{-0.2, 0.0, 0.0}));
    r = limiter.limit(LimitInput{Twist{0.0, 0.2, 0.0}, &inMargin, &history, true, Twist{}, t0});
    std::printf("        sliding left along it: vy %.4f\n", r.command.vy);
    check("sliding along it, left: whole", same(r.command, Twist{0.0, 0.2, 0.0}));
    r = limiter.limit(LimitInput{Twist{0.0, -0.2, 0.0}, &inMargin, &history, true, Twist{}, t0});
    check("  and right: whole", same(r.command, Twist{0.0, -0.2, 0.0}));
    r = limiter.limit(LimitInput{Twist{0.1, 0.2, 0.0}, &inMargin, &history, true, Twist{}, t0});
    check("sliding and edging in: the edging stops, the slide stays",
          same(r.command, Twist{0.0, 0.2, 0.0}));
    r = limiter.limit(LimitInput{Twist{0.0, 0.0, 0.5}, &inMargin, &history, true, Twist{}, t0});
    check("turning, which swings a front corner into it: stopped", near(r.command.wz, 0.0, 1e-12));

    // A wall on the left in the margin too: a corner. Backing out
    // diagonally is allowed; sliding along either wall into the other not.
    std::vector<Vec2> corner = wall(Vec2{0.18, -0.5}, Vec2{0.18, 0.5});
    const std::vector<Vec2> side = wall(Vec2{-0.5, 0.15}, Vec2{0.18, 0.15});
    corner.insert(corner.end(), side.begin(), side.end());
    const Scan cornered = scanOf(corner, t0);
    r = limiter.limit(LimitInput{Twist{-0.2, -0.2, 0.0}, &cornered, &history, true, Twist{}, t0});
    check("in a corner: backing out diagonally, whole", same(r.command, Twist{-0.2, -0.2, 0.0}));
    r = limiter.limit(LimitInput{Twist{0.0, 0.2, 0.0}, &cornered, &history, true, Twist{}, t0});
    check("  sliding into the side wall: stopped", near(r.command.vy, 0.0, 1e-9));
  }

  std::printf("Turning near a box's corner\n");
  {
    // A point at (0.22, 0.10), just ahead of the stop outline's front. A
    // left turn (anticlockwise) carries it away; a right turn brings the
    // front edge round onto it after 9.8 deg (0.171 rad): after T = 0.137
    // s, so not cut, but within the slow zone: a corner moving at 0.262 m/s
    // reaches it after 0.045 m, 0.179 of 0.25 m.
    const Scan box = scanOf({Vec2{0.22, 0.10}}, t0);
    LimitResult r =
        limiter.limit(LimitInput{Twist{0.0, 0.0, 1.0}, &box, &history, true, Twist{}, t0});
    check("turning left, away from it: Clear", same(r.command, Twist{0.0, 0.0, 1.0}));
    r = limiter.limit(LimitInput{Twist{0.0, 0.0, -1.0}, &box, &history, true, Twist{}, t0});
    std::printf("        turning right, toward it: wz %.4f, %s\n", r.command.wz, toString(r.reason));
    check("turning right, toward it: slowed to about 0.18 rad/s",
          near(r.command.wz, -0.179, 0.01) && r.reason == Reason::Slowed);
  }

  std::printf("Moving the points for Heron's motion; pose not fresh (design §7.4)\n");
  {
    // Heron drives forward at 0.3 m/s along x and is at the origin now.
    // A wall at x = 0.6; the scan ended 100 ms ago, its point straight
    // ahead seen at its start, 233 ms ago, when Heron was 7 cm back.
    PoseHistory moving;
    for (std::int32_t i = -100; i <= 1; ++i) {
      const double s = 0.01 * i;
      moving.add(PoseSample{t0 + std::chrono::milliseconds(10 * i), Pose2{0.3 * s, 0.0, 0.0},
                            Twist{0.3, 0.0, 0.0}});
    }
    Scan scan;
    scan.seq = 1;
    scan.tEnd = t0 - 100ms;
    scan.tStart = t0 - 233ms;
    scan.sweepStartRad = 0.0f;
    scan.rawSampleCount = 520;
    scan.worstSamplingGapRad = 0.14f;
    const double back = 0.3 * 0.233;
    scan.points.push_back(seen(Vec2{0.6 + back, 0.0}));
    const Twist ahead{0.3, 0.0, 0.0};
    const LimitResult fresh = limiter.limit(LimitInput{ahead, &scan, &moving, true, ahead, t0});
    const LimitResult stale = limiter.limit(LimitInput{ahead, &scan, &moving, false, ahead, t0});
    std::printf("        the point now: %.4f m fresh, %.4f m not; T %.3f s and %.3f s\n",
                fresh.nearest->x, stale.nearest->x, fresh.horizonS, stale.horizonS);
    check("pose fresh: the point moved to where it is now, 0.60 m",
          near(fresh.nearest->x, 0.6, 1e-4));
    check("not fresh: left where it was seen, 0.67 m", near(stale.nearest->x, 0.6 + back, 1e-4));
    check("not fresh: the reaction doubles, T = 0.3 + 2 x 0.237 s",
          near(fresh.horizonS, 0.3 + 0.237, 1e-6) && near(stale.horizonS, 0.3 + 2 * 0.237, 1e-6));
    const PoseHistory empty;
    const LimitResult lost = limiter.limit(LimitInput{ahead, &scan, &empty, true, ahead, t0});
    check("fresh but no pose now in the history: treated as not fresh",
          near(lost.horizonS, stale.horizonS, 1e-9));
  }

  std::printf("The blind spot, checked with the profile (design §7.4)\n");
  {
    check("0.30 x 0.24 m, the LiDAR 6 cm ahead: fits", checkBlindSpot(testConfig()).empty());
    LimiterConfig small = testConfig();
    small.outline = Outline{0.12, 0.10};
    const std::string why = checkBlindSpot(small);
    std::printf("        0.24 x 0.20 m: \"%s\"\n", why.c_str());
    check("0.24 x 0.20 m: refused, 0.187 m past 0.17", !why.empty());
  }

  std::printf("The promise, over random scenes\n");
  {
    // 2000 scenes of a few boxes' worth of points round Heron, none inside
    // the stop margin, and a random command and measured speed. Whatever
    // comes back, held for T, must keep every point out of the body grown
    // by the stop margin, less 1 cm. The 1 cm is the sweep's resolution:
    // turning, a step of 2 deg moves a corner 9 mm, and contact is found
    // at a step's start. The check sweeps ten times finer.
    std::mt19937 random(7);
    std::uniform_real_distribution<double> where(-1.0, 1.0);
    std::uniform_real_distribution<double> speed(-0.5, 0.5);
    std::uniform_real_distribution<double> turn(-1.5, 1.5);
    const LimiterConfig config = testConfig();
    const Outline stop = Heron::Geometry::grown(config.outline, config.stopMarginM);
    const Outline kept = Heron::Geometry::grown(config.outline, config.stopMarginM - 0.01);
    const Heron::Geometry::SweepStep fine{0.002, 0.00349};
    std::int32_t scenes = 0;
    std::int32_t touched = 0;
    std::int32_t limited = 0;
    double worstEarly = 0.0;
    while (scenes < 2000) {
      std::vector<Vec2> points;
      for (std::int32_t box = 0; box < 3; ++box) {
        const Vec2 c{where(random), where(random)};
        const std::vector<Vec2> edge = wall(c, c + Vec2{0.1 * where(random), 0.1 * where(random)});
        points.insert(points.end(), edge.begin(), edge.end());
      }
      bool inMargin = false;
      for (const Vec2 &p : points) {
        inMargin = inMargin || Heron::Geometry::contains(stop, p) ||
                   Heron::Geometry::norm(p - config.lidarOffset) < kBlindRadiusM;
      }
      if (inMargin) {
        continue;
      }
      ++scenes;
      const Scan scan = scanOf(points, t0);
      const Twist command{speed(random), speed(random), turn(random)};
      const Twist measured{speed(random), speed(random), 0.0};
      const LimitResult r = limiter.limit(LimitInput{command, &scan, &history, true, measured, t0});
      limited += same(r.command, command) ? 0 : 1;
      for (const Heron::Point &point : scan.points) {
        const Vec2 p = Heron::Geometry::polarToRobot(point.bearingRad, point.rangeM,
                                                     config.lidarOffset);
        // Held for T, no contact before T.
        if (const auto t = Heron::Geometry::firstContact(kept, p, r.command, r.horizonS, fine)) {
          worstEarly = std::max(worstEarly, r.horizonS - *t);
          if (*t < r.horizonS - 1e-9) {
            ++touched;
            break;
          }
        }
      }
    }
    std::printf("        %d scenes, %d commands limited, %d touched within T (worst %.2g s early)\n",
                scenes, limited, touched, worstEarly);
    check("no limited command brings a point within 4 cm of the body within T", touched == 0);
    check("  and the scenes did limit (the check is not empty)", limited > 200);
  }

  std::printf("Same input, same answer; and the time a call takes\n");
  {
    // 530 points in a ring 0.30 m round the robot, all within reach, and a
    // command on every axis: about the most work one call can be given.
    std::vector<Vec2> ring;
    for (std::int32_t i = 0; i < 530; ++i) {
      const double a = 2.0 * kPi * i / 530.0;
      ring.push_back(Vec2{0.30 * std::cos(a), 0.30 * std::sin(a)});
    }
    const Scan scan = scanOf(ring, t0);
    const Twist all{0.3, 0.2, 1.0};
    const LimitInput input{all, &scan, &history, true, Twist{0.3, 0.2, 0.0}, t0};
    const LimitResult a = limiter.limit(input);
    const LimitResult b = limiter.limit(input);
    check("twice: the same command and reason", same(a.command, b.command, 0.0) &&
                                                     a.reason == b.reason);
    constexpr std::int32_t kCalls = 200;
    const auto start = std::chrono::steady_clock::now();
    double sink = 0.0;
    for (std::int32_t i = 0; i < kCalls; ++i) {
      sink += limiter.limit(input).command.vx;
    }
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count() /
        kCalls;
    std::printf("        530 points in reach, all three axes: %.0f us a call (%s, %.3f)\n", us,
                toString(a.reason), sink / kCalls);
    check("well inside the 20 ms cycle", us < 10000.0);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
