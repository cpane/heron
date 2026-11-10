// File        : PlannerTest.cpp
// Author      : Chris Pane
// Description : The local planner's core, its distance field and its course,
//               each against hand-built maps.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// docs/navigation-cpp-design.md §8.8, planner_test: CourseKeeper's rules;
// the distance field against brute force; and Planner on maps built by
// hand from rays (straight on, round a box, a dead end, unknown cells near
// and far and under Heron, already close, every reason it stops, the speed
// window and cap, the line). The scenarios with the simulator are in
// nav_scenario_test. Same style as the other tests: no framework.

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

#include "heron/planner/CourseKeeper.h"
#include "heron/planner/Planner.h"
#include "planner/DistanceField.h"

namespace {

using namespace std::chrono_literals;
using namespace Heron::Planner;
using Heron::TimePoint;
using Heron::Geometry::Outline;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Perception::kCellM;
using Heron::Perception::kGridCells;
using Heron::Perception::ObstacleGrid;
using Heron::Perception::ObstacleMap;

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;
const TimePoint kNow{100s};

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

// The body of limiter_test and the scenarios: 0.30 x 0.24 m.
PlannerConfig testConfig() {
  PlannerConfig config;
  config.outline = Outline{0.15, 0.12};
  return config;
}

/** An unknown map round the origin, in epoch 1, its scan just ended. */
ObstacleMap emptyMap() {
  ObstacleMap map{ObstacleGrid{}};
  map.grid.reset(Vec2{0.0, 0.0});
  map.epoch = 1;
  map.scanEnd = kNow;
  return map;
}

/** Sees the map free from inner to outer metres round the origin, twice
 *  (two passes make a cell free), except where skip says. */
template <typename Skip>
void seeFree(ObstacleMap &map, double inner, double outer, Skip skip) {
  for (int pass = 0; pass < 2; ++pass) {
    for (int i = 0; i < 2880; ++i) {
      const double a = 2.0 * kPi * i / 2880;
      const Vec2 u{std::cos(a), std::sin(a)};
      const double reach = skip(a, outer);
      if (reach > inner) {
        map.grid.addRay(inner * u, reach * u, false);
      }
    }
  }
}

void seeFree(ObstacleMap &map, double inner = 0.0, double outer = 3.0) {
  seeFree(map, inner, outer, [](double, double reach) { return reach; });
}

/** A hit at every centimetre of the segment, seen from the origin. */
void seeWall(ObstacleMap &map, Vec2 a, Vec2 b) {
  const double length = Heron::Geometry::norm(b - a);
  const int n = static_cast<int>(std::ceil(length / 0.01));
  for (int i = 0; i <= n; ++i) {
    map.grid.addRay(Vec2{0.0, 0.0}, a + (static_cast<double>(i) / n) * (b - a), true);
  }
}

void seeBox(ObstacleMap &map, Vec2 c, double length, double width) {
  const double hx = length / 2.0;
  const double hy = width / 2.0;
  seeWall(map, Vec2{c.x - hx, c.y - hy}, Vec2{c.x + hx, c.y - hy});
  seeWall(map, Vec2{c.x + hx, c.y - hy}, Vec2{c.x + hx, c.y + hy});
  seeWall(map, Vec2{c.x + hx, c.y + hy}, Vec2{c.x - hx, c.y + hy});
  seeWall(map, Vec2{c.x - hx, c.y + hy}, Vec2{c.x - hx, c.y - hy});
}

Course courseAlong(double headingRad, double speed, Vec2 anchor = Vec2{}) {
  Course course;
  course.seq = 7;
  course.epoch = 1;
  course.anchor = anchor;
  course.headingRad = headingRad;
  course.speedMps = speed;
  return course;
}

PlanInput inputFor(const ObstacleMap &map, const Course &course, Twist measured = Twist{}) {
  PlanInput input;
  input.map = &map;
  input.pose = Pose2{};
  input.epoch = 1;
  input.poseFresh = true;
  input.measured = measured;
  input.course = course;
  input.now = kNow;
  return input;
}

/** The nearest a straight line of the body's path comes, in truth, to a
 *  point: the least rectangle gap over the published path. */
double trueClearance(const Plan &plan, const Outline &body, const std::vector<Vec2> &points) {
  double least = 1e9;
  for (const Pose2 &p : plan.path) {
    for (const Vec2 &q : points) {
      const Vec2 local = Heron::Geometry::toChild(p, q);
      least = std::min(least, std::max(std::abs(local.x) - body.halfLength,
                                       std::abs(local.y) - body.halfWidth));
    }
  }
  return least;
}

std::vector<Vec2> boxPoints(Vec2 c, double length, double width) {
  std::vector<Vec2> points;
  for (double x = -length / 2; x <= length / 2 + 1e-9; x += 0.01) {
    points.push_back(Vec2{c.x + x, c.y - width / 2});
    points.push_back(Vec2{c.x + x, c.y + width / 2});
  }
  for (double y = -width / 2; y <= width / 2 + 1e-9; y += 0.01) {
    points.push_back(Vec2{c.x - length / 2, c.y + y});
    points.push_back(Vec2{c.x + length / 2, c.y + y});
  }
  return points;
}

void print(const char *label, const Plan &plan) {
  std::printf("        %s: %s %s, (%.3f, %.3f), clearance %.3f, %d of %d passed\n", label,
              toString(plan.state), toString(plan.reason), plan.twist.vx, plan.twist.vy,
              plan.clearanceM, plan.candidatesPassed, plan.candidatesTried);
}

}  // namespace

int main() {
  std::printf("CourseKeeper (design §8.1)\n");
  {
    CourseKeeper keeper;
    const Pose2 at{1.0, 2.0, 0.5};
    auto c = keeper.update(Twist{0.2, 0.0, 0.3}, at, 1);
    check("engage: the heading is the stick's, turned into the odom frame",
          c && near(c->headingRad, 0.5, 1e-12) && c->seq == 1);
    check("engage: the line runs through Heron; speed and turn rate are the stick's",
          c && c->anchor.x == 1.0 && c->anchor.y == 2.0 && near(c->speedMps, 0.2, 1e-12) &&
              c->turnRateRadps == 0.3);
    const Pose2 moved{1.5, 2.1, 0.5};
    c = keeper.update(Twist{0.3 * std::cos(9 * kDeg), 0.3 * std::sin(9 * kDeg), 0.0}, moved, 1);
    check("the stick 9 deg round: the same line, the new speed",
          c && c->seq == 1 && c->anchor.x == 1.0 && near(c->speedMps, 0.3, 1e-12));
    c = keeper.update(Twist{0.3 * std::cos(11 * kDeg), 0.3 * std::sin(11 * kDeg), 0.0}, moved, 1);
    check("11 deg round: re-anchored where Heron is", c && c->seq == 2 && c->anchor.x == 1.5 &&
                                                          near(c->headingRad, 0.5 + 11 * kDeg, 1e-12));
    const Pose2 turned{1.5, 2.1, 0.5 + 11 * kDeg + 12 * kDeg};
    c = keeper.update(Twist{0.3, 0.0, 0.0}, turned, 1);
    check("Heron turned 12 deg under a held stick: re-anchored", c && c->seq == 3);
    c = keeper.update(Twist{0.0, 0.0, 0.4}, turned, 1);
    check("let go (turning only): no course", !c);
    c = keeper.update(Twist{0.3, 0.0, 0.0}, turned, 1);
    check("pushed again: engaged afresh", c && c->seq == 4);
    c = keeper.update(Twist{0.3, 0.0, 0.0}, turned, 2);
    check("a new epoch: re-anchored in it", c && c->seq == 5 && c->epoch == 2);
  }

  std::printf("The distance field (§8.4)\n");
  {
    std::mt19937 random(11);
    std::bernoulli_distribution flagged(0.02);
    bool exact = true;
    for (int trial = 0; trial < 5 && exact; ++trial) {
      CellFlags flags{};
      std::vector<std::pair<int, int>> cells;
      for (int iy = 0; iy < kGridCells; ++iy) {
        for (int ix = 0; ix < kGridCells; ++ix) {
          if (flagged(random)) {
            flags[static_cast<std::size_t>(iy * kGridCells + ix)] = true;
            cells.emplace_back(ix, iy);
          }
        }
      }
      const DistanceField field(flags, Vec2{-2.0, -2.0});
      for (int iy = 0; iy < kGridCells && exact; ++iy) {
        for (int ix = 0; ix < kGridCells && exact; ++ix) {
          double best = 1e9;
          for (const auto &[cx, cy] : cells) {
            best = std::min(best, std::hypot(ix - cx, iy - cy) * kCellM);
          }
          exact = near(field.atCell(ix, iy), best, 1e-9);
        }
      }
    }
    check("every cell exact against brute force, 5 random grids of 2% flagged", exact);
    const DistanceField none(CellFlags{}, Vec2{-2.0, -2.0});
    check("nothing flagged: far everywhere", none.at(Vec2{0.0, 0.0}) == DistanceField::kFar);
    CellFlags one{};
    one[static_cast<std::size_t>(40 * kGridCells + 40)] = true;  // the cell at (0, 0)
    const DistanceField single(one, Vec2{-2.0, -2.0});
    check("1 m away along x, at a cell centre: 1.0 m", near(single.at(Vec2{1.025, 0.025}), 1.0, 1e-9));
    check("half way between centres: interpolated",
          near(single.at(Vec2{1.05, 0.025}), 1.025, 1e-9));
    const double outside = single.at(Vec2{3.0, 0.01});
    check("outside the grid: never more than the truth (2.975 m)", outside <= 2.975 && outside >= 0.0);
  }

  const PlannerConfig config = testConfig();
  const Outline body = config.outline;

  std::printf("Straight on (§8.3)\n");
  {
    ObstacleMap map = emptyMap();
    seeFree(map);
    Planner planner(config);
    Plan plan = planner.plan(inputFor(map, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0}));
    print("at 0.3 m/s", plan);
    check("an empty map at speed: Following, straight on at 0.3 m/s",
          plan.state == State::Following && near(plan.twist.vx, 0.3, 1e-9) &&
              near(plan.twist.vy, 0.0, 1e-9));
    check("the path: 1.5 s every 0.1 s, from where Heron is",
          plan.path.size() == 16 && plan.path.front().x == 0.0 && near(plan.path.back().x, 0.45, 1e-9));

    planner.reset();
    plan = planner.plan(inputFor(map, courseAlong(0.0, 0.3)));
    print("from rest", plan);
    check("from rest: as fast as 0.3 s at 0.6 m/s^2 reaches, 0.18 m/s",
          plan.state == State::Following && near(plan.twist.vx, 0.18, 1e-9));

    plan = planner.plan(inputFor(map, courseAlong(0.0, 0.1)));
    check("the stick asks 0.1 m/s: no faster (I13)",
          plan.state == State::Following && near(plan.twist.vx, 0.1, 1e-9));

    Course turning = courseAlong(0.0, 0.3);
    turning.turnRateRadps = 0.8;
    plan = planner.plan(inputFor(map, turning, Twist{0.3, 0.0, 0.0}));
    check("the turn rate is the course's, passed through (I16)", plan.twist.wz == 0.8);

    plan = planner.plan(inputFor(map, courseAlong(kPi / 2.0, 0.3), Twist{0.0, 0.3, 0.0}));
    check("a course to the left: sliding left, heading kept",
          plan.state == State::Following && near(plan.twist.vy, 0.3, 1e-9) &&
              near(plan.twist.vx, 0.0, 1e-9) && plan.twist.wz == 0.0);
  }

  std::printf("Round a box\n");
  {
    // A 0.3 m box, its near face 0.45 m ahead of Heron's front, a little
    // to the left; Heron at 0.3 m/s would reach it in 1.5 s.
    const Vec2 centre{0.75, 0.03};
    ObstacleMap map = emptyMap();
    seeFree(map);
    seeBox(map, centre, 0.3, 0.3);
    Planner planner(config);
    const Plan plan = planner.plan(inputFor(map, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0}));
    print("box ahead", plan);
    const double truth = trueClearance(plan, body, boxPoints(centre, 0.3, 0.3));
    std::printf("        the chosen path's true clearance: %.3f m\n", truth);
    check("Avoiding, sliding right, round its far side", plan.state == State::Avoiding &&
                                                              plan.twist.vy < -0.05);
    check("the path keeps the stop margin in truth (I11)", truth >= config.stopMarginM);
    check("and still makes progress", plan.twist.vx > 0.0);
  }

  std::printf("A dead end\n");
  {
    // Walls 12 cm ahead of the front and 18 cm beside each side: already
    // inside the planner's margin ahead, nothing forward can pass.
    ObstacleMap map = emptyMap();
    seeFree(map);
    seeWall(map, Vec2{0.27, -0.30}, Vec2{0.27, 0.30});
    seeWall(map, Vec2{-0.6, 0.30}, Vec2{0.27, 0.30});
    seeWall(map, Vec2{-0.6, -0.30}, Vec2{0.27, -0.30});
    Planner planner(config);
    const Plan plan = planner.plan(inputFor(map, courseAlong(0.0, 0.3), Twist{0.1, 0.0, 0.0}));
    print("dead end", plan);
    check("Blocked, NothingSafe, a zero twist",
          plan.state == State::Blocked && plan.reason == Reason::NothingSafe &&
              plan.twist.vx == 0.0 && plan.twist.vy == 0.0);
  }

  std::printf("Unknown cells (P6)\n");
  {
    Planner planner(config);
    // Never seen: a sector 15 deg either side straight ahead.
    ObstacleMap ahead = emptyMap();
    seeFree(ahead, 0.0, 3.0, [](double a, double reach) {
      return std::abs(std::remainder(a, 2.0 * kPi)) < 15.0 * kDeg ? 0.0 : reach;
    });
    Plan plan = planner.plan(inputFor(ahead, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0}));
    print("unknown ahead", plan);
    check("unknown within 0.6 m ahead: not driven straight into", plan.state != State::Following &&
                                                                    plan.twist.vx < 0.3);

    // Seen free only to 0.75 m ahead; beyond, unknown.
    ObstacleMap far = emptyMap();
    seeFree(far, 0.0, 3.0, [](double a, double reach) {
      return std::abs(std::remainder(a, 2.0 * kPi)) < 15.0 * kDeg ? 0.75 : reach;
    });
    planner.reset();
    plan = planner.plan(inputFor(far, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0}));
    print("unknown past 0.75 m", plan);
    check("unknown beyond 0.6 m: ignored, Following", plan.state == State::Following);

    // Seen free, then made unknown again within 0.2 m of Heron's centre:
    // under it. (Rays started 0.2 m out would do, but they skip a few
    // cells farther out: the grid draws each ray from cell to cell.)
    ObstacleMap under = emptyMap();
    seeFree(under);
    for (int iy = 0; iy < kGridCells; ++iy) {
      for (int ix = 0; ix < kGridCells; ++ix) {
        const Vec2 c = under.grid.centreOf(Heron::Perception::CellIndex{ix, iy});
        if (Heron::Geometry::norm(c) <= 0.2) {
          for (int k = 0; k < 3; ++k) {
            under.grid.addRay(c, c, true);  // -8 + 9: unknown
          }
        }
      }
    }
    const bool unknownUnder = under.grid.classify(Vec2{0.1, 0.1}) == Heron::Perception::Cell::Unknown;
    planner.reset();
    plan = planner.plan(inputFor(under, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0}));
    print("unknown under Heron", plan);
    check("unknown under Heron and its margin: ignored, Following",
          unknownUnder && plan.state == State::Following);
  }

  std::printf("Already close (I12)\n");
  {
    // A wall 5 cm from the front: inside the planner's margin now.
    ObstacleMap map = emptyMap();
    seeFree(map);
    seeWall(map, Vec2{0.20, -2.0}, Vec2{0.20, 2.0});
    Planner planner(config);
    Plan plan = planner.plan(inputFor(map, courseAlong(0.0, 0.3)));
    print("into it", plan);
    check("into the wall: Blocked", plan.state == State::Blocked);
    planner.reset();
    plan = planner.plan(inputFor(map, courseAlong(kPi, 0.3)));
    print("away", plan);
    check("away from it: allowed", plan.state == State::Following && plan.twist.vx < 0.0);
    planner.reset();
    plan = planner.plan(inputFor(map, courseAlong(kPi / 2.0, 0.3)));
    print("along", plan);
    check("along it: allowed, and nothing nearer",
          plan.state != State::Blocked && plan.twist.vy > 0.0 && plan.twist.vx <= 1e-9);
  }

  std::printf("When it cannot plan\n");
  {
    ObstacleMap map = emptyMap();
    seeFree(map);
    Planner planner(config);
    PlanInput input = inputFor(map, courseAlong(0.0, 0.3), Twist{0.3, 0.0, 0.0});
    const auto stoppedFor = [&planner](PlanInput in, Reason why) {
      const Plan p = planner.plan(in);
      return p.state == State::Stopped && p.reason == why && p.twist.vx == 0.0 &&
             p.twist.vy == 0.0;
    };
    PlanInput in = input;
    in.course.reset();
    check("no course: Stopped, NoCourse", stoppedFor(in, Reason::NoCourse));
    in = input;
    in.map = nullptr;
    check("no map: Stopped, NoMap", stoppedFor(in, Reason::NoMap));
    in = input;
    in.now = kNow + 500ms;
    check("a map 500 ms old: still used", planner.plan(in).state == State::Following);
    in.now = kNow + 501ms;
    check("501 ms old: Stopped, StaleMap", stoppedFor(in, Reason::StaleMap));
    in = input;
    in.poseFresh = false;
    check("the pose not fresh: Stopped, PoseNotFresh", stoppedFor(in, Reason::PoseNotFresh));
    in = input;
    in.epoch = 2;
    check("the pose in another epoch: Stopped, OtherFrame", stoppedFor(in, Reason::OtherFrame));
    in = input;
    in.course->epoch = 2;
    check("the course in another epoch: Stopped, OtherFrame", stoppedFor(in, Reason::OtherFrame));
    check("a plan says which course it was made for", planner.plan(input).courseSeq == 7);
  }

  std::printf("The line (I14)\n");
  {
    // Heron already 1 m left of its line, a box ahead a little to the
    // right of centre: the easy way round is left, farther off the line.
    const Vec2 centre{0.75, -0.04};
    ObstacleMap map = emptyMap();
    seeFree(map);
    seeBox(map, centre, 0.3, 0.3);
    Planner planner(config);
    const Plan plan =
        planner.plan(inputFor(map, courseAlong(0.0, 0.3, Vec2{0.0, -1.0}), Twist{0.3, 0.0, 0.0}));
    print("1 m off the line", plan);
    check("never farther than 1 m off the line: round the right", plan.state == State::Avoiding &&
                                                                       plan.twist.vy < 0.0);
    check("the path ends no farther off", plan.path.back().y <= 1e-9);
  }

  std::printf("Time\n");
  {
    ObstacleMap map = emptyMap();
    seeFree(map);
    seeBox(map, Vec2{0.75, 0.03}, 0.3, 0.3);
    Planner planner(config);
    Course turning = courseAlong(0.0, 0.3);
    turning.turnRateRadps = 2.0;
    const PlanInput input = inputFor(map, turning, Twist{0.3, 0.0, 0.0});
    constexpr int kCalls = 50;
    const auto start = std::chrono::steady_clock::now();
    int tried = 0;
    for (int i = 0; i < kCalls; ++i) {
      tried = planner.plan(input).candidatesTried;
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() /
        kCalls;
    std::printf("        plan(), a box ahead, turning at 2 rad/s: %.2f ms a call, %d candidates\n",
                ms, tried);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
