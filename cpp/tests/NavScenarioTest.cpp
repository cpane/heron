// File        : NavScenarioTest.cpp
// Author      : Chris Pane
// Description : The limiter in the loop: the simulated Heron driven by a
//               stick, through the limiter, at walls, along them, and with
//               its LiDAR lost.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The scenarios of docs/navigation-cpp-design.md §9, from design §12.2:
//
//   1. Navigate at a box: pass it with at least 15 cm to spare and be back
//      within 10 cm of the course line within 2 m.
//   2. Assisted, driven at a wall at full speed: stop with the outline
//      outside it, every time, over 1000 runs with random starts, speeds,
//      angles and sensor noise.
//   3. Assisted, sliding along a wall: keep sliding.
//   4. Navigate into a dead end: Blocked, stopped.
//   5. The LiDAR stops mid-run: stop within 300 ms.
//
// Each run is the control loop of design §4 in virtual time: every 10 ms
// the base moves, the OTOS is read into the pose history, and the LiDAR
// turns; every 20 ms the stick's command goes through the limiter with
// the newest scan, and the base is given what comes out. The base brakes
// at 0.6 m/s^2 and answers 60 ms late (BaseConfig); the limiter is told
// the brake, as the profile will tell it the measured one. In Navigate (1
// and 4) the map is built from each scan as Perception builds it, the
// planner runs every 100 ms (and at once for a new course) as
// LocalPlanner runs it, and the control loop drives its plan through the
// limiter as heron_robot will (navigation-cpp-design.md §8.7). No threads, so
// a run of seconds takes milliseconds and every run repeats. Same style
// as the other tests: no framework.
//
// To look into a run: HERON_SCENARIO_DEBUG=1 lists scenario 2's runs that
// came within 3 cm, with their start; HERON_SCENARIO_TRACE=<run> prints
// that run cycle by cycle (clearance, velocity, command, reason, T) before
// the rest.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <thread>
#include <vector>

#include "heron/limiter/Limiter.h"
#include "heron/perception/ObstacleMap.h"
#include "heron/planner/CourseKeeper.h"
#include "heron/planner/Planner.h"
#include "heron/pose/PoseHistory.h"
#include "nav_sim/Base.h"
#include "nav_sim/Lidar.h"
#include "nav_sim/Otos.h"
#include "nav_sim/World.h"

namespace {

using namespace std::chrono_literals;
using Heron::Duration;
using Heron::Scan;
using Heron::TimePoint;
using Heron::Geometry::Outline;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Limiter::LimitInput;
using Heron::Limiter::LimitResult;
using Heron::Limiter::Limiter;
using Heron::Limiter::LimiterConfig;
using Heron::Limiter::Reason;

constexpr double kPi = std::numbers::pi;
constexpr double kDegToRad = kPi / 180.0;
constexpr Duration kTick = 10ms;
constexpr std::int32_t kTicksPerCycle = 2;  // the control loop: 20 ms

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

double seconds(Duration d) {
  return std::chrono::duration<double>(d).count();
}

// The body: 0.30 x 0.24 m, the LiDAR 6 cm ahead, as limiter_test's.
const Outline kBody{0.15, 0.12};

LimiterConfig scenarioConfig() {
  LimiterConfig config;
  config.outline = kBody;
  config.lidarOffset = Vec2{0.06, 0.0};
  config.brakeMps2 = Heron::NavSim::BaseConfig{}.accel;  // the base's own brake
  return config;
}

/** One simulated Heron in its world, and the control loop round it. */
struct Run {
  Run(Heron::NavSim::World w, Pose2 start, std::uint32_t seed)
      : world(std::move(w)),
        base(Heron::NavSim::BaseConfig{}, start, now),
        lidar(world, lidarConfig(seed), now),
        otos(Heron::NavSim::OtosConfig{0.5, 0.001, seed}, now),
        limiter(scenarioConfig()) {
    history.add(otos.read(now, base));
  }

  static Heron::NavSim::LidarConfig lidarConfig(std::uint32_t seed) {
    Heron::NavSim::LidarConfig config;
    config.seed = seed;
    config.noReturnFraction = 0.3;  // a room that answers, as the recording's
    return config;
  }

  /** One 20 ms control cycle: the sensors for two ticks, then the limiter. */
  LimitResult cycle(const Twist &stick) {
    for (std::int32_t i = 0; i < kTicksPerCycle; ++i) {
      now += kTick;
      base.advance(now);
      history.add(otos.read(now, base));
      if (lidarOn) {
        if (auto s = lidar.advance(now, base)) {
          scan = std::move(s);
        }
      }
    }
    const Twist measured = history.newest()->velocity;
    const LimitResult r = limiter.limit(
        LimitInput{stick, scan ? &*scan : nullptr, &history, true, measured, now});
    base.command(r.command, now);
    return r;
  }

  /** How far the body is from a wall along the line x = wallX, robot side. */
  double clearanceTo(double wallX) const {
    const Pose2 p = base.truePose();
    double nearest = 1e9;
    for (const double sx : {-1.0, 1.0}) {
      for (const double sy : {-1.0, 1.0}) {
        const Vec2 corner = Heron::Geometry::toParent(
            p, Vec2{sx * kBody.halfLength, sy * kBody.halfWidth});
        nearest = std::min(nearest, wallX - corner.x);
      }
    }
    return nearest;
  }

  /** The same, for a wall along y = wallY on the left. */
  double clearanceToLeft(double wallY) const {
    const Pose2 p = base.truePose();
    double nearest = 1e9;
    for (const double sx : {-1.0, 1.0}) {
      for (const double sy : {-1.0, 1.0}) {
        const Vec2 corner = Heron::Geometry::toParent(
            p, Vec2{sx * kBody.halfLength, sy * kBody.halfWidth});
        nearest = std::min(nearest, wallY - corner.y);
      }
    }
    return nearest;
  }

  double speed() const {
    const Twist v = base.trueVelocity();
    return std::hypot(v.vx, v.vy);
  }

  TimePoint now{std::chrono::seconds(100)};
  Heron::NavSim::World world;
  Heron::NavSim::Base base;
  Heron::NavSim::Lidar lidar;
  Heron::NavSim::Otos otos;
  Heron::Pose::PoseHistory history;
  std::optional<Scan> scan;
  bool lidarOn{true};
  Limiter limiter;
};

Heron::NavSim::World wallAcross(double x) {
  Heron::NavSim::World world;
  world.segments.push_back(Heron::NavSim::Segment{Vec2{x, -20.0}, Vec2{x, 20.0}});
  return world;
}

/** One run of scenario 2: how it starts, and how it went. */
struct WallRun {
  double wallX{0.0};
  double headingRad{0.0};
  Twist stick{};
  std::uint32_t seed{0};
  double closest{1e9};  // the nearest the body came to the wall
  double final{0.0};    // and where it was at the end
};

/** Drives Heron at the wall for 6 s; fills in closest and final. With
 *  trace, prints every cycle: clearance, velocity, command, reason, T. */
void driveAtWall(WallRun &w, bool trace) {
  Run run(wallAcross(w.wallX), Pose2{0.0, 0.0, w.headingRad}, w.seed);
  for (std::int32_t c = 0; c < 300; ++c) {
    const LimitResult r = run.cycle(w.stick);
    const double clearance = run.clearanceTo(w.wallX);
    if (trace) {
      const Twist v = run.base.trueVelocity();
      std::printf("          %3d clear %6.1f mm  v (%.3f, %.3f)  cmd (%.3f, %.3f)  %s  T %.3f\n", c,
                  1000.0 * clearance, v.vx, v.vy, r.command.vx, r.command.vy,
                  Heron::Limiter::toString(r.reason), r.horizonS);
    }
    w.closest = std::min(w.closest, clearance);
  }
  w.final = run.clearanceTo(w.wallX);
}

/** The least gap between the body, at its true pose, and any point. */
double gapTo(const Pose2 &body, const std::vector<Vec2> &points) {
  double least = 1e9;
  for (const Vec2 &q : points) {
    const Vec2 local = Heron::Geometry::toChild(body, q);
    least = std::min(least, std::max(std::abs(local.x) - kBody.halfLength,
                                      std::abs(local.y) - kBody.halfWidth));
  }
  return least;
}

/** Points every centimetre along a world's segments, to measure gaps to. */
std::vector<Vec2> pointsOf(const std::vector<Heron::NavSim::Segment> &segments) {
  std::vector<Vec2> points;
  for (const Heron::NavSim::Segment &seg : segments) {
    const double length = Heron::Geometry::norm(seg.b - seg.a);
    const auto n = static_cast<std::int32_t>(std::ceil(length / 0.01));
    for (std::int32_t i = 0; i <= n; ++i) {
      points.push_back(seg.a + (static_cast<double>(i) / n) * (seg.b - seg.a));
    }
  }
  return points;
}

/** Navigate: Run, plus the map, the course and the planner. */
struct NavRun {
  NavRun(Heron::NavSim::World w, Pose2 start, std::uint32_t seed)
      : run(std::move(w), start, seed), planner(plannerConfig()) {
    map.grid.reset(Vec2{start.x, start.y});
  }

  static Heron::Planner::PlannerConfig plannerConfig() {
    Heron::Planner::PlannerConfig config;
    config.outline = kBody;
    config.maxSpeedMps = 0.35;
    config.accelMps2 = Heron::NavSim::BaseConfig{}.accel;
    return config;
  }

  /** One 20 ms cycle of Navigate, the stick held as given. */
  LimitResult cycle(const Twist &stick) {
    for (std::int32_t i = 0; i < kTicksPerCycle; ++i) {
      run.now += kTick;
      run.base.advance(run.now);
      run.history.add(run.otos.read(run.now, run.base));
      if (auto s = run.lidar.advance(run.now, run.base)) {
        run.scan = std::move(s);
        (void)Heron::Perception::addScan(map, *run.scan, run.history, 1,
                                         Heron::Perception::LidarMount{Vec2{0.06, 0.0}}, run.now);
      }
    }
    const Pose2 pose = run.history.at(run.now).pose;
    const std::optional<Heron::Planner::Course> course = keeper.update(stick, pose, 1);
    if (course && (!plan || plan->courseSeq != course->seq || run.now - plan->time >= 100ms)) {
      Heron::Planner::PlanInput input;
      input.map = &map;
      input.pose = pose;
      input.epoch = 1;
      input.poseFresh = true;
      input.measured = run.history.newest()->velocity;
      input.course = course;
      input.now = run.now;
      plan = planner.plan(input);
      if (trace) {
        const Pose2 t = run.base.truePose();
        std::printf("          %6.2f s  at (%.3f, %.3f)  plan (%.3f, %.3f) %s  clearance %.3f\n",
                    seconds(run.now.time_since_epoch()) - 100.0, t.x, t.y, plan->twist.vx,
                    plan->twist.vy, Heron::Planner::toString(plan->state), plan->clearanceM);
      }
      blocked += plan->state == Heron::Planner::State::Blocked ? 1 : 0;
      avoiding += plan->state == Heron::Planner::State::Avoiding ? 1 : 0;
    }
    Twist want{};
    if (course && plan && plan->courseSeq == course->seq && run.now - plan->time <= 300ms) {
      want = Twist{plan->twist.vx, plan->twist.vy, course->turnRateRadps};
    }
    const Twist measured = run.history.newest()->velocity;
    const LimitResult r = run.limiter.limit(
        LimitInput{want, run.scan ? &*run.scan : nullptr, &run.history, true, measured, run.now});
    const bool moving = want.vx != 0.0 || want.vy != 0.0;
    driven += moving ? 1 : 0;
    cut += moving && (r.reason == Reason::Cut || r.reason == Reason::BackingOff) ? 1 : 0;
    slowed += moving && r.reason == Reason::Slowed ? 1 : 0;
    run.base.command(r.command, run.now);
    return r;
  }

  Run run;
  Heron::Perception::ObstacleMap map{Heron::Perception::ObstacleGrid{}, 1};
  Heron::Planner::CourseKeeper keeper;
  Heron::Planner::Planner planner;
  std::optional<Heron::Planner::Plan> plan;
  std::int32_t blocked{0};   // plans
  std::int32_t avoiding{0};  // plans
  std::int32_t driven{0};    // cycles with a non-zero plan driven
  std::int32_t cut{0};       // ...of which the limiter cut (Cut, BackingOff)
  std::int32_t slowed{0};    // ...or only slowed, in its slow zone
  bool trace{false};         // print every plan
};

}  // namespace

int main() {
  std::printf("Scenario 1: Navigate at a box\n");
  {
    // A room 9 x 4 m and a 0.3 m box on the course, at two distances and
    // five offsets from the line, dead centre included; each run with its
    // own sensor noise. The stick held forward at 0.3 m/s until Heron is
    // 4.5 m along. HERON_SCENARIO_TRACE1=<run> prints that run's plans.
    struct BoxRun {
      Vec2 box{};
      std::uint32_t seed{0};
      bool reached{false};
      double closest{1e9};
      double widest{0.0};
      std::optional<double> yAfter;  // off the line 2 m past the box
      double seconds{0.0};
      std::int32_t driven{0};
      std::int32_t cut{0};
      std::int32_t slowed{0};
    };
    std::vector<BoxRun> runs;
    std::uint32_t seed = 100;
    for (const double x : {1.35, 1.95}) {
      for (const double y : {-0.10, -0.05, 0.0, 0.05, 0.10}) {
        BoxRun b;
        b.box = Vec2{x, y};
        b.seed = ++seed;
        runs.push_back(b);
      }
    }
    // Independent runs, shared across the cores as scenario 2's are.
    const char *traceRun = std::getenv("HERON_SCENARIO_TRACE1");
    const auto drive = [traceRun](BoxRun &b, std::size_t r) {
      Heron::NavSim::World world;
      world.addBox(Vec2{2.5, 0.0}, 9.0, 4.0);
      Heron::NavSim::World boxOnly;
      boxOnly.addBox(b.box, 0.3, 0.3);
      world.segments.insert(world.segments.end(), boxOnly.segments.begin(), boxOnly.segments.end());
      const std::vector<Vec2> box = pointsOf(boxOnly.segments);
      NavRun nav(std::move(world), Pose2{}, b.seed);
      nav.trace = traceRun != nullptr && static_cast<std::size_t>(std::atoi(traceRun)) == r;
      const Twist stick{0.3, 0.0, 0.0};
      std::int32_t c = 0;
      for (; c < 1500 && nav.run.base.truePose().x < 4.5; ++c) {
        (void)nav.cycle(stick);
        const Pose2 p = nav.run.base.truePose();
        b.closest = std::min(b.closest, gapTo(p, box));
        b.widest = std::max(b.widest, std::abs(p.y));
        if (!b.yAfter && p.x >= b.box.x + 0.15 + 2.0) {
          b.yAfter = p.y;
        }
      }
      b.reached = nav.run.base.truePose().x >= 4.5;
      b.seconds = 0.02 * c;
      b.driven = nav.driven;
      b.cut = nav.cut;
      b.slowed = nav.slowed;
    };
    const std::size_t workers = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    for (std::size_t w = 0; w < workers; ++w) {
      threads.emplace_back([&runs, &drive, w, workers]() {
        for (std::size_t r = w; r < runs.size(); r += workers) {
          drive(runs[r], r);
        }
      });
    }
    for (std::thread &t : threads) {
      t.join();
    }
    for (const BoxRun &b : runs) {
      std::printf("        box at (%.2f, %+.2f): %.1f s; closest %.0f mm; widest %.0f mm off the "
                  "line, %.0f mm 2 m past; limiter cut %d, slowed %d of %d\n",
                  b.box.x, b.box.y, b.seconds, 1000.0 * b.closest, 1000.0 * b.widest,
                  b.yAfter ? 1000.0 * *b.yAfter : -1.0, b.cut, b.slowed, b.driven);
    }
    bool reached = true;
    bool spare = true;
    bool back = true;
    bool uncut = true;
    for (const BoxRun &b : runs) {
      reached = reached && b.reached;
      spare = spare && b.closest >= 0.15;
      back = back && b.yAfter && std::abs(*b.yAfter) <= 0.10;
      uncut = uncut && b.cut == 0;
    }
    check("every run reached 4.5 m along", reached);
    check("passed the box with at least 15 cm to spare, every run", spare);
    check("back within 10 cm of the line 2 m past it, every run", back);
    check("the limiter cut none of the planner's choices (I11)", uncut);
  }

  std::printf("Scenario 2: driven at a wall, 1000 random runs\n");
  {
    // The wall across the way at 0.6 to 1.5 m; Heron facing it within 60
    // deg either side, pushed at 0.2 to 0.6 m/s with a random sideways
    // part, sensor noise on, each run its own seed, for 6 s: long enough
    // to reach the wall and creep through the slow zone, where the margin
    // is tested.
    //
    // Every run's start is drawn first, then the runs share the cores:
    // they are independent, and 1000 of them took 88 s on the Pi one at a
    // time. The results do not depend on how many threads there are.
    std::mt19937 random(2026);
    std::uniform_real_distribution<double> distance(0.6, 1.5);
    std::uniform_real_distribution<double> angle(-60.0, 60.0);
    std::uniform_real_distribution<double> push(0.2, 0.6);
    std::uniform_real_distribution<double> sideways(-0.15, 0.15);
    constexpr std::int32_t kRuns = 1000;
    std::vector<WallRun> runs;
    for (std::int32_t i = 0; i < kRuns; ++i) {
      WallRun w;
      w.wallX = distance(random);
      w.headingRad = angle(random) * kDegToRad;
      w.stick = Twist{push(random), sideways(random), 0.0};
      w.seed = static_cast<std::uint32_t>(i + 1);
      runs.push_back(w);
    }

    // One run, printed cycle by cycle, to look into it.
    if (const char *trace = std::getenv("HERON_SCENARIO_TRACE")) {
      WallRun one = runs[static_cast<std::size_t>(std::atoi(trace))];
      driveAtWall(one, true);
    }

    const auto start = std::chrono::steady_clock::now();
    const std::size_t workers = std::max(1u, std::thread::hardware_concurrency());
    std::vector<std::thread> threads;
    for (std::size_t w = 0; w < workers; ++w) {
      threads.emplace_back([&runs, w, workers]() {
        for (std::size_t i = w; i < runs.size(); i += workers) {
          driveAtWall(runs[i], false);
        }
      });
    }
    for (std::thread &t : threads) {
      t.join();
    }
    const double took =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    std::int32_t touched = 0;
    double worstClearance = 1e9;
    double worstFinal = 1e9;
    double sumFinal = 0.0;
    std::vector<double> finals;
    for (std::size_t i = 0; i < runs.size(); ++i) {
      const WallRun &w = runs[i];
      if (w.closest < 0.03 && std::getenv("HERON_SCENARIO_DEBUG") != nullptr) {
        std::printf("        run %zu: wall %.2f m, heading %.1f deg, stick (%.2f, %.2f), "
                    "closest %.1f mm\n",
                    i, w.wallX, w.headingRad / kDegToRad, w.stick.vx, w.stick.vy,
                    1000.0 * w.closest);
      }
      touched += w.closest <= 0.0 ? 1 : 0;
      worstClearance = std::min(worstClearance, w.closest);
      worstFinal = std::min(worstFinal, w.final);
      sumFinal += w.final;
      finals.push_back(w.final);
    }
    std::sort(finals.begin(), finals.end());
    std::printf("        closest the body came to the wall: %.1f mm; at the end %.1f mm "
                "(nearest), %.1f (median), %.1f mm (mean)\n",
                1000.0 * worstClearance, 1000.0 * worstFinal,
                1000.0 * finals[finals.size() / 2], 1000.0 * sumFinal / kRuns);
    std::printf("        %d runs of 6 s in %.2f s, %zu threads\n", kRuns, took, workers);
    check("the body never touched the wall, in 1000 runs", touched == 0);
    // The margin is 5 cm. With no range noise the closest any run comes is
    // 49 mm; with the simulator's 5 mm (a guess, not measured) 23.5 mm:
    // the closest point is the least of many noisy ones, so it reads low,
    // and rule 4's "no nearer than the closest" lets some approach through
    // (docs/navigation-cpp-design.md §6.2). A guard against that growing.
    check("never within 2 cm of it", worstClearance >= 0.02);
  }

  std::printf("Scenario 3: sliding along a wall\n");
  {
    // A wall on the left at y = 0.40, the body's side 0.28 m from it.
    // Pushed forward at 0.3 m/s and into the wall at 0.2 m/s for 6 s:
    // Heron should close in, then keep going along it.
    constexpr double kWallY = 0.40;
    Heron::NavSim::World world;
    world.segments.push_back(Heron::NavSim::Segment{Vec2{-5.0, kWallY}, Vec2{20.0, kWallY}});
    Run run(std::move(world), Pose2{}, 3);
    const Twist stick{0.3, 0.2, 0.0};
    double worst = 1e9;
    double xAt4 = 0.0;
    std::int32_t backingOff = 0;
    for (std::int32_t c = 0; c < 300; ++c) {
      const LimitResult r = run.cycle(stick);
      backingOff += r.reason == Reason::BackingOff ? 1 : 0;
      worst = std::min(worst, run.clearanceToLeft(kWallY));
      if (c == 199) {
        xAt4 = run.base.truePose().x;
      }
    }
    const double xAt6 = run.base.truePose().x;
    const double gap = run.clearanceToLeft(kWallY);
    std::printf("        after 6 s: %.2f m along, %.1f mm from the wall (closest %.1f mm); "
                "%.2f m/s over the last 2 s; %d cycles BackingOff\n",
                xAt6, 1000.0 * gap, 1000.0 * worst, (xAt6 - xAt4) / 2.0, backingOff);
    check("never touched the wall", worst > 0.0);
    check("closed in on it: within the 25 cm slow zone", gap < 0.25);
    check("kept sliding: over 0.25 m/s along it over the last 2 s", (xAt6 - xAt4) / 2.0 > 0.25);
  }

  std::printf("Scenario 4: Navigate into a dead end\n");
  {
    // A corridor 0.8 m wide, closed 1.5 m ahead, in a room; the stick held
    // forward at 0.3 m/s for 12 s.
    Heron::NavSim::World world;
    world.addBox(Vec2{0.5, 0.0}, 6.0, 4.0);
    std::vector<Heron::NavSim::Segment> end{
        {Vec2{-1.0, 0.4}, Vec2{1.5, 0.4}},
        {Vec2{-1.0, -0.4}, Vec2{1.5, -0.4}},
        {Vec2{1.5, -0.4}, Vec2{1.5, 0.4}}};
    world.segments.insert(world.segments.end(), end.begin(), end.end());
    const std::vector<Vec2> walls = pointsOf(end);
    NavRun nav(std::move(world), Pose2{}, 4);
    const Twist stick{0.3, 0.0, 0.0};
    double closest = 1e9;
    for (std::int32_t c = 0; c < 600; ++c) {
      (void)nav.cycle(stick);
      closest = std::min(closest, gapTo(nav.run.base.truePose(), walls));
    }
    const double gap = gapTo(nav.run.base.truePose(), walls);
    const Heron::Planner::State last = nav.plan ? nav.plan->state : Heron::Planner::State::Stopped;
    std::printf("        after 12 s: %s, %.0f mm from the end, %.3f m/s; closest %.1f mm; "
                "%d plans blocked\n",
                Heron::Planner::toString(last), 1000.0 * gap, nav.run.speed(), 1000.0 * closest,
                nav.blocked);
    check("Blocked", last == Heron::Planner::State::Blocked);
    check("stopped", nav.run.speed() < 0.005);
    check("never within the 5 cm margin of a wall", closest >= 0.05);
    check("came within 30 cm of the end before stopping", gap <= 0.30);
  }

  std::printf("Scenario 5: the LiDAR stops mid-run\n");
  {
    // An empty room (every scan covered, nothing in it), Heron pushed
    // forward at 0.4 m/s. After 2 s the LiDAR stops: no new scan. The
    // limiter must stop the command once the last scan is 300 ms old.
    Run run(Heron::NavSim::World{}, Pose2{}, 5);
    const Twist stick{0.4, 0.0, 0.0};
    bool heldAtStart = true;
    for (std::int32_t c = 0; c < 5; ++c) {
      const LimitResult r = run.cycle(stick);
      heldAtStart = heldAtStart && (run.scan.has_value() || r.reason == Reason::NoScan);
    }
    while (run.now < TimePoint{std::chrono::seconds(102)}) {
      (void)run.cycle(stick);
    }
    const double speedBefore = run.speed();
    run.lidarOn = false;
    const TimePoint lastEnd = run.scan->tEnd;
    std::optional<TimePoint> stoppedAt;
    std::optional<TimePoint> stillAt;
    Reason why = Reason::Clear;
    for (std::int32_t c = 0; c < 100; ++c) {
      const LimitResult r = run.cycle(stick);
      if (!stoppedAt && r.command.vx == 0.0) {
        stoppedAt = run.now;
        why = r.reason;
      }
      if (!stillAt && run.speed() < 1e-6) {
        stillAt = run.now;
      }
    }
    const double afterScan = seconds(*stoppedAt - lastEnd);
    const double toStill = seconds(*stillAt - *stoppedAt);
    std::printf("        driving at %.2f m/s; the command stopped %.0f ms after the last scan "
                "ended (%s); the base still %.0f ms later\n",
                speedBefore, 1000.0 * afterScan, Heron::Limiter::toString(why), 1000.0 * toStill);
    check("before the first scan: no motion (NoScan)", heldAtStart);
    check("the command stops within 300 ms of the last scan, plus one cycle",
          afterScan <= 0.300 + 0.020 && why == Reason::Stale);
    check("the base is still within its lag and braking time, 60 ms + 0.4 / 0.6 s",
          toStill <= 0.060 + 0.4 / 0.6 + 0.020);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
