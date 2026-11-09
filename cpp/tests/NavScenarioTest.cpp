// File        : NavScenarioTest.cpp
// Author      : Chris Pane
// Description : The limiter in the loop: the simulated Heron driven by a
//               stick, through the limiter, at walls, along them, and with
//               its LiDAR lost.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The N4 scenarios of docs/navigation-cpp-design.md §9, from design §12.2:
//
//   2. Assisted, driven at a wall at full speed: stop with the outline
//      outside it, every time, over 1000 runs with random starts, speeds,
//      angles and sensor noise.
//   3. Assisted, sliding along a wall: keep sliding.
//   5. The LiDAR stops mid-run: stop within 300 ms.
//
// Each run is the control loop of design §4 in virtual time: every 10 ms
// the base moves, the OTOS is read into the pose history, and the LiDAR
// turns; every 20 ms the stick's command goes through the limiter with
// the newest scan, and the base is given what comes out. The base brakes
// at 0.6 m/s^2 and answers 60 ms late (BaseConfig); the limiter is told
// the brake, as the profile will tell it the measured one. No threads, so
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

}  // namespace

int main() {
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
