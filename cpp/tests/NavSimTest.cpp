// File        : NavSimTest.cpp
// Author      : Chris Pane
// Description : The simulator: its world, its base, its LiDAR against what
//               the real A1M8 was measured to do, and its OTOS.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The simulator is only worth having if its scans look like real ones, so
// most checks here hold a simulated scan against a measured figure from
// docs/rplidar-a1m8-findings.md: samples per revolution, the widest gap,
// the share of no-returns, the period. Then the geometry: a still robot in
// a square room sees the four walls where they are, with the LiDAR turned
// or not, and a turning robot's scan lands on the walls only when each
// point is placed with the pose at its own time (Scan::timeOf()). That is
// the skew deskewing will remove (docs/navigation-design.md §12.2). Last,
// the OTOS: its heading drifts at the rate asked, and the drift bends the
// position it reports as the base drives.
//
// The random numbers differ between standard libraries, so every check is
// a range or a statistic, never an exact value. Same style as the other
// tests: no framework.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <optional>
#include <vector>

#include "heron/geometry/Pose2.h"
#include "nav_sim/Base.h"
#include "nav_sim/Lidar.h"
#include "nav_sim/Otos.h"
#include "nav_sim/World.h"

namespace {

using namespace std::chrono_literals;
using namespace Heron::Geometry;
using Heron::Duration;
using Heron::Scan;
using Heron::TimePoint;
using Heron::NavSim::Base;
using Heron::NavSim::BaseConfig;
using Heron::NavSim::Lidar;
using Heron::NavSim::LidarConfig;
using Heron::NavSim::Otos;
using Heron::NavSim::OtosConfig;
using Heron::NavSim::World;

constexpr double kPi = std::numbers::pi;
constexpr double kRadToDeg = 180.0 / std::numbers::pi;
constexpr Duration kTick = 10ms;

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

double seconds(Duration d) {
  return std::chrono::duration<double>(d).count();
}

/** A 4 m square room centred on the origin, seen from inside. */
World squareRoom() {
  World world;
  world.addBox(Vec2{0.0, 0.0}, 4.0, 4.0);
  return world;
}

/** How far a point is from the nearest wall of squareRoom(). */
double offWall(Vec2 p) {
  return std::min(std::abs(std::abs(p.x) - 2.0), std::abs(std::abs(p.y) - 2.0));
}

/** Runs the base and the LiDAR together for `span`, collecting every scan
 *  and the true pose at every tick. */
struct Run {
  std::vector<Scan> scans;
  std::vector<std::pair<TimePoint, Pose2>> poses;
};

Run drive(Base &base, Lidar &lidar, TimePoint &now, Duration span) {
  Run run;
  const TimePoint end = now + span;
  run.poses.emplace_back(now, base.truePose());
  while (now < end) {
    now += kTick;
    base.advance(now);
    run.poses.emplace_back(now, base.truePose());
    if (auto scan = lidar.advance(now, base)) {
      run.scans.push_back(std::move(*scan));
    }
  }
  return run;
}

/** The true pose at t, from the run's ticks, blended between the two around it. */
Pose2 truePoseAt(const Run &run, TimePoint t) {
  const auto after = std::lower_bound(run.poses.begin(), run.poses.end(), t,
                                      [](const auto &p, TimePoint v) { return p.first < v; });
  if (after == run.poses.begin()) {
    return after->second;
  }
  if (after == run.poses.end()) {
    return run.poses.back().second;
  }
  const auto before = after - 1;
  const double f = seconds(t - before->first) / seconds(after->first - before->first);
  const Pose2 &a = before->second;
  const Pose2 &b = after->second;
  return Pose2{a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
               wrapAngle(a.heading + f * wrapAngle(b.heading - a.heading))};
}

/** Every Scan invariant (Scan.h), checked on one scan. */
bool keepsInvariants(const Scan &scan) {
  float previous = -Heron::kPi;
  for (const Heron::Point &p : scan.points) {
    if (p.bearingRad < -Heron::kPi || p.bearingRad >= Heron::kPi || !(p.rangeM > 0.0f) ||
        p.bearingRad < previous) {
      return false;
    }
    previous = p.bearingRad;
  }
  return scan.sweepStartRad >= -Heron::kPi && scan.sweepStartRad < Heron::kPi &&
         scan.tStart <= scan.tEnd;
}

}  // namespace

int main() {
  std::printf("The world\n");
  {
    World world = squareRoom();
    world.addBox(Vec2{1.2, 0.0}, 0.3, 0.3);
    check("four sides a box", world.segments.size() == 8);
    const auto ahead = world.cast(Vec2{0.0, 0.0}, 0.0, 12.0);
    check("straight at the box: its near face, 1.05 m", ahead && near(*ahead, 1.05, 1e-9));
    const auto left = world.cast(Vec2{0.0, 0.0}, kPi / 2.0, 12.0);
    check("to the left: the wall, 2 m", left && near(*left, 2.0, 1e-9));
    const auto corner = world.cast(Vec2{0.0, 0.0}, -3.0 * kPi / 4.0, 12.0);
    check("into the back-right corner: 2 sqrt 2", corner && near(*corner, 2.0 * std::sqrt(2.0), 1e-9));
    check("not far enough: no hit", !world.cast(Vec2{0.0, 0.0}, kPi / 2.0, 1.9).has_value());
    World empty;
    check("an empty world: no hit", !empty.cast(Vec2{}, 0.0, 12.0).has_value());
  }

  std::printf("The base\n");
  {
    TimePoint now{};
    Base base(BaseConfig{}, Pose2{}, now);
    base.command(Twist{0.25, 0.0, 0.0}, now);
    now += 50ms;
    base.advance(now);
    check("within the 60 ms lag: still", base.trueVelocity().vx == 0.0);
    now += 110ms;  // 160 ms: 100 ms of 0.6 m/s^2
    base.advance(now);
    check("speeding up at 0.6 m/s^2", near(base.trueVelocity().vx, 0.06, 1e-6));
    now += 840ms;  // 1 s
    base.advance(now);
    check("at the asked speed once reached", near(base.trueVelocity().vx, 0.25, 1e-9));
    // 60 ms still, 0.417 s speeding up (0.0521 m), then 0.523 s at 0.25.
    const double ramp = 0.25 / 0.6;
    const double expected = 0.5 * 0.25 * ramp + 0.25 * (1.0 - 0.06 - ramp);
    // Within 1 mm: the base steps every 1 ms, speed first, then position.
    check("the distance covered: lag, ramp, then steady",
          near(base.truePose().x, expected, 1e-3) && near(base.truePose().y, 0.0, 1e-9));
    base.command(Twist{}, now);
    now += 60ms + 420ms;
    base.advance(now);
    check("stopping takes as long as starting", base.trueVelocity().vx == 0.0);

    TimePoint t2{};
    Base spinner(BaseConfig{}, Pose2{1.0, 1.0, 0.0}, t2);
    spinner.command(Twist{0.0, 0.0, 1.0}, t2);
    t2 += 2s;
    spinner.advance(t2);
    // 60 ms lag, 1/3 s ramp at 3 rad/s^2, then 1 rad/s.
    const double turned = 0.5 * (1.0 / 3.0) + (2.0 - 0.06 - 1.0 / 3.0);
    check("spinning on the spot: turns, stays put",
          near(spinner.truePose().heading, turned, 1e-3) && near(spinner.truePose().x, 1.0, 1e-9) &&
              near(spinner.truePose().y, 1.0, 1e-9));
    check("a past time does nothing", [&] {
      const Pose2 before = spinner.truePose();
      spinner.advance(t2 - 1s);
      return spinner.truePose().heading == before.heading;
    }());
  }

  std::printf("The LiDAR, against the measured A1M8\n");
  {
    const World world = squareRoom();
    TimePoint now{};
    Base base(BaseConfig{}, Pose2{0.5, -0.3, 0.5}, now);
    Lidar lidar(world, LidarConfig{}, now);
    const Run run = drive(base, lidar, now, 30s);

    check("about 7.3 scans a second: 215 to 225 in 30 s",
          run.scans.size() >= 215 && run.scans.size() <= 225);
    bool invariants = true;
    bool consecutive = true;
    bool joined = true;
    double countSum = 0.0;
    std::uint32_t countMin = 1000000;
    std::uint32_t countMax = 0;
    double gapSum = 0.0;
    double gapMin = 360.0;
    double gapMax = 0.0;
    double invalidMin = 1.0;
    double invalidMax = 0.0;
    double periodMin = 1.0;
    double periodMax = 0.0;
    bool covered = true;
    for (std::size_t i = 0; i < run.scans.size(); ++i) {
      const Scan &s = run.scans[i];
      invariants = invariants && keepsInvariants(s);
      if (i > 0) {
        consecutive = consecutive && s.seq == run.scans[i - 1].seq + 1;
        joined = joined && s.tStart == run.scans[i - 1].tEnd;
      }
      countSum += s.rawSampleCount;
      countMin = std::min(countMin, s.rawSampleCount);
      countMax = std::max(countMax, s.rawSampleCount);
      gapSum += s.worstSamplingGapRad * kRadToDeg;
      gapMin = std::min(gapMin, s.worstSamplingGapRad * kRadToDeg);
      gapMax = std::max(gapMax, s.worstSamplingGapRad * kRadToDeg);
      invalidMin = std::min(invalidMin, static_cast<double>(s.invalidFraction()));
      invalidMax = std::max(invalidMax, static_cast<double>(s.invalidFraction()));
      periodMin = std::min(periodMin, seconds(s.duration()));
      periodMax = std::max(periodMax, seconds(s.duration()));
      covered = covered && s.isCoverageOk() && s.unobserved.empty() && s.droppedEstimate == 0;
    }
    const double countMean = countSum / static_cast<double>(run.scans.size());
    const double gapMean = gapSum / static_cast<double>(run.scans.size());
    std::printf("        samples %u to %u, mean %.1f; widest gap %.2f to %.2f deg, mean %.2f;\n"
                "        no-returns %.0f%% to %.0f%%; period %.1f to %.1f ms\n",
                countMin, countMax, countMean, gapMin, gapMax, gapMean, 100.0 * invalidMin,
                100.0 * invalidMax, 1000.0 * periodMin, 1000.0 * periodMax);

    check("every scan keeps the Scan invariants (built by the adapter's own code)", invariants);
    check("seq counts up by one, each scan starting where the last ended",
          consecutive && joined);
    // Measured: express 526 to 532 a revolution, mean 531. The simulated
    // period varies as the real one does, so the count with it.
    check("samples a revolution: mean 526 to 532 (measured mean 531)",
          countMean >= 526.0 && countMean <= 532.0);
    // Measured: each within 526 to 532 at 7.35 Hz, and 549 to 554 at 7.10 Hz
    // (2026-11-08), a spread of 0.75 samples. The count follows the period.
    check("samples a revolution: each within 526 to 536", countMin >= 526 && countMax <= 536);
    // Measured: the widest gap per revolution, 8.0 to 8.1 deg in express
    // (medians per capture), and 8.9 the widest of any healthy revolution.
    // Ours is a whole number of sample spacings, 11 or 12 (7.3 or 7.95).
    check("widest gap between samples: about 8 deg on average (7.6 to 8.4)",
          gapMean >= 7.6 && gapMean <= 8.4);
    check("widest gap between samples: never past 8.9 deg", gapMax <= 8.9);
    check("every revolution fully covered: no unobserved arc, nothing dropped", covered);
    // Measured: 61 to 84% of samples return nothing, indoors.
    check("no-returns 61 to 84% of samples", invalidMin >= 0.61 && invalidMax <= 0.84);
    // Measured: period 134.2 to 138.1 ms at 7.35 Hz; ours is 7.3 Hz, 137.0 ms.
    check("period 134 to 140 ms", periodMin >= 0.134 && periodMax <= 0.140);
    check("the same seed gives the same run", [&] {
      TimePoint t{};
      Base b(BaseConfig{}, Pose2{0.5, -0.3, 0.5}, t);
      Lidar again(world, LidarConfig{}, t);
      const Run second = drive(b, again, t, 1s);
      return !second.scans.empty() && second.scans[0].points.size() == run.scans[0].points.size() &&
             second.scans[0].points.front().rangeM == run.scans[0].points.front().rangeM;
    }());
  }

  std::printf("The LiDAR, still in a square room\n");
  for (const double yawDeg : {0.0, 90.0, -150.0}) {
    const World world = squareRoom();
    TimePoint now{};
    const Pose2 pose{0.5, -0.3, 30.0 * kDegToRad};
    Base base(BaseConfig{}, pose, now);
    LidarConfig config;
    config.yawRad = yawDeg * kDegToRad;
    config.offset = Vec2{0.06, 0.02};
    Lidar lidar(world, config, now);
    const Run run = drive(base, lidar, now, 2s);

    // Every point, placed by the design's two steps (§3.3), lands on a wall,
    // and every wall is seen.
    double worst = 0.0;
    std::int32_t onWall[4] = {0, 0, 0, 0};  // +x, +y, -x, -y
    std::size_t points = 0;
    for (const Scan &s : run.scans) {
      for (const Heron::Point &p : s.points) {
        const Vec2 world2 = toParent(pose, polarToRobot(p.bearingRad, p.rangeM, config.offset));
        worst = std::max(worst, offWall(world2));
        if (near(world2.x, 2.0, 0.05)) ++onWall[0];
        if (near(world2.y, 2.0, 0.05)) ++onWall[1];
        if (near(world2.x, -2.0, 0.05)) ++onWall[2];
        if (near(world2.y, -2.0, 0.05)) ++onWall[3];
        ++points;
      }
    }
    char what[160];
    std::snprintf(what, sizeof(what),
                  "LiDAR turned %+.0f deg: %zu points, each within 3 cm of a wall (worst %.1f mm)",
                  yawDeg, points, 1000.0 * worst);
    check(what, points > 0 && worst <= 0.03);
    std::snprintf(what, sizeof(what), "LiDAR turned %+.0f deg: all four walls seen (%d %d %d %d)",
                  yawDeg, onWall[0], onWall[1], onWall[2], onWall[3]);
    check(what, onWall[0] > 10 && onWall[1] > 10 && onWall[2] > 10 && onWall[3] > 10);
  }

  std::printf("The LiDAR, turning: the skew is real, and timeOf() undoes it\n");
  {
    const World world = squareRoom();
    TimePoint now{};
    Base base(BaseConfig{}, Pose2{0.3, 0.2, 0.0}, now);
    LidarConfig config;
    Lidar lidar(world, config, now);
    base.command(Twist{0.15, 0.0, 1.5}, now);
    drive(base, lidar, now, 2s);  // up to speed
    const Run run = drive(base, lidar, now, 2s);

    double worstAtEnd = 0.0;
    double worstAtOwnTime = 0.0;
    for (const Scan &s : run.scans) {
      if (s.tStart < run.poses.front().first) {
        continue;  // began before this run's poses: no true pose to place it with
      }
      const Pose2 atEnd = truePoseAt(run, s.tEnd);
      for (const Heron::Point &p : s.points) {
        const Vec2 inRobot = polarToRobot(p.bearingRad, p.rangeM, config.offset);
        const Pose2 own = truePoseAt(run, s.timeOf(p.bearingRad));
        worstAtEnd = std::max(worstAtEnd, offWall(toParent(atEnd, inRobot)));
        worstAtOwnTime = std::max(worstAtOwnTime, offWall(toParent(own, inRobot)));
      }
    }
    std::printf("        worst off a wall: %.0f mm placed at the scan's end, %.1f mm at "
                "each point's own time\n",
                1000.0 * worstAtEnd, 1000.0 * worstAtOwnTime);
    // At 1.5 rad/s a revolution turns the robot 12 deg: a point 2 m away
    // placed with the wrong heading lands up to 40 cm off.
    check("placed with the pose at the scan's end: off the walls by over 20 cm",
          worstAtEnd > 0.20);
    check("placed with the pose at its own time (timeOf): within 3 cm", worstAtOwnTime <= 0.03);
  }

  std::printf("The OTOS: drift and noise\n");
  {
    // Still for a minute: the heading drifts by the rate, the position not.
    TimePoint now{};
    Base base(BaseConfig{}, Pose2{1.0, 2.0, 0.3}, now);
    Otos otos(OtosConfig{0.6, 0.0, 1}, now);
    const Pose2 first = otos.read(now, base).pose;
    check("the first reading is the true pose", near(first.x, 1.0, 1e-12) &&
                                                    near(first.y, 2.0, 1e-12) &&
                                                    near(first.heading, 0.3, 1e-12));
    Pose2 last{};
    for (std::int32_t i = 0; i < 6000; ++i) {
      now += kTick;
      base.advance(now);
      last = otos.read(now, base).pose;
    }
    const double driftDeg = wrapAngle(last.heading - 0.3) * kRadToDeg;
    std::printf("        still for 60 s at 0.6 deg/min: heading off by %.3f deg\n", driftDeg);
    check("still for a minute: the heading off by 0.6 deg", near(driftDeg, 0.6, 1e-6));
    check("  and the position not moved", near(last.x, 1.0, 1e-9) && near(last.y, 2.0, 1e-9));

    // Driving 2 m straight along x while the heading drifts 6 deg/min: the
    // believed heading ends 0.8 deg out, and the path bends to the left by
    // about the mean angle times the distance, 0.4 deg x 2 m = 14 mm.
    now = TimePoint{};
    Base driver(BaseConfig{1000.0, 1000.0, Duration::zero()}, Pose2{}, now);
    Otos drifting(OtosConfig{6.0, 0.0, 1}, now);
    (void)drifting.read(now, driver);
    driver.command(Twist{0.25, 0.0, 0.0}, now);
    for (std::int32_t i = 0; i < 800; ++i) {
      now += kTick;
      driver.advance(now);
      last = drifting.read(now, driver).pose;
    }
    std::printf("        2 m straight at 6 deg/min: believed (%.4f, %.4f), true (%.4f, %.4f)\n",
                last.x, last.y, driver.truePose().x, driver.truePose().y);
    check("driving 2 m: the drift bends the believed path left, about 14 mm",
          near(last.y, 0.014, 0.002) && near(driver.truePose().y, 0.0, 1e-9));

    // The noise: per reading, not accumulated.
    now = TimePoint{};
    Base still(BaseConfig{}, Pose2{}, now);
    Otos noisy(OtosConfig{0.0, 0.001, 7}, now);
    double sum = 0.0;
    double sumSq = 0.0;
    double worst = 0.0;
    for (std::int32_t i = 0; i < 2000; ++i) {
      now += kTick;
      still.advance(now);
      const double x = noisy.read(now, still).pose.x;
      sum += x;
      sumSq += x * x;
      worst = std::max(worst, std::abs(x));
    }
    const double mean = sum / 2000.0;
    const double sd = std::sqrt(sumSq / 2000.0 - mean * mean);
    std::printf("        noise on x: mean %.2f mm, sd %.2f mm, worst %.2f mm\n", 1000.0 * mean,
                1000.0 * sd, 1000.0 * worst);
    check("noise 1 mm sd, centred, not accumulated",
          near(sd, 0.001, 0.0001) && near(mean, 0.0, 0.0001) && worst < 0.006);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
