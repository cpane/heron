// File        : RobotTest.cpp
// Author      : Chris Pane
// Description : heron_robot's decisions: Manual is heron_teleop, the mode
//               switch, the units, each library's settings from the bench
//               rig's profile, and Assisted through the limiter.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The N4 row of docs/navigation-cpp-design.md §9 for robot_test:
//
//   - Manual gives heron_teleop's wheel values for the same frames (D5):
//     5000 random frames, arming, link and throttle included, through
//     teleop's own calls and through decide(), with a wall ahead that
//     Assisted would stop for. They must match exactly.
//   - SwC as a mode, either end up; the unit conversions and their round
//     trip; the settings built from profiles/bench.conf, as measured in N1.
//   - Assisted refused without its sections, or with the blind spot
//     uncovered; Assisted with a wall ahead cuts a forward stick, keeps a
//     backward one, and stops with no scan.
//   - Navigate: the planner's and the map's settings from the profile; the
//     plan's command driven only for this course and fresh, the turn the
//     stick's; through decide() exactly as Assisted would drive it; BLOCKED
//     after 0.5 s of the limiter holding a plan back.
//
// The bench profile is read from the repository on the Mac
// (HERON_PROFILE_DIR) and from ~/heron/profiles on a Pi, where
// deploy_cpp.sh puts it. Same style as the other tests: no framework.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "heron/drive/Drive.h"
#include "heron/limiter/Limiter.h"
#include "heron/profile/RigProfile.h"
#include "robot/RobotCore.h"

namespace {

using namespace std::chrono_literals;
using namespace Heron::Robot;
using Heron::Scan;
using Heron::TimePoint;
using Heron::Drive::BodyVelocity;
using Heron::Drive::WheelValues;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Pose::PoseHistory;
using Heron::Pose::PoseSample;
using Heron::Remote::Switch3;

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

bool same(const BodyVelocity &a, const BodyVelocity &b, double tolerance = 1e-9) {
  return near(a.vx, b.vx, tolerance) && near(a.vy, b.vy, tolerance) &&
         near(a.wz, b.wz, tolerance);
}

bool stopped(const WheelValues &w) {
  return w[0] == 0.0 && w[1] == 0.0 && w[2] == 0.0 && w[3] == 0.0;
}

std::filesystem::path benchProfile() {
#ifdef HERON_PROFILE_DIR
  const std::filesystem::path repo = std::filesystem::path(HERON_PROFILE_DIR) / "bench.conf";
  if (std::filesystem::exists(repo)) {
    return repo;
  }
#endif
  return Heron::Profile::heronHome() / "profiles" / "bench.conf";
}

// A point where the robot frame says, as the LiDAR at `lidar` reports it
// (bearings in the robot frame, the mounting already applied by the scan
// builder: LimiterTest.cpp's helper).
Heron::Point seen(Vec2 p, Vec2 lidar) {
  const Vec2 d = p - lidar;
  return Heron::Point{static_cast<float>(std::atan2(d.y, d.x)),
                      static_cast<float>(Heron::Geometry::norm(d))};
}

// A healthy scan of a wall across x = `at`, 1 cm points, ending at `end`.
Scan wallAhead(double at, Vec2 lidar, TimePoint end) {
  Scan scan;
  scan.seq = 1;
  scan.tEnd = end;
  scan.tStart = end - 133ms;
  scan.sweepStartRad = 3.0f;
  scan.rawSampleCount = 520;
  scan.worstSamplingGapRad = 8.0f * static_cast<float>(kPi / 180.0);
  for (std::int32_t i = -50; i <= 50; ++i) {
    scan.points.push_back(seen(Vec2{at, 0.01 * i}, lidar));
  }
  std::sort(scan.points.begin(), scan.points.end(),
            [](const Heron::Point &a, const Heron::Point &b) { return a.bearingRad < b.bearingRad; });
  return scan;
}

// Heron standing at the origin from well before to after t.
PoseHistory still(TimePoint t) {
  PoseHistory history;
  history.add(PoseSample{t - 1s, Pose2{}, Twist{}});
  history.add(PoseSample{t + 10ms, Pose2{}, Twist{}});
  return history;
}

// A frame with every channel at rest, then the sticks and throttle set.
Heron::Remote::IBusFrame frameOf(std::uint16_t forward, std::uint16_t side, std::uint16_t turn,
                                 std::uint16_t throttle) {
  Heron::Remote::IBusFrame frame;
  frame.channels.fill(1500);
  const Heron::Drive::StickConfig sticks;
  frame.channels[sticks.forwardChannel] = forward;
  frame.channels[sticks.sideChannel] = side;
  frame.channels[sticks.turnChannel] = turn;
  frame.channels[sticks.throttleChannel] = throttle;
  return frame;
}

}  // namespace

int main() {
  const Heron::Profile::RigProfile bench = Heron::Profile::loadRigProfile(benchProfile());
  std::printf("Profile: %s\n", bench.source.c_str());
  const Heron::Profile::GeometryProfile geometry = bench.geometry.value_or(
      Heron::Profile::GeometryProfile{});

  std::printf("SwC as a mode (design §9; which end is up: §18 item 4)\n");
  {
    check("middle: Assisted, either way up",
          modeFromSwitch(Switch3::Mid, false) == Mode::Assisted &&
              modeFromSwitch(Switch3::Mid, true) == Mode::Assisted);
    check("Manual low: low Manual, high Navigate",
          modeFromSwitch(Switch3::Low, false) == Mode::Manual &&
              modeFromSwitch(Switch3::High, false) == Mode::Navigate);
    check("Manual high: high Manual, low Navigate",
          modeFromSwitch(Switch3::High, true) == Mode::Manual &&
              modeFromSwitch(Switch3::Low, true) == Mode::Navigate);
    check("a channel that is not there (raw 0) reads Low",
          Heron::Remote::decodeSwitch3(0) == Switch3::Low);
  }

  std::printf("Units (I2), the bench rig's [geometry]\n");
  {
    const Twist full = toTwist(BodyVelocity{1.0, 1.0, 1.0}, geometry);
    check("full forward and sideways: max_speed 0.42 m/s",
          near(full.vx, 0.42, 1e-12) && near(full.vy, 0.42, 1e-12));
    check("full turn: max_turn_rate 3.4 rad/s", near(full.wz, 3.4, 1e-12));
    bool roundTrip = true;
    for (double vx = -1.0; vx <= 1.0; vx += 0.25) {
      for (double wz = -1.0; wz <= 1.0; wz += 0.25) {
        const BodyVelocity v{vx, -vx / 2.0, wz};
        roundTrip = roundTrip && same(toBodyVelocity(toTwist(v, geometry), geometry), v, 1e-12);
      }
    }
    check("fractions to SI and back: unchanged", roundTrip);
    check("faster than full: held to 1",
          same(toBodyVelocity(Twist{1.0, -1.0, 10.0}, geometry), BodyVelocity{1.0, -1.0, 1.0}));

    // The dead band: the least that moves is min_speed 0.16 m/s.
    const Twist least = toTwist(BodyVelocity{0.001, -0.001, 0.001}, geometry);
    check("the smallest command: min_speed, not in proportion",
          near(least.vx, 0.16026, 1e-9) && near(least.vy, -0.16026, 1e-9) &&
              near(least.wz, 3.4 * 0.16 / 0.42 + (3.4 - 3.4 * 0.16 / 0.42) * 0.001, 1e-9));
    check("  a fifth of full: 0.212 m/s (measured 0.19)",
          near(toTwist(BodyVelocity{0.2, 0.0, 0.0}, geometry).vx, 0.212, 1e-12));
    check("under min_speed: stopped",
          same(toBodyVelocity(Twist{0.10, -0.10, 0.5}, geometry), BodyVelocity{}));
    check("  or, asked to, the least that moves, signs kept",
          same(toBodyVelocity(Twist{0.10, -0.10, 0.5}, geometry, true),
               BodyVelocity{0.001, -0.001, 0.001}));
    check("  min_speed exactly: the least that moves",
          same(toBodyVelocity(Twist{0.16, 0.0, 0.0}, geometry), BodyVelocity{0.001, 0.0, 0.0}));
    Heron::Profile::GeometryProfile plain = geometry;
    plain.minSpeed = 0.0;
    check("min_speed 0: in proportion, as before",
          near(toTwist(BodyVelocity{0.2, 0.0, 0.2}, plain).vx, 0.084, 1e-12) &&
              same(toBodyVelocity(Twist{0.084, 0.0, 0.0}, plain), BodyVelocity{0.2, 0.0, 0.0}));
  }

  std::printf("Each library's settings from the profile (I6)\n");
  {
    check("the bench profile has all four sections",
          bench.geometry && bench.lidar && bench.odometry && bench.navigation);
    const Heron::Lidar::LidarConfig lidar = toLidarConfig(*bench.lidar);
    check("LiDAR: housing yaw 180 is a mounting offset of pi",
          near(lidar.mountingOffsetRad, kPi, 1e-6));
    check("  the port and mode stay the defaults",
          lidar.port == "/dev/rplidar" && lidar.scanMode == 1);
    const Heron::Odometry::OtosConfig otos = toOtosConfig(*bench.odometry);
    check("OTOS: scalars 0.908 and 0.991",
          near(otos.linearScalar, 0.908, 1e-12) && near(otos.angularScalar, 0.991, 1e-12));
    check("  at the centre, facing forward",
          otos.offset.x == 0.0 && otos.offset.y == 0.0 && otos.offset.h == 0.0);
    const Heron::Limiter::LimiterConfig limiter = toLimiterConfig(bench);
    check("limiter: the outline halved, 0.09525 each way",
          near(limiter.outline.halfLength, 0.09525, 1e-12) &&
              near(limiter.outline.halfWidth, 0.09525, 1e-12));
    check("  the LiDAR 8 mm behind the centre",
          near(limiter.lidarOffset.x, -0.008, 1e-12) && limiter.lidarOffset.y == 0.0);
    check("  margins 0.05 and 0.25, brake 0.7",
          near(limiter.stopMarginM, 0.05, 1e-12) && near(limiter.slowMarginM, 0.25, 1e-12) &&
              near(limiter.brakeMps2, 0.7, 1e-12));
    const Heron::Perception::PerceptionConfig map = toPerceptionConfig(*bench.lidar);
    check("map: the LiDAR 8 mm behind the centre",
          near(map.mount.offset.x, -0.008, 1e-12) && map.mount.offset.y == 0.0);
    const auto table = std::make_shared<const Heron::Motion::MotionTable>(
        *toMotionTable(Heron::Profile::onSurface(bench, bench.surface)));
    const Heron::Planner::PlannerConfig planner = toPlannerConfig(bench, table);
    check("planner: the outline halved, stop margin 0.05",
          near(planner.outline.halfLength, 0.09525, 1e-12) && near(planner.outline.halfWidth, 0.09525, 1e-12) &&
              near(planner.stopMarginM, 0.05, 1e-12));
    check("  max_speed 0.42, accel 0.6, the table given (P15)",
          near(planner.maxSpeedMps, 0.42, 1e-12) && near(planner.accelMps2, 0.6, 1e-12) &&
              planner.motion == table);
    Heron::Profile::RigProfile noNav = bench;
    noNav.navigation.reset();
    bool threw = false;
    try {
      (void)toPlannerConfig(noNav, nullptr);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check("  without [navigation]: throws", threw);
  }

  std::printf("Assisted refused (§7.1)\n");
  {
    check("the bench rig: ready", assistedRefusal(bench).empty());
    Heron::Profile::RigProfile bare = bench;
    bare.odometry.reset();
    bare.navigation.reset();
    const std::string why = assistedRefusal(bare);
    std::printf("        %s\n", why.c_str());
    check("without [odometry] and [navigation]: refused, naming both",
          why.find("[odometry], [navigation]") != std::string::npos);
    Heron::Profile::RigProfile outside = bench;
    outside.lidar->x = -0.08;  // the blind circle reaches 0.207 m behind
    const std::string blind = assistedRefusal(outside);
    check("a LiDAR whose blind circle leaves the stop outline: refused",
          blind.find("blind circle") != std::string::npos);
    bare.lidar.reset();
    bool threw = false;
    try {
      (void)toLimiterConfig(bare);
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check("toLimiterConfig() without its sections: throws", threw);
  }

  // The bench rig's own limiter, a wall ahead, Heron still.
  const Heron::Limiter::Limiter limiter(toLimiterConfig(bench));
  const Vec2 lidarAt{bench.lidar->x, bench.lidar->y};
  const TimePoint t0{100s};
  const PoseHistory history = still(t0);
  const Scan close = wallAhead(0.30, lidarAt, t0);
  const Scan far = wallAhead(3.0, lidarAt, t0);
  const auto assistedWith = [&](const Scan *scan) {
    return AssistedInput{&limiter, geometry, nullptr,
                         Heron::Limiter::LimitInput{Twist{}, scan, &history, true, Twist{}, t0 + 20ms}, std::nullopt, false,
                         std::nullopt, {}};
  };

  std::printf("Manual is heron_teleop (D5)\n");
  {
    const Heron::Drive::StickConfig sticks;
    Heron::Drive::ArmGate teleopGate;
    Heron::Drive::ArmGate robotGate;
    const AssistedInput wall = assistedWith(&close);
    std::mt19937 random(7);
    std::uniform_int_distribution<std::int32_t> raw(980, 2020);
    std::uniform_int_distribution<std::int32_t> percent(0, 99);
    std::int32_t mismatches = 0;
    std::int32_t driven = 0;
    bool remoteOk = false;
    std::uint16_t throttle = 1000;
    for (std::int32_t i = 0; i < 5000; ++i) {
      if (percent(random) < 2) {
        remoteOk = !remoteOk;  // disarm, rearm, radio lost
      }
      if (percent(random) < 5) {
        throttle = static_cast<std::uint16_t>(percent(random) < 30 ? 1000 : raw(random));
      }
      const auto frame = frameOf(static_cast<std::uint16_t>(raw(random)),
                                 static_cast<std::uint16_t>(raw(random)),
                                 static_cast<std::uint16_t>(raw(random)), throttle);
      const double level = Heron::Drive::readThrottle(frame, sticks);
      const BodyVelocity stick = Heron::Drive::sticksToVelocity(frame, sticks);

      // heron_teleop's loop body, as written there.
      WheelValues teleop{};
      if (teleopGate.update(remoteOk, level)) {
        teleop = Heron::Drive::mix(stick);
      }
      const Decision robot = decide(Mode::Manual, robotGate.update(remoteOk, level), stick, &wall);
      if (robot.wheels != teleop) {
        ++mismatches;
      }
      if (!stopped(teleop)) {
        ++driven;
      }
    }
    std::printf("        5000 frames, %d driving, %d different\n", driven, mismatches);
    check("the same wheels, frame by frame", mismatches == 0);
    check("  and the sequence drove as well as stopped", driven > 500 && driven < 4500);
    const Decision d = decide(Mode::Manual, true, BodyVelocity{1.0, 0.0, 0.0}, &wall);
    check("full forward at a wall 0.3 m ahead: driven, no limiter",
          d.why == Why::Manual && same(d.body, BodyVelocity{1.0, 0.0, 0.0}) && !d.limited);
  }

  std::printf("Assisted through the bench rig's limiter\n");
  {
    const AssistedInput wall = assistedWith(&close);
    Decision d = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &wall);
    std::printf("        wall at 0.30 m, full forward: vx %.3f of full, %s\n", d.body.vx,
                d.limited ? Heron::Limiter::toString(d.limited->reason) : "-");
    check("wall at 0.30 m, full forward: slowed or cut, not stopped",
          d.why == Why::Limited && d.body.vx > 0.0 && d.body.vx < 0.9);
    check("  the wheels are the cut motion, mixed",
          d.wheels == Heron::Drive::mix(d.body));
    d = decide(Mode::Assisted, true, BodyVelocity{-1.0, 0.0, 0.0}, &wall);
    check("  full backward, away from it: unchanged",
          same(d.body, BodyVelocity{-1.0, 0.0, 0.0}, 1e-12));
    const AssistedInput open = assistedWith(&far);
    d = decide(Mode::Assisted, true, BodyVelocity{0.5, 0.2, 0.1}, &open);
    check("wall at 3 m: unchanged, Clear",
          same(d.body, BodyVelocity{0.5, 0.2, 0.1}, 1e-12) &&
              d.limited->reason == Heron::Limiter::Reason::Clear);
    // Under min_speed: the slow zone alone goes at the least that moves; a
    // cut stops (header, "Units").
    d = decide(Mode::Assisted, true, BodyVelocity{0.05, 0.0, 0.0}, &wall);
    std::printf("        wall at 0.30 m, 0.05 forward: vx %.3f of full, %s\n", d.body.vx,
                Heron::Limiter::toString(d.limited->reason));
    check("  slowed under min_speed: the least that moves",
          d.limited->reason == Heron::Limiter::Reason::Slowed && d.limited->command.vx < 0.16 &&
              near(d.body.vx, 0.001, 1e-12));
    const Scan veryClose = wallAhead(0.17, lidarAt, t0);
    const AssistedInput nearWall = assistedWith(&veryClose);
    d = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &nearWall);
    std::printf("        wall at 0.17 m, full forward: %.3f m/s allowed, vx %.3f of full, %s\n",
                d.limited->command.vx, d.body.vx, Heron::Limiter::toString(d.limited->reason));
    check("  cut under min_speed: stopped",
          d.limited->reason == Heron::Limiter::Reason::Cut && d.limited->command.vx > 0.0 &&
              d.limited->command.vx < 0.16 && d.body.vx == 0.0);
    const AssistedInput blind = assistedWith(nullptr);
    d = decide(Mode::Assisted, true, BodyVelocity{0.5, 0.0, 0.0}, &blind);
    check("no scan: stopped, the limiter's NoScan",
          stopped(d.wheels) && d.limited->reason == Heron::Limiter::Reason::NoScan);
    d = decide(Mode::Assisted, true, BodyVelocity{0.5, 0.0, 0.0}, nullptr);
    check("not ready: stopped, AssistedUnavailable, never Manual",
          stopped(d.wheels) && d.why == Why::AssistedUnavailable);
  }

  std::printf("On a surface, through its motion table (design §19)\n");
  {
    const Heron::Profile::RigProfile hw = Heron::Profile::onSurface(bench, bench.surface);
    check("bench.conf's default surface: hardwood-1, its numbers in place",
          bench.surface == "hardwood-1" && hw.activeSurface == "hardwood-1" &&
              near(hw.geometry->brake, 0.7, 1e-12) && near(hw.odometry->linearScalar, 0.908, 1e-12));
    const std::optional<Heron::Motion::MotionTable> table = toMotionTable(hw);
    check("  its motion table: 32 points in 8 columns", table && table->columns() == 8);
    check("  Assisted not refused", assistedRefusal(hw).empty());
    check("no surface: no table", !toMotionTable(bench));
    Heron::Profile::RigProfile bare = bench;
    bare.surfaces.push_back(Heron::Profile::SurfaceProfile{});
    bare.surfaces.back().name = "bare";
    bare = Heron::Profile::onSurface(bare, "bare");
    check("a surface with no table: Assisted refused, says so",
          assistedRefusal(bare) == "surface bare has no motion table");
    check("the pose's plausibility: 1.5 x max_speed, the jump check off",
          near(toPoseConfig(hw).maxPlausibleSpeedMps, 0.63, 1e-12) &&
              toPoseConfig(hw).maxPlausibleAccelMps2 == 0.0);

    const auto through = [&](const Scan *scan, bool fresh) {
      return AssistedInput{&limiter, hw.geometry.value(), &*table,
                           Heron::Limiter::LimitInput{Twist{}, scan, &history, fresh, Twist{}, t0 + 20ms}, std::nullopt,
                           false, std::nullopt, {}};
    };
    const AssistedInput open = through(&far, true);
    Decision d = decide(Mode::Assisted, true, BodyVelocity{0.6, 0.0, 0.0}, &open);
    check("wall at 3 m, forward 0.6: Clear, the stick's command as it is",
          d.why == Why::Limited && d.limited->reason == Heron::Limiter::Reason::Clear &&
              same(d.body, BodyVelocity{0.6, 0.0, 0.0}, 1e-12));
    const Twist told = d.limited->command;
    check("  the limiter told 0.36 x 1.2 m/s forward, the drift's turn included",
          near(told.vx, 0.36 * kTableMargin, 1e-9) && near(told.wz, 0.07, 1e-9));
    const AssistedInput wall = through(&close, true);
    d = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &wall);
    const Twist made = table->predict(d.body.vx, d.body.vy);
    std::printf("        wall at 0.30 m, full forward: %s, allowed %.3f m/s, command (%.3f, %.3f) makes %.3f\n",
                Heron::Limiter::toString(d.limited->reason), d.limited->command.vx / kTableMargin,
                d.body.vx, d.body.vy, made.vx);
    check("wall at 0.30 m, full forward: a smaller command, still moving, making no more than allowed",
          d.body.vx > 0.0 && d.body.vx < 1.0 && made.vx <= d.limited->command.vx / kTableMargin + 0.005);
    // That command seen not to start the robot (StallWatch): standing, it
    // is passed over; rolling, the table's word stands.
    const BodyVelocity substitute = d.body;
    const std::vector<Heron::Motion::DeadCommand> deadOne{
        Heron::Motion::DeadCommand{std::atan2(substitute.vy, substitute.vx), std::hypot(substitute.vx, substitute.vy)}};
    AssistedInput wallDead = through(&close, true);
    wallDead.dead = deadOne;
    d = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &wallDead);
    std::printf("        that command dead, standing: (%.3f, %.3f)\n", d.body.vx, d.body.vy);
    check("  that command dead, standing: not it nor smaller near it (another, or stopped)",
          !Heron::Motion::isDead(deadOne, d.body.vx, d.body.vy));
    AssistedInput wallRolling = through(&close, true);
    wallRolling.limit.measured = Twist{0.30, 0.0, 0.0};
    const BodyVelocity rolling = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &wallRolling).body;
    wallDead.limit.measured = wallRolling.limit.measured;
    d = decide(Mode::Assisted, true, BodyVelocity{1.0, 0.0, 0.0}, &wallDead);
    check("  rolling at 0.30 m/s: the same command as with none dead", same(d.body, rolling, 1e-12));
    // Check 1 (2026-11-14): stick 0.58 forward toward a box, the limiter
    // cutting. A diagonal stood in for forward (vy -0.19 of full) and the
    // rolling robot stopped 8 cm short. Now: forward-ish or stopped.
    bool forwardOrStopped = true;
    bool someStopped = false;
    for (double at = 0.20; at <= 0.60; at += 0.01) {
      const Scan box = wallAhead(at, lidarAt, t0);
      const AssistedInput toBox = through(&box, true);
      d = decide(Mode::Assisted, true, BodyVelocity{0.58, 0.0, 0.0}, &toBox);
      const bool zero = d.body.vx == 0.0 && d.body.vy == 0.0;
      someStopped = someStopped || zero;
      forwardOrStopped = forwardOrStopped &&
                         (zero || (d.body.vx > 0.0 && std::abs(std::atan2(d.body.vy, d.body.vx)) <=
                                                         15.0 * Heron::Geometry::kDegToRad + 1e-9));
      if (!zero && std::abs(d.body.vy) > 0.05) {
        std::printf("        box at %.2f m: command (%.3f, %.3f), %s\n", at, d.body.vx, d.body.vy,
                    Heron::Limiter::toString(d.limited->reason));
      }
    }
    check("check 1, box 0.20 to 0.60 m ahead, stick 0.58: forward within 15 degrees, or stopped",
          forwardOrStopped);
    check("  and near the box, stopped", someStopped);
    std::printf("Heading hold (P14, I20)\n");
    {
      const auto &g = hw.geometry.value();
      HeadingHold hold;
      check("standing still: no turn, nothing held",
            hold.update(0.0, false, 0.3, 0.0, 1) == 0.0 && !hold.holding());
      double rate = hold.update(0.0, true, 0.3, -0.2, 1);
      check("translating: holds the heading it began at, and cancels the drift (+0.2 for -0.2)",
            hold.holding() && near(hold.targetRad(), 0.3, 1e-12) && near(rate, 0.2, 1e-12));
      rate = hold.update(0.0, true, 0.2, 0.0, 1);
      check("turned 0.1 rad clockwise: turns back anticlockwise, 2/s x 0.1", near(rate, 0.2, 1e-12));
      rate = hold.update(0.0, true, 0.3 - 1.0, 0.0, 1);
      check("a large error: held to 0.8 rad/s", near(rate, 0.8, 1e-12));
      rate = hold.update(0.5, true, -0.5, 0.0, 1);
      check("the turn stick overrides: its rate, nothing held", rate == 0.5 && !hold.holding());
      (void)hold.update(0.0, true, -0.5, 0.0, 1);
      check("  let go: the new heading is held", hold.holding() && near(hold.targetRad(), -0.5, 1e-12));
      (void)hold.update(0.0, true, 0.1, 0.0, 2);
      check("a new pose epoch: the heading taken again in the new frame", near(hold.targetRad(), 0.1, 1e-12));
      hold.reset();
      check("reset: nothing held", !hold.holding());
      (void)hold.update(0.0, true, 3.1, 0.0, 2);
      rate = hold.update(0.0, true, -3.1, 0.0, 2);
      check("across +-pi: the short way round", near(rate, 2.0 * (3.1 - (2.0 * kPi - 3.1)), 1e-9) &&
                                                  rate < 0.0);

      const double perUnit = g.maxTurnRate * (1.0 - g.minSpeed / g.maxSpeed);
      check("holdFraction, a full strafe: rate / (max_turn_rate x (1 - min/max))",
            near(holdFraction(0.4, BodyVelocity{0.0, -1.0, 0.0}, g), 0.4 / perUnit, 1e-12));
      check("  held to half the slowest wheel (0.1 forward: 0.05)",
            near(holdFraction(0.8, BodyVelocity{0.1, 0.0, 0.0}, g), 0.05, 1e-12));
      check("  no translation, or a 45 degree diagonal: nothing",
            holdFraction(0.4, BodyVelocity{}, g) == 0.0 &&
                holdFraction(0.4, BodyVelocity{0.5, 0.5, 0.0}, g) == 0.0);

      AssistedInput held = open;
      held.holdRadps = 0.3;
      d = decide(Mode::Assisted, true, BodyVelocity{0.0, -1.0, 0.0}, &held);
      check("Assisted, clear, full strafe with the hold at 0.3 rad/s: the stick's translation, the hold's turn",
            d.limited->reason == Heron::Limiter::Reason::Clear && d.body.vx == 0.0 && d.body.vy == -1.0 &&
                near(d.body.wz, holdFraction(0.3, BodyVelocity{0.0, -1.0, 0.0}, g), 1e-12) && d.body.wz > 0.0);
      check("  the limiter told the hold's turn plus the table's drift",
            near(d.limited->command.wz, 0.3 + table->predict(0.0, -1.0).wz, 1e-9));
      const Scan box = wallAhead(0.20, lidarAt, t0);
      AssistedInput heldToBox = through(&box, true);
      heldToBox.holdRadps = 0.5;
      d = decide(Mode::Assisted, true, BodyVelocity{0.58, 0.0, 0.0}, &heldToBox);
      check("translation cut to nothing: no turn either, so it never spins at the turn's floor",
            d.body.vx == 0.0 && d.body.vy == 0.0 && d.body.wz == 0.0 && stopped(d.wheels));
      AssistedInput heldBare = assistedWith(&far);
      heldBare.holdRadps = 0.3;
      d = decide(Mode::Assisted, true, BodyVelocity{0.5, 0.0, 0.0}, &heldBare);
      check("no table: the hold's turn as well",
            near(d.body.wz, holdFraction(0.3, d.body, heldBare.geometry), 1e-12) && d.body.wz > 0.0);
    }

    std::printf("While cutting, the command only falls (CutHold, 2026-11-16)\n");
    {
      const auto limitedBy = [](Heron::Limiter::Reason reason, BodyVelocity body) {
        Decision out;
        out.body = body;
        out.why = Why::Limited;
        out.limited = Heron::Limiter::LimitResult{};
        out.limited->reason = reason;
        return out;
      };
      const BodyVelocity forward{1.0, 0.0, 0.0};
      CutHold cut;
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Slowed, BodyVelocity{0.6, 0.0, 0.0}));
      check("Slowed alone: no hold", !cut.holding() && !cut.ceiling(forward));
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Cut, BodyVelocity{0.75, 0.0, 0.0}));
      check("the first Cut: holds, last cycle's translation the ceiling",
            cut.holding() && cut.ceiling(forward) && same(*cut.ceiling(forward), BodyVelocity{0.75, 0.0, 0.0}, 1e-12));
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Slowed, BodyVelocity{0.5, 0.0, 0.0}));
      check("  Slowed after it: still holds, the new ceiling", cut.holding() && near(cut.ceiling(forward)->vx, 0.5, 1e-12));
      check("  a stick 10 degrees off: the same ceiling",
            cut.ceiling(BodyVelocity{std::cos(0.17), std::sin(0.17), 0.0}).has_value());
      check("  pulled back (180 degrees): no ceiling", !cut.ceiling(BodyVelocity{-0.5, 0.0, 0.0}));
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Clear, BodyVelocity{0.5, 0.0, 0.0}));
      check("one Clear between Cuts: still holds", cut.holding());
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Cut, BodyVelocity{0.5, 0.0, 0.0}));
      for (int i = 0; i < CutHold::kReleaseCycles - 1; ++i) {
        cut.update(forward, limitedBy(Heron::Limiter::Reason::Clear, BodyVelocity{0.5, 0.0, 0.0}));
      }
      check("  Clear 4 cycles in a row: still holds", cut.holding());
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Clear, BodyVelocity{0.5, 0.0, 0.0}));
      check("  the 5th (0.1 s): let go", !cut.holding());
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Cut, BodyVelocity{0.4, 0.0, 0.0}));
      cut.update(BodyVelocity{}, limitedBy(Heron::Limiter::Reason::Cut, BodyVelocity{}));
      check("the translation stick centred: let go", !cut.holding());
      cut.update(forward, limitedBy(Heron::Limiter::Reason::Cut, BodyVelocity{0.4, 0.0, 0.0}));
      cut.update(BodyVelocity{0.0, -1.0, 0.0}, limitedBy(Heron::Limiter::Reason::Slowed, BodyVelocity{0.0, -0.5, 0.0}));
      check("turned 90 degrees and only Slowed: let go", !cut.holding());

      const Scan box = wallAhead(0.45, lidarAt, t0);
      AssistedInput toBox = through(&box, true);
      toBox.limit.measured = Twist{0.36, 0.0, 0.0};
      const Decision free = decide(Mode::Assisted, true, forward, &toBox);
      toBox.cutCeiling = BodyVelocity{0.0, 0.0, 0.0};
      d = decide(Mode::Assisted, true, forward, &toBox);
      check("box at 0.45 m, the ceiling nothing: stopped, still LIMITED",
            stopped(d.wheels) && d.why == Why::Limited);
      toBox.cutCeiling = BodyVelocity{0.25, 0.0, 0.0};
      d = decide(Mode::Assisted, true, forward, &toBox);
      const auto speedOf = [&](const BodyVelocity &b) {
        const Twist m = table->predict(b.vx, b.vy);
        return std::hypot(m.vx, m.vy);
      };
      std::printf("        box at 0.45 m doing 0.36: %s, free (%.2f, %.2f) makes %.3f; ceiling 0.25 gives (%.2f, %.2f)\n",
                  Heron::Limiter::toString(free.limited->reason), free.body.vx, free.body.vy, speedOf(free.body),
                  d.body.vx, d.body.vy);
      check("  a ceiling below what it would drive: the ceiling's command, or nothing",
            speedOf(d.body) <= speedOf(BodyVelocity{0.25, 0.0, 0.0}) + 1e-9);
      AssistedInput openCeiling = through(&far, true);
      openCeiling.cutCeiling = BodyVelocity{0.5, 0.0, 0.0};
      d = decide(Mode::Assisted, true, forward, &openCeiling);
      check("Clear while holding: held to the ceiling too",
            d.limited->reason == Heron::Limiter::Reason::Clear && same(d.body, BodyVelocity{0.5, 0.0, 0.0}, 1e-12));
      openCeiling.cutCeiling.reset();
      d = decide(Mode::Assisted, true, forward, &openCeiling);
      check("  no hold: the stick as it is", same(d.body, forward, 1e-12));

      // Check 2 again (2026-11-16) replayed: full forward at a box, closing
      // 1 cm a cycle from 0.60 m, the OTOS's speed alternating 0.33 and 0.39
      // as it read. Without the hold the output rose again while cutting;
      // with it, it only falls.
      const auto approach = [&](bool withHold) {
        CutHold replay;
        int rises = 0;
        double last = -1.0;
        bool cutting = false;
        for (int i = 0; i <= 40; ++i) {
          const Scan at = wallAhead(0.60 - 0.01 * i, lidarAt, t0);
          AssistedInput in = through(&at, true);
          in.limit.measured = Twist{(i % 2 == 0) ? 0.33 : 0.39, (i % 2 == 0) ? 0.05 : -0.05, 0.0};
          if (withHold) {
            in.cutCeiling = replay.ceiling(forward);
          }
          const Decision step = decide(Mode::Assisted, true, forward, &in);
          replay.update(forward, step);
          cutting = cutting || step.limited->reason == Heron::Limiter::Reason::Cut;
          const double now = speedOf(step.body);
          if (std::getenv("HERON_CUT_TRACE") != nullptr) {
            std::printf("          %d %.2f m  %s  (%.2f, %.2f) makes %.3f%s\n", withHold ? 1 : 0, 0.60 - 0.01 * i,
                        Heron::Limiter::toString(step.limited->reason), step.body.vx, step.body.vy, now,
                        in.cutCeiling ? " ceiling" : "");
          }
          if (cutting && last >= 0.0 && now > last + 1e-9) {
            ++rises;
          }
          if (cutting) {
            last = now;
          }
        }
        return rises;
      };
      const int risesBefore = approach(false);
      const int risesAfter = approach(true);
      std::printf("        replay of check 2: %d rises while cutting without the hold, %d with it\n", risesBefore,
                  risesAfter);
      check("check 2 replayed: once cutting, the command never rises", risesAfter == 0);
    }

    std::printf("Commanded but not moving (design §19.4 step 4)\n");
    {
      MotionWatch watch;
      const TimePoint t = t0;
      bool tripped = watch.update(0.27, 0.0, true, t);
      tripped = tripped || watch.update(0.27, 0.0, true, t + 480ms);
      check("full strafe expected, the OTOS reads 0: not tripped before 0.5 s", !tripped);
      check("  tripped at 0.5 s", watch.update(0.27, 0.0, true, t + 500ms));
      check("  stays tripped while the stick still asks", watch.update(0.0, 0.0, true, t + 2s));
      check("  the stick centred: cleared", !watch.update(0.0, 0.0, false, t + 2100ms));
      MotionWatch moving;
      bool everTripped = false;
      for (int i = 0; i <= 50; ++i) {
        const double measured = std::min(0.27, 0.75 * 0.02 * i);  // 0.15 m/s within 0.2 s, as measured
        everTripped = moving.update(0.27, measured, true, t + i * 20ms) || everTripped;
      }
      check("starting from rest as the robot did (0.15 m/s in 0.2 s): never trips", !everTripped);
      MotionWatch deadBand;
      for (int i = 0; i <= 100; ++i) {
        everTripped = deadBand.update(0.01, 0.0, true, t + i * 20ms) || everTripped;
      }
      check("a dead-band command (expects 0.01 from rest) that moves nothing: never trips", !everTripped);
      MotionWatch onFloor;
      for (int i = 0; i <= 100; ++i) {
        everTripped = onFloor.update(0.27, (i % 2 == 0) ? 0.012 : 0.0, true, t + i * 20ms) || everTripped;
      }
      check("readings of 0.01 m/s or more break the window: not tripped", !everTripped);
      MotionWatch reset;
      (void)reset.update(0.27, 0.0, true, t);
      (void)reset.update(0.27, 0.0, true, t + 600ms);
      reset.reset();
      check("reset: cleared", !reset.tripped());

      AssistedInput lifted = through(&far, true);
      lifted.notMoving = true;
      d = decide(Mode::Assisted, true, BodyVelocity{0.0, -1.0, 0.0}, &lifted);
      check("tripped: Assisted stops, NOT_MOVING", stopped(d.wheels) && d.why == Why::NotMoving);
      check("  NOT_MOVING as text", std::string(toString(Why::NotMoving)) == "NOT_MOVING");
    }

    std::printf("Driven and standing (StallWatch, 2026-11-18)\n");
    {
      const TimePoint t = t0;
      const BodyVelocity right{0.0, -0.5, 0.0};
      StallWatch stall;
      std::optional<Heron::Motion::DeadCommand> found;
      for (int i = 0; i <= 49 && !found; ++i) {
        found = stall.update(right, 0.0, t + i * 20ms);
      }
      check("right at 0.5, the pose still: nothing before 1 s", !found);
      found = stall.update(right, 0.0, t + 1000ms);
      check("  at 1 s: right at 0.5 dead", found && near(found->size, 0.5, 1e-9) &&
                                                 near(found->directionRad, -std::numbers::pi / 2.0, 1e-9));
      check("  then a fresh window: nothing the next cycle", !stall.update(right, 0.0, t + 1020ms));
      StallWatch moving;
      bool any = false;
      for (int i = 0; i <= 100; ++i) {
        any = moving.update(right, 0.10, t + i * 20ms).has_value() || any;
      }
      check("the pose moving 0.10 m/s: never", !any);
      StallWatch phantom;  // the 9 s: the OTOS read 0.03 to 0.04, the pose 0.00
      for (int i = 0; i <= 100; ++i) {
        any = phantom.update(right, 0.02, t + i * 20ms).has_value() || any;
      }
      check("the pose moving 0.02 m/s (under 0.03): found", any);
      StallWatch unknown;
      any = false;
      for (int i = 0; i <= 100; ++i) {
        any = unknown.update(right, -1.0, t + i * 20ms).has_value() || any;
      }
      check("the pose's movement not known: never", !any);
      StallWatch full;
      for (int i = 0; i <= 100; ++i) {
        any = full.update(BodyVelocity{0.0, -1.0, 0.0}, 0.0, t + i * 20ms).has_value() || any;
      }
      check("full drive: never (a lift or a wall, not the table)", !any);
      StallWatch changing;
      bool early = false;
      for (int i = 0; i <= 50; ++i) {
        const BodyVelocity b = i < 30 ? right : BodyVelocity{0.5, 0.0, 0.0};
        early = changing.update(b, 0.0, t + i * 20ms).has_value() || early;
      }
      check("right then forward within the second: a fresh window, nothing yet", !early);
      StallWatch growing;
      found.reset();
      for (int i = 0; i <= 50 && !found; ++i) {
        found = growing.update(BodyVelocity{0.0, i < 25 ? -0.50 : -0.54, 0.0}, 0.0, t + i * 20ms);
      }
      check("0.50 then 0.54 (within 0.05): one window, the larger given", found && near(found->size, 0.54, 1e-9));

      std::vector<Heron::Motion::DeadCommand> dead;
      addDead(dead, Heron::Motion::DeadCommand{-std::numbers::pi / 2.0, 0.5});
      addDead(dead, Heron::Motion::DeadCommand{-80.0 * Heron::Geometry::kDegToRad, 0.4});
      check("addDead: 10 degrees apart, one, the larger size", dead.size() == 1 && near(dead[0].size, 0.5, 1e-9));
      addDead(dead, Heron::Motion::DeadCommand{std::numbers::pi / 2.0, 0.6});
      check("  left as well: two", dead.size() == 2);
    }

    const AssistedInput stale = through(&far, false);
    d = decide(Mode::Assisted, true, BodyVelocity{0.6, 0.0, 0.0}, &stale);
    check("the pose not fresh: stopped, POSE_NOT_FRESH (P16)",
          stopped(d.wheels) && d.why == Why::PoseNotFresh);
  }

  std::printf("Navigate (§8.7, step 6)\n");
  {
    const TimePoint now = t0 + 20ms;
    Heron::Planner::Course course;
    course.seq = 4;
    Heron::Planner::Plan plan;
    plan.courseSeq = 4;
    plan.commandVx = 0.6;
    plan.commandVy = -0.2;
    plan.twist = Twist{0.30, -0.05, 0.0};
    plan.time = now - 100ms;
    const BodyVelocity turning{0.9, 0.0, 0.25};  // only wz is used
    check("the plan's command, the stick's turn",
          same(navigateCommand(course, &plan, turning, geometry, true, now), BodyVelocity{0.6, -0.2, 0.25}));
    check("the stick centred (no course): stop at once",
          same(navigateCommand(std::nullopt, &plan, turning, geometry, true, now), BodyVelocity{}));
    check("no plan yet: stop", same(navigateCommand(course, nullptr, turning, geometry, true, now), BodyVelocity{}));
    Heron::Planner::Plan older = plan;
    older.courseSeq = 3;
    check("a plan for the course before a re-anchor: stop",
          same(navigateCommand(course, &older, turning, geometry, true, now), BodyVelocity{}));
    Heron::Planner::Plan stale = plan;
    stale.time = now - 301ms;
    check("a plan older than 0.3 s: stop", same(navigateCommand(course, &stale, turning, geometry, true, now), BodyVelocity{}));
    stale.time = now - 300ms;
    check("  0.3 s exactly: driven", !same(navigateCommand(course, &stale, turning, geometry, true, now), BodyVelocity{}));
    const BodyVelocity speeds = navigateCommand(course, &plan, turning, geometry, false, now);
    const BodyVelocity expected = toBodyVelocity(Twist{0.30, -0.05, 0.0}, geometry, true);
    check("no table: the plan's speeds as fractions, the stick's turn",
          same(speeds, BodyVelocity{expected.vx, expected.vy, 0.25}));

    const AssistedInput open = assistedWith(&far);
    const AssistedInput wall = assistedWith(&close);
    const BodyVelocity want{0.6, 0.0, 0.0};
    Decision n = decide(Mode::Navigate, true, want, &open);
    Decision a = decide(Mode::Assisted, true, want, &open);
    check("through decide(): exactly as Assisted drives the same command",
          n.why == Why::Limited && n.wheels == a.wheels && same(n.body, a.body));
    n = decide(Mode::Navigate, true, want, &wall);
    a = decide(Mode::Assisted, true, want, &wall);
    check("  a wall ahead: the limiter cuts it the same way",
          n.wheels == a.wheels && n.limited && a.limited && n.limited->reason == a.limited->reason &&
              n.body.vx < want.vx);
    check("  not ready: stopped, ASSISTED_UNAVAILABLE",
          decide(Mode::Navigate, true, want, nullptr).why == Why::AssistedUnavailable);

    BlockedWatch watch;
    bool shown = false;
    for (int i = 0; i <= 25; ++i) {  // 0.5 s exactly
      shown = watch.update(true, false, t0 + i * 20ms) || shown;
    }
    check("BLOCKED: not at 0.5 s of the plan held back", !shown);
    check("  shown past 0.5 s", watch.update(true, false, t0 + 520ms));
    check("  cleared once anything is driven", !watch.update(true, true, t0 + 540ms));
    (void)watch.update(true, false, t0 + 560ms);
    check("  and starts its window again", !watch.update(true, false, t0 + 1000ms));
    check("  cleared when the plan asks for nothing", !watch.update(false, false, t0 + 2000ms));
  }

  std::printf("Still by the LiDAR (2026-11-16)\n");
  {
    const auto scanAt = [](double dx, std::uint64_t seq, TimePoint end) {
      Scan scan = wallAhead(1.0, Vec2{dx, 0.0}, end);
      scan.seq = seq;
      return std::make_shared<const Scan>(scan);
    };
    const std::optional<double> moved = scanChange(*scanAt(0.0, 1, t0), *scanAt(0.05, 2, t0));
    check("scanChange: the wall 5 cm nearer reads about 5 cm", moved && near(*moved, 0.05, 0.005));
    Scan few;
    few.points.push_back(Heron::Point{0.0f, 1.0f});
    check("  too few sectors to compare: nothing", !scanChange(few, few));

    LidarStill none;
    check("no scan: not still", !none.update(nullptr));
    LidarStill standing;
    bool at900 = true;
    bool at1100 = false;
    for (std::uint64_t k = 0; k <= 8; ++k) {  // a scan every 137 ms, 1 mm of noise
      const bool s = standing.update(scanAt(k % 2 == 0 ? 0.0 : 0.001, k + 1, t0 + k * 137ms));
      at900 = k == 6 ? s : at900;   // 822 ms after the first
      at1100 = k == 8 ? s : at1100; // 1096 ms
    }
    check("standing: not still before 1 s of scans", !at900);
    check("  still after it", at1100);
    check("  the same scan again changes nothing", standing.update(scanAt(0.001, 9, t0 + 8 * 137ms)));
    LidarStill carried;
    bool any = false;
    for (std::uint64_t k = 0; k <= 30; ++k) {  // 1 cm a scan, about 0.07 m/s, for 4 s
      any = carried.update(scanAt(0.01 * static_cast<double>(k), k + 1, t0 + k * 137ms)) || any;
    }
    check("carried slowly: never still (each scan against the first of the run)", !any);
  }

  std::printf("The OTOS restarted after a lift (2026-11-16)\n");
  {
    using In = OtosRecovery::Input;
    const In healthy{true, false, 0.0, true, true};
    OtosRecovery r;
    bool any = false;
    for (int i = 0; i < 300; ++i) {  // 6 s: fresh and still throughout
      any = r.update(healthy, t0 + i * 20ms) || any;
    }
    check("fresh, still, no turn read: never", !any);
    OtosRecovery lift;
    any = false;
    for (int i = 0; i <= 15; ++i) {  // not fresh 0.3 s, as earlier lifts cleared
      any = lift.update(In{false, false, 0.0, true, true}, t0 + i * 20ms) || any;
    }
    for (int i = 16; i < 200; ++i) {
      any = lift.update(healthy, t0 + i * 20ms) || any;
    }
    check("not fresh 0.3 s, then fresh: never", !any);
    OtosRecovery stuck;
    int firstAt = -1;
    int count = 0;
    for (int i = 0; i <= 600; ++i) {  // 12 s not fresh, sticks centred, still
      if (stuck.update(In{false, false, 0.0, true, true}, t0 + i * 20ms)) {
        firstAt = firstAt < 0 ? i : firstAt;
        ++count;
      }
    }
    check("not fresh 2 s, sticks centred, still: restarts at 2 s", firstAt == 100);
    check("  then no more than one every 5 s (at 2, 7, 12 s)", count == 3);
    OtosRecovery driving;
    any = false;
    for (int i = 0; i <= 300; ++i) {  // 6 s not fresh, a stick held
      any = driving.update(In{false, false, 0.0, false, true}, t0 + i * 20ms) || any;
    }
    check("a stick held: never", !any);
    int at = -1;
    for (int i = 301; i <= 400 && at < 0; ++i) {
      at = driving.update(In{false, false, 0.0, true, true}, t0 + i * 20ms) ? i : -1;
    }
    check("  centred: after 1 s of it", at == 351);
    // navigate-fresh-2026-11-16: not fresh while carried, sticks centred.
    OtosRecovery carried;
    any = false;
    for (int i = 0; i <= 300; ++i) {  // 6 s carried
      any = carried.update(In{false, false, 0.0, true, false}, t0 + i * 20ms) || any;
    }
    check("carried (the LiDAR not still), sticks centred: never", !any);
    check("  put down and still: at once", carried.update(In{false, false, 0.0, true, true}, t0 + 6020ms));
    // ...where the first version restarted, and the OTOS came back reading
    // a turn of -0.19 rad/s, and TILT, standing.
    OtosRecovery badStart;
    at = -1;
    for (int i = 0; i <= 100 && at < 0; ++i) {
      at = badStart.update(In{true, false, -0.19, true, true}, t0 + i * 20ms) ? i : -1;
    }
    check("fresh but a turn of -0.19 rad/s read standing: restarts after 1 s", at == 50);
    check("  says why", std::string(badStart.why()).find("turn") != std::string::npos);
    OtosRecovery tilted;
    at = -1;
    for (int i = 0; i <= 100 && at < 0; ++i) {
      at = tilted.update(In{true, true, 0.0, true, true}, t0 + i * 20ms) ? i : -1;
    }
    check("TILT standing: restarts after 1 s", at == 50);
    OtosRecovery turning;
    any = false;
    for (int i = 0; i <= 300; ++i) {  // a real turn, or TILT while driving: the LiDAR not still
      any = turning.update(In{true, true, -0.8, true, false}, t0 + i * 20ms) || any;
    }
    check("a turn or TILT while the LiDAR sees motion: never", !any);
  }

  std::printf("Disarmed\n");
  {
    const AssistedInput open = assistedWith(&far);
    Decision d;
    bool allStopped = true;
    for (const Mode mode : {Mode::Manual, Mode::Assisted, Mode::Navigate}) {
      d = decide(mode, false, BodyVelocity{1.0, 0.0, 0.0}, &open);
      allStopped = allStopped && stopped(d.wheels) && d.why == Why::Disarmed;
    }
    check("disarmed: stopped in every mode", allStopped);
  }

  std::printf("\n%s: %d failure%s\n", s_failures == 0 ? "OK" : "FAILED", s_failures,
              s_failures == 1 ? "" : "s");
  return s_failures == 0 ? 0 : 1;
}
