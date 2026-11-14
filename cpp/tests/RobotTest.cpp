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
//     backward one, and stops with no scan; Navigate stops (step 2).
//
// The bench profile is read from the repository on the Mac
// (HERON_PROFILE_DIR) and from ~/heron/profiles on a Pi, where
// deploy_cpp.sh puts it. Same style as the other tests: no framework.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
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
    return AssistedInput{&limiter, geometry,
                         Heron::Limiter::LimitInput{Twist{}, scan, &history, true, Twist{}, t0 + 20ms}};
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

  std::printf("Navigate and disarmed\n");
  {
    const AssistedInput open = assistedWith(&far);
    Decision d = decide(Mode::Navigate, true, BodyVelocity{0.5, 0.0, 0.0}, &open);
    check("Navigate: stopped until step 2", stopped(d.wheels) && d.why == Why::NavigateNotBuilt);
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
