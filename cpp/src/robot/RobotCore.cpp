// File        : RobotCore.cpp
// Author      : Chris Pane
// Description : heron_robot's decisions: the mode, the units, each library's
//               settings from the profile, and one cycle's choice.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "robot/RobotCore.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Heron::Robot {

namespace {

// The smallest wheel command the protocol carries, one step in 1000: the
// least that turns the motors, at the dead band's floor.
constexpr double kSmallestFraction = 0.001;

// One axis, a fraction of full drive to a speed: nothing at 0, otherwise
// the floor plus the rest in proportion (the motors' dead band, measured
// 2026-11-14 on the bench rig; header, "Units").
double toSpeed(double fraction, double full, double floor) {
  if (fraction == 0.0) {
    return 0.0;
  }
  return std::copysign(floor + (full - floor) * std::abs(fraction), fraction);
}

// One axis back. A speed under the floor cannot be driven: it stops, or,
// with upToFloor, goes at the floor, the least that moves. Held to -1..1.
double toFraction(double speed, double full, double floor, bool upToFloor) {
  const double size = std::abs(speed);
  if (size == 0.0) {
    return 0.0;
  }
  double fraction = 0.0;
  if (size < floor * (1.0 - 1e-9)) {
    if (!upToFloor) {
      return 0.0;
    }
    fraction = kSmallestFraction;
  } else {
    fraction = (size - floor) / (full - floor);
    if (fraction <= 0.0) {
      fraction = kSmallestFraction;  // the floor exactly: the least that moves
    }
  }
  return std::copysign(std::min(fraction, 1.0), speed);
}

// The turn's floor, in the same proportion as the speed's: the same dead
// band on the same wheels. Not measured on its own.
double turnFloor(const Profile::GeometryProfile &g) {
  return g.maxTurnRate * g.minSpeed / g.maxSpeed;
}

}  // namespace

Mode modeFromSwitch(Heron::Remote::Switch3 swc, bool manualIsHigh) noexcept {
  switch (swc) {
    case Heron::Remote::Switch3::Mid:
      return Mode::Assisted;
    case Heron::Remote::Switch3::High:
      return manualIsHigh ? Mode::Manual : Mode::Navigate;
    case Heron::Remote::Switch3::Low:
      return manualIsHigh ? Mode::Navigate : Mode::Manual;
  }
  return Mode::Manual;
}

const char *toString(Mode mode) noexcept {
  switch (mode) {
    case Mode::Manual:
      return "MANUAL";
    case Mode::Assisted:
      return "ASSISTED";
    case Mode::Navigate:
      return "NAVIGATE";
  }
  return "?";
}

Geometry::Twist toTwist(const Drive::BodyVelocity &v, const Profile::GeometryProfile &g) noexcept {
  return Geometry::Twist{toSpeed(v.vx, g.maxSpeed, g.minSpeed),
                         toSpeed(v.vy, g.maxSpeed, g.minSpeed),
                         toSpeed(v.wz, g.maxTurnRate, turnFloor(g))};
}

Drive::BodyVelocity toBodyVelocity(const Geometry::Twist &t, const Profile::GeometryProfile &g,
                                   bool upToMinSpeed) noexcept {
  return Drive::BodyVelocity{toFraction(t.vx, g.maxSpeed, g.minSpeed, upToMinSpeed),
                             toFraction(t.vy, g.maxSpeed, g.minSpeed, upToMinSpeed),
                             toFraction(t.wz, g.maxTurnRate, turnFloor(g), upToMinSpeed)};
}

Odometry::OtosConfig toOtosConfig(const Profile::OdometryProfile &o) {
  Odometry::OtosConfig config;
  config.linearScalar = o.linearScalar;
  config.angularScalar = o.angularScalar;
  config.offset = Odometry::Planar{o.x, o.y, o.yawDeg * Geometry::kDegToRad};
  return config;
}

Lidar::LidarConfig toLidarConfig(const Profile::LidarProfile &l) {
  Lidar::LidarConfig config;
  config.mountingOffsetRad = static_cast<float>(l.yawDeg * Geometry::kDegToRad);
  return config;
}

Limiter::LimiterConfig toLimiterConfig(const Profile::RigProfile &profile) {
  if (!profile.geometry || !profile.lidar || !profile.navigation) {
    throw std::invalid_argument(
        "Heron::Robot::toLimiterConfig() : the profile needs [geometry], [lidar] and [navigation]");
  }
  Limiter::LimiterConfig config;
  config.outline = Geometry::Outline{profile.geometry->length / 2.0, profile.geometry->width / 2.0};
  config.lidarOffset = Geometry::Vec2{profile.lidar->x, profile.lidar->y};
  config.stopMarginM = profile.navigation->stopMargin;
  config.slowMarginM = profile.navigation->slowMargin;
  config.brakeMps2 = profile.geometry->brake;
  return config;
}

std::optional<Motion::MotionTable> toMotionTable(const Profile::RigProfile &profile) {
  const Profile::SurfaceProfile *surface = Profile::findSurface(profile, profile.activeSurface);
  if (profile.activeSurface.empty() || surface == nullptr || surface->points.empty()) {
    return std::nullopt;
  }
  std::vector<Motion::MotionPoint> points;
  for (const Profile::SurfacePoint &p : surface->points) {
    points.push_back(Motion::MotionPoint{p.directionDeg * Geometry::kDegToRad, p.size,
                                         Geometry::Twist{p.vx, p.vy, p.wz}});
  }
  return Motion::MotionTable(std::move(points), surface->mirror);
}

Pose::PoseConfig toPoseConfig(const Profile::RigProfile &profile) {
  Pose::PoseConfig config;
  if (profile.geometry) {
    config.maxPlausibleSpeedMps = 1.5 * profile.geometry->maxSpeed;
  }
  return config;
}

std::string assistedRefusal(const Profile::RigProfile &profile) {
  std::string missing;
  const auto need = [&missing](bool present, const char *section) {
    if (!present) {
      missing += missing.empty() ? section : std::string(", ") + section;
    }
  };
  need(profile.geometry.has_value(), "[geometry]");
  need(profile.lidar.has_value(), "[lidar]");
  need(profile.odometry.has_value(), "[odometry]");
  need(profile.navigation.has_value(), "[navigation]");
  if (!missing.empty()) {
    return profile.source + " has no " + missing;
  }
  // A chosen floor with no table is refused, never another floor's (P18).
  if (!profile.activeSurface.empty()) {
    const Profile::SurfaceProfile *surface = Profile::findSurface(profile, profile.activeSurface);
    if (surface == nullptr || surface->points.empty()) {
      return "surface " + profile.activeSurface + " has no motion table";
    }
    try {
      (void)toMotionTable(profile);
    } catch (const std::invalid_argument &e) {
      return "surface " + profile.activeSurface + ": " + e.what();
    }
  }
  return Limiter::checkBlindSpot(toLimiterConfig(profile));
}

const char *toString(Why why) noexcept {
  switch (why) {
    case Why::Disarmed:
      return "DISARMED";
    case Why::Manual:
      return "MANUAL";
    case Why::Limited:
      return "LIMITED";
    case Why::AssistedUnavailable:
      return "ASSISTED_UNAVAILABLE";
    case Why::PoseNotFresh:
      return "POSE_NOT_FRESH";
    case Why::NavigateNotBuilt:
      return "NAVIGATE_NOT_BUILT";
  }
  return "?";
}

Decision decide(Mode mode, bool armed, const Drive::BodyVelocity &stick,
                const AssistedInput *assisted) {
  Decision decision;
  if (!armed) {
    return decision;  // Why::Disarmed, all zero
  }
  switch (mode) {
    case Mode::Manual:
      // heron_teleop exactly, and nothing else is read (D5).
      decision.body = stick;
      decision.why = Why::Manual;
      break;
    case Mode::Assisted: {
      if (assisted == nullptr || assisted->limiter == nullptr) {
        decision.why = Why::AssistedUnavailable;
        return decision;
      }
      if (!assisted->limit.poseFresh) {
        decision.why = Why::PoseNotFresh;  // P16: a nonsense speed is not trusted
        return decision;
      }
      Limiter::LimitInput input = assisted->limit;
      if (assisted->motion == nullptr) {
        input.command = toTwist(stick, assisted->geometry);
        const Limiter::LimitResult limited = assisted->limiter->limit(input);
        // Under min_speed: up to it if only the slow zone lowered it (the
        // sweep found the faster command clear for T, so the slower one is
        // too); otherwise stop. Header, "Units".
        decision.body = toBodyVelocity(limited.command, assisted->geometry,
                                       limited.reason == Limiter::Reason::Slowed);
        decision.why = Why::Limited;
        decision.limited = limited;
        break;
      }
      // With the floor's table (design §19.2): the limiter is told what
      // the command really makes, kTableMargin faster, drift included.
      const Motion::MotionTable &table = *assisted->motion;
      const double turnRate = toTwist(Drive::BodyVelocity{0.0, 0.0, stick.wz}, assisted->geometry).wz;
      const auto told = [&](double vx, double vy, double turn) {
        const Geometry::Twist made = table.predict(vx, vy);
        return Geometry::Twist{made.vx * kTableMargin, made.vy * kTableMargin, turn + made.wz};
      };
      input.command = told(stick.vx, stick.vy, turnRate);
      const Limiter::LimitResult limited = assisted->limiter->limit(input);
      decision.why = Why::Limited;
      decision.limited = limited;
      if (limited.reason == Limiter::Reason::Clear) {
        decision.body = stick;
        break;
      }
      // Limited: the turn by its scale; the translation the command nearest
      // what was allowed without exceeding it on either axis, checked once
      // more, or none. The limiter cuts by axis, so the allowed motion's
      // direction is not the stick's; a strafe that creeps forward into a
      // wall is refused rather than slid (§7.4 as built).
      decision.body.wz = stick.wz * limited.scale.wz;
      const auto command = table.nearestCommand(
          Geometry::Vec2{limited.command.vx / kTableMargin, limited.command.vy / kTableMargin});
      if (command) {
        Limiter::LimitInput again = assisted->limit;
        again.command = told(command->first, command->second, turnRate * limited.scale.wz);
        // Clear, or Slowed: the slow zone scales every command near
        // something, so nothing there is Clear; Slowed means the sweep found
        // this one clear for T, as the min_speed rule reasons.
        const Limiter::Reason recheck = assisted->limiter->limit(again).reason;
        if (recheck == Limiter::Reason::Clear || recheck == Limiter::Reason::Slowed) {
          decision.body.vx = command->first;
          decision.body.vy = command->second;
        }
      }
      break;
    }
    case Mode::Navigate:
      decision.why = Why::NavigateNotBuilt;
      return decision;
  }
  decision.wheels = Drive::mix(decision.body);
  return decision;
}

}  // namespace Heron::Robot
