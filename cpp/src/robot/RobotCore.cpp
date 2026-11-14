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
#include <stdexcept>

namespace Heron::Robot {

namespace {

double held(double fraction) {
  return std::clamp(fraction, -1.0, 1.0);
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
  return Geometry::Twist{v.vx * g.maxSpeed, v.vy * g.maxSpeed, v.wz * g.maxTurnRate};
}

Drive::BodyVelocity toBodyVelocity(const Geometry::Twist &t,
                                   const Profile::GeometryProfile &g) noexcept {
  return Drive::BodyVelocity{held(t.vx / g.maxSpeed), held(t.vy / g.maxSpeed),
                             held(t.wz / g.maxTurnRate)};
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
      Limiter::LimitInput input = assisted->limit;
      input.command = toTwist(stick, assisted->geometry);
      const Limiter::LimitResult limited = assisted->limiter->limit(input);
      decision.body = toBodyVelocity(limited.command, assisted->geometry);
      decision.why = Why::Limited;
      decision.limited = limited;
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
