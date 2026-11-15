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

Perception::PerceptionConfig toPerceptionConfig(const Profile::LidarProfile &l) {
  Perception::PerceptionConfig config;
  config.mount = Perception::LidarMount{Geometry::Vec2{l.x, l.y}};
  return config;
}

Planner::PlannerConfig toPlannerConfig(const Profile::RigProfile &profile,
                                       std::shared_ptr<const Motion::MotionTable> motion) {
  if (!profile.geometry || !profile.navigation) {
    throw std::invalid_argument("Heron::Robot::toPlannerConfig() : the profile needs [geometry] and [navigation]");
  }
  Planner::PlannerConfig config;
  config.outline = Geometry::Outline{profile.geometry->length / 2.0, profile.geometry->width / 2.0};
  config.stopMarginM = profile.navigation->stopMargin;
  config.maxSpeedMps = profile.geometry->maxSpeed;
  config.accelMps2 = profile.geometry->accel;
  config.motion = std::move(motion);
  return config;
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

HeadingHold::HeadingHold() noexcept : HeadingHold(Config{}) {}

HeadingHold::HeadingHold(Config config) noexcept : m_config(config) {}

double HeadingHold::update(double turnStickRadps, bool translating, double headingRad, double driftRadps,
                           std::uint64_t epoch) noexcept {
  if (epoch != m_epoch) {
    m_epoch = epoch;
    m_holding = false;  // the heading held is in another frame (I8)
  }
  if (turnStickRadps != 0.0) {
    m_holding = false;  // the stick turns; letting it go takes the new heading
    return turnStickRadps;
  }
  if (!translating) {
    m_holding = false;  // standing still nothing is held
    return 0.0;
  }
  if (!m_holding) {
    m_holding = true;
    m_target = headingRad;
  }
  const double error = Geometry::wrapAngle(m_target - headingRad);
  return std::clamp(m_config.gainPerS * error - driftRadps, -m_config.maxRateRadps, m_config.maxRateRadps);
}

void HeadingHold::reset() noexcept {
  m_holding = false;
}

double holdFraction(double turnRadps, const Drive::BodyVelocity &translation,
                    const Profile::GeometryProfile &g) noexcept {
  const double slowest = std::min(std::abs(translation.vx - translation.vy),
                                  std::abs(translation.vx + translation.vy));
  const double perUnit = g.maxTurnRate * (1.0 - g.minSpeed / g.maxSpeed);
  if (slowest <= 0.0 || !(perUnit > 0.0)) {
    return 0.0;
  }
  return std::clamp(turnRadps / perUnit, -slowest / 2.0, slowest / 2.0);
}

MotionWatch::MotionWatch() noexcept : MotionWatch(Config{}) {}

MotionWatch::MotionWatch(Config config) noexcept : m_config(config) {}

bool MotionWatch::update(double expectedMps, double measuredMps, bool translating,
                         std::chrono::steady_clock::time_point now) noexcept {
  if (m_tripped) {
    if (!translating) {
      m_tripped = false;  // the stick centred: the driver has seen it
      m_since.reset();
    }
    return m_tripped;
  }
  if (expectedMps >= m_config.expectMps && measuredMps < m_config.stillMps) {
    if (!m_since) {
      m_since = now;
    }
    if (now - *m_since >= m_config.window) {
      m_tripped = true;
    }
  } else {
    m_since.reset();
  }
  return m_tripped;
}

void MotionWatch::reset() noexcept {
  m_tripped = false;
  m_since.reset();
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
    case Why::NotMoving:
      return "NOT_MOVING";
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
    case Mode::Assisted:
    case Mode::Navigate: {  // Navigate: the plan's command, as a stick
      if (assisted == nullptr || assisted->limiter == nullptr) {
        decision.why = Why::AssistedUnavailable;
        return decision;
      }
      if (!assisted->limit.poseFresh) {
        decision.why = Why::PoseNotFresh;  // P16: a nonsense speed is not trusted
        return decision;
      }
      if (assisted->notMoving) {
        decision.why = Why::NotMoving;  // lifted, or stuck: told to move, nothing moved
        return decision;
      }
      Limiter::LimitInput input = assisted->limit;
      if (assisted->motion == nullptr) {
        input.command = toTwist(stick, assisted->geometry);
        if (assisted->holdRadps) {
          input.command.wz = *assisted->holdRadps;
        }
        const Limiter::LimitResult limited = assisted->limiter->limit(input);
        // Under min_speed: up to it if only the slow zone lowered it (the
        // sweep found the faster command clear for T, so the slower one is
        // too); otherwise stop. Header, "Units".
        decision.body = toBodyVelocity(limited.command, assisted->geometry,
                                       limited.reason == Limiter::Reason::Slowed);
        if (assisted->holdRadps) {
          decision.body.wz = holdFraction(*assisted->holdRadps * limited.scale.wz, decision.body,
                                          assisted->geometry);
        }
        decision.why = Why::Limited;
        decision.limited = limited;
        break;
      }
      // With the floor's table (design §19.2): the limiter is told what
      // the command really makes, kTableMargin faster, drift included.
      const Motion::MotionTable &table = *assisted->motion;
      // The turn: heading hold's rate when it holds (P14), else the stick's.
      const double turnRate = assisted->holdRadps
                                  ? *assisted->holdRadps
                                  : toTwist(Drive::BodyVelocity{0.0, 0.0, stick.wz}, assisted->geometry).wz;
      const auto told = [&](double vx, double vy, double turn) {
        const Geometry::Twist made = table.predict(vx, vy);
        return Geometry::Twist{made.vx * kTableMargin, made.vy * kTableMargin, turn + made.wz};
      };
      input.command = told(stick.vx, stick.vy, turnRate);
      const Limiter::LimitResult limited = assisted->limiter->limit(input);
      decision.why = Why::Limited;
      decision.limited = limited;
      // While holding (CutHold), nothing that makes more than last cycle:
      // last cycle's command if the limiter still passes it, else none.
      const auto applyCeiling = [&]() {
        if (!assisted->cutCeiling) {
          return;
        }
        const auto speedOf = [&](double vx, double vy) {
          const Geometry::Twist made = table.predict(vx, vy);
          return std::hypot(made.vx, made.vy);
        };
        const Drive::BodyVelocity &ceiling = *assisted->cutCeiling;
        if (speedOf(decision.body.vx, decision.body.vy) <= speedOf(ceiling.vx, ceiling.vy) + 1e-9) {
          return;
        }
        decision.body.vx = 0.0;
        decision.body.vy = 0.0;
        if (ceiling.vx != 0.0 || ceiling.vy != 0.0) {
          Limiter::LimitInput again = assisted->limit;
          again.command = told(ceiling.vx, ceiling.vy, turnRate * limited.scale.wz);
          const Limiter::Reason recheck = assisted->limiter->limit(again).reason;
          if (recheck == Limiter::Reason::Clear || recheck == Limiter::Reason::Slowed) {
            decision.body.vx = ceiling.vx;
            decision.body.vy = ceiling.vy;
          }
        }
      };
      if (limited.reason == Limiter::Reason::Clear) {
        decision.body = stick;
        applyCeiling();
        if (assisted->holdRadps) {
          decision.body.wz = holdFraction(*assisted->holdRadps, decision.body, assisted->geometry);
        }
        break;
      }
      // Limited: the turn by its scale; the translation the command nearest
      // what was allowed without exceeding it on either axis, checked once
      // more, or none. The limiter cuts by axis, so the allowed motion's
      // direction is not the stick's; a strafe that creeps forward into a
      // wall is refused rather than slid (§7.4 as built). Only commands
      // near the stick's direction stand in, so none swerves the robot
      // (check 1, 2026-11-14).
      decision.body.wz = stick.wz * limited.scale.wz;
      const auto command = table.nearestCommand(
          Geometry::Vec2{limited.command.vx / kTableMargin, limited.command.vy / kTableMargin},
          std::atan2(stick.vy, stick.vx));
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
      applyCeiling();
      // Held, the turn rides on the translation driven: none, no turn, so
      // a stopped robot never meets the turn's dead band and spins.
      if (assisted->holdRadps) {
        decision.body.wz = holdFraction(*assisted->holdRadps * limited.scale.wz, decision.body,
                                        assisted->geometry);
      }
      break;
    }
  }
  decision.wheels = Drive::mix(decision.body);
  return decision;
}

Drive::BodyVelocity navigateCommand(const std::optional<Planner::Course> &course, const Planner::Plan *plan,
                                    const Drive::BodyVelocity &stick, const Profile::GeometryProfile &g,
                                    bool withTable, std::chrono::steady_clock::time_point now) noexcept {
  if (!course || plan == nullptr || plan->courseSeq != course->seq || now - plan->time > kMaxPlanAge) {
    return Drive::BodyVelocity{};
  }
  if (!withTable) {
    // The plan chose speeds; under min_speed, up to it: the planner found
    // the faster path clear.
    const Drive::BodyVelocity v = toBodyVelocity(Geometry::Twist{plan->twist.vx, plan->twist.vy, 0.0}, g, true);
    return Drive::BodyVelocity{v.vx, v.vy, stick.wz};
  }
  return Drive::BodyVelocity{plan->commandVx, plan->commandVy, stick.wz};
}

bool BlockedWatch::update(bool asked, bool driven, std::chrono::steady_clock::time_point now) noexcept {
  if (!asked || driven) {
    m_blocked = false;
    m_since.reset();
    return false;
  }
  if (!m_since) {
    m_since = now;
  }
  m_blocked = now - *m_since > kWindow;
  return m_blocked;
}

void BlockedWatch::reset() noexcept {
  m_blocked = false;
  m_since.reset();
}

bool OtosRecovery::update(bool poseFresh, bool sticksCentred, std::chrono::steady_clock::time_point now) noexcept {
  if (poseFresh) {
    m_notFreshSince.reset();
  } else if (!m_notFreshSince) {
    m_notFreshSince = now;
  }
  if (!sticksCentred) {
    m_centredSince.reset();
  } else if (!m_centredSince) {
    m_centredSince = now;
  }
  if (!m_notFreshSince || !m_centredSince || now - *m_notFreshSince < kNotFresh ||
      now - *m_centredSince < kCentred || (m_lastRestart && now - *m_lastRestart < kBetween)) {
    return false;
  }
  m_lastRestart = now;
  return true;
}

bool CutHold::along(const Drive::BodyVelocity &stick) const noexcept {
  return (stick.vx != 0.0 || stick.vy != 0.0) &&
         std::abs(Geometry::wrapAngle(std::atan2(stick.vy, stick.vx) - m_directionRad)) <=
             Motion::kSubstituteWidthRad + 1e-9;
}

std::optional<Drive::BodyVelocity> CutHold::ceiling(const Drive::BodyVelocity &stick) const noexcept {
  if (!m_holding || !along(stick)) {
    return std::nullopt;
  }
  return m_last;
}

void CutHold::update(const Drive::BodyVelocity &stick, const Decision &decision) noexcept {
  const bool translating = stick.vx != 0.0 || stick.vy != 0.0;
  if (decision.why != Why::Limited || !decision.limited || !translating) {
    reset();
    return;
  }
  if (m_holding && !along(stick)) {
    reset();  // turned away: a new hold only on a new Cut
  }
  if (decision.limited->reason == Limiter::Reason::Clear) {
    if (!m_holding || ++m_clears >= kReleaseCycles) {
      reset();
      return;
    }
  } else {
    m_clears = 0;
  }
  if (!m_holding) {
    if (decision.limited->reason != Limiter::Reason::Cut) {
      return;
    }
    m_holding = true;
    m_directionRad = std::atan2(stick.vy, stick.vx);
  }
  m_last = Drive::BodyVelocity{decision.body.vx, decision.body.vy, 0.0};
}

void CutHold::reset() noexcept {
  m_holding = false;
  m_clears = 0;
  m_last = Drive::BodyVelocity{};
}

bool CutHold::holding() const noexcept {
  return m_holding;
}

}  // namespace Heron::Robot
