// File        : Base.cpp
// Author      : Chris Pane
// Description : The simulator's mecanum base.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "nav_sim/Base.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Heron::NavSim {

namespace {

constexpr Duration kMaxStep = std::chrono::milliseconds(1);

/** current moved toward target by at most maxChange. */
double approach(double current, double target, double maxChange) noexcept {
  return current + std::clamp(target - current, -maxChange, maxChange);
}

double seconds(Duration d) noexcept {
  return std::chrono::duration<double>(d).count();
}

}  // namespace

Base::Base(BaseConfig config, Geometry::Pose2 start, TimePoint startTime)
    : m_config(config), m_pose(start), m_now(startTime) {}

void Base::command(const Geometry::Twist &twist, TimePoint now) {
  m_pending.push_back(Pending{now + m_config.lag, twist});
}

Geometry::Twist Base::fractionsToTwist(double vx, double vy, double wz) const {
  if (!m_config.motion) {
    throw std::logic_error("Heron::NavSim::Base::fractionsToTwist() : the config has no motion table");
  }
  Geometry::Twist t = m_config.motion->predict(vx, vy);
  if (wz != 0.0) {
    const double full = m_config.maxTurnRateRadps;
    const double floor = m_config.turnFloorRadps;
    const bool translating = vx != 0.0 || vy != 0.0;
    // Translating, the wheels are past the dead band: wz shifts them in
    // proportion. On the spot, the dead band lifts it to the floor first.
    t.wz += translating ? wz * (full - floor) : std::copysign(floor + (full - floor) * std::abs(wz), wz);
  }
  return t;
}

void Base::commandFractions(double vx, double vy, double wz, TimePoint now) {
  command(fractionsToTwist(vx, vy, wz), now);
}

void Base::advance(TimePoint now) {
  while (m_now < now) {
    // Step to the next command's time, the end, or 1 ms, whichever is first,
    // so a command takes effect at its own time and not a step late.
    TimePoint next = std::min(now, m_now + kMaxStep);
    if (!m_pending.empty() && m_pending.front().applyAt > m_now) {
      next = std::min(next, m_pending.front().applyAt);
    }
    while (!m_pending.empty() && m_pending.front().applyAt <= m_now) {
      m_target = m_pending.front().twist;
      m_pending.pop_front();
    }
    step(seconds(next - m_now));
    m_now = next;
  }
  while (!m_pending.empty() && m_pending.front().applyAt <= m_now) {
    m_target = m_pending.front().twist;
    m_pending.pop_front();
  }
}

void Base::step(double dtS) {
  // Velocity first, then the pose with the new velocity over the step: at
  // 1 ms steps the difference from averaging the two is under a micrometre.
  m_velocity.vx = approach(m_velocity.vx, m_target.vx, m_config.accel * dtS);
  m_velocity.vy = approach(m_velocity.vy, m_target.vy, m_config.accel * dtS);
  m_velocity.wz = approach(m_velocity.wz, m_target.wz, m_config.turnAccel * dtS);
  m_pose = Geometry::integrate(m_pose, m_velocity, dtS);
}

}  // namespace Heron::NavSim
