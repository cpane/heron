// File        : Otos.cpp
// Author      : Chris Pane
// Description : The simulator's OTOS.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "nav_sim/Otos.h"

#include <chrono>

namespace Heron::NavSim {

Otos::Otos(OtosConfig config, TimePoint start)
    : m_config(config), m_random(config.seed), m_lastTime(start) {}

Pose::PoseSample Otos::read(TimePoint now, const Base &base) {
  const Geometry::Pose2 truth = base.truePose();
  if (!m_started) {
    m_believed = truth;
    m_lastTrue = truth;
    m_started = true;
  }
  if (now > m_lastTime) {
    // The true step since the last read, seen from where the base was then,
    // applied to the believed pose: so it goes in the believed heading.
    // Then the heading drifts on, and the next step goes in that.
    const double dtS = std::chrono::duration<double>(now - m_lastTime).count();
    m_believed = Geometry::compose(m_believed, Geometry::relative(m_lastTrue, truth));
    m_believed.heading = Geometry::wrapAngle(
        m_believed.heading + m_config.driftDegPerMin * Geometry::kDegToRad / 60.0 * dtS);
    m_lastTrue = truth;
    m_lastTime = now;
  }

  Geometry::Pose2 reported = m_believed;
  if (m_config.noiseM > 0.0) {
    std::normal_distribution<double> noise(0.0, m_config.noiseM);
    reported.x += noise(m_random);
    reported.y += noise(m_random);
  }
  return Pose::PoseSample{now, reported, base.trueVelocity()};
}

}  // namespace Heron::NavSim
