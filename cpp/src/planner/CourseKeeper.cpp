// File        : CourseKeeper.cpp
// Author      : Chris Pane
// Description : Navigate's course, from the stick.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/planner/CourseKeeper.h"

#include <cmath>

namespace Heron::Planner {

CourseKeeper::CourseKeeper(CourseConfig config) : m_config(config) {}

std::optional<Course> CourseKeeper::update(const Geometry::Twist &stick,
                                           const Geometry::Pose2 &pose, std::uint64_t epoch) {
  const double speed = std::hypot(stick.vx, stick.vy);
  if (speed == 0.0) {
    // Centred: sticksToVelocity() has already applied the dead band, so
    // exactly zero is the stick let go (D1).
    m_course.reset();
    return std::nullopt;
  }
  const double heading = Geometry::wrapAngle(pose.heading + std::atan2(stick.vy, stick.vx));
  const bool reanchor =
      !m_course || m_course->epoch != epoch ||
      std::abs(Geometry::wrapAngle(heading - m_course->headingRad)) > m_config.reanchorRad;
  if (reanchor) {
    Course course;
    course.seq = ++m_seq;
    course.epoch = epoch;
    course.anchor = Geometry::Vec2{pose.x, pose.y};
    course.headingRad = heading;
    m_course = course;
  }
  m_course->speedMps = speed;
  m_course->turnRateRadps = stick.wz;
  return m_course;
}

void CourseKeeper::reset() noexcept {
  m_course.reset();
}

}  // namespace Heron::Planner
