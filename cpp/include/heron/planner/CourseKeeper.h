// File        : CourseKeeper.h
// Author      : Chris Pane
// Description : Navigate's course: the stick's direction, held as a line in
//               the odom frame for the planner to follow.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// In Navigate you hold the right stick the way you want Heron to go, and
// let go to stop it (D1). The planner needs that as a line, not a stick:
// where it started and which way it runs, in the odom frame, so that after
// sliding round a box Heron can come back to it (docs/navigation-design.md
// §8.1). CourseKeeper turns the stick into that line and keeps it:
//
//   - Engage: the first push. The stick's direction, turned into the odom
//     frame by Heron's heading, is the course heading, and the line runs
//     through Heron's position at that moment.
//   - Held: the speed and turn rate follow the stick on every call, the
//     line stays put. Only when the stick's odom-frame direction drifts
//     more than 10 degrees from the heading (you moved the stick, or Heron
//     turned under it) is the line re-anchored where Heron is.
//   - Let go: no course. The control loop stops Heron at once.
//   - A new epoch (odometry restarted): re-anchored, since the old anchor
//     is in a frame that no longer exists.
//
// Every engage and re-anchor raises the course's seq, and every plan says
// which seq it was made for, so the control loop never drives a plan made
// for a line that has since moved.
//
// Pure: no clock, no thread. The control loop calls update() every 20 ms
// in Navigate. Design: docs/navigation-cpp-design.md §8.2.
//
// Usage
// -----
//   Heron::Planner::CourseKeeper keeper;
//   const auto course = keeper.update(toTwist(stick, geometry), pose, epoch);
//   localPlanner.setCourse(course);           // no value: Heron stops
//   ...
//   keeper.reset();                            // on leaving Navigate

#ifndef HERON_PLANNER_COURSE_KEEPER_H
#define HERON_PLANNER_COURSE_KEEPER_H

#include <cstdint>
#include <optional>

#include "heron/geometry/Pose2.h"

namespace Heron::Planner {

/** What the stick asks for, design §8.1, in the odom frame. */
struct Course {
  std::uint64_t seq{0};       // raised on every engage and re-anchor
  std::uint64_t epoch{0};     // the odom frame it is in (I8)
  Geometry::Vec2 anchor{};    // the line runs through here...
  double headingRad{0.0};     // ...in this direction
  double speedMps{0.0};       // the stick's size x throttle x max_speed (I13)
  double turnRateRadps{0.0};  // the left stick's, as in Manual (I16)
};

struct CourseConfig {
  double reanchorRad{10.0 * Geometry::kDegToRad};
};

/**
 * Turns the stick into a course and keeps it while the stick is held.
 *
 * Thread Safety: a plain value; one owner, the control loop.
 */
class CourseKeeper {
  public:
    explicit CourseKeeper(CourseConfig config = {});

    /**
     * @param stick The sticks as a twist, m/s and rad/s, robot frame
     * @param pose Heron now, odom frame
     * @param epoch The pose's epoch
     * @return The course, or no value when the right stick is centred
     */
    std::optional<Course> update(const Geometry::Twist &stick, const Geometry::Pose2 &pose,
                                 std::uint64_t epoch);

    /** Forgets the course; the next push engages afresh. seq keeps rising. */
    void reset() noexcept;

  private:
    CourseConfig m_config;
    std::optional<Course> m_course;
    std::uint64_t m_seq{0};
};

}  // namespace Heron::Planner

#endif  // HERON_PLANNER_COURSE_KEEPER_H
