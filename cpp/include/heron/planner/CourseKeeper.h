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
//     line stays put. It is re-anchored where Heron is only when you
//     change it: the right stick moved more than 10 degrees from where it
//     pointed, or, after you used the turn stick, the course direction
//     (heading plus stick) more than 10 degrees from the line's. Heron
//     turning on its own, with the turn stick centred, never moves the
//     line: on a 45 degree diagonal heading hold can do nothing and the
//     floor's drift turned Heron past 10 degrees, so the line was lost
//     (nav_scenario_test, scenario 1); held, heading hold turns Heron back
//     once it can, and the line is still there (design §19.4 step 5,
//     question 1, decided 2026-11-16).
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
    double m_stickRad{0.0};  // the right stick's direction, robot frame, at anchoring
    bool m_turned{false};    // the turn stick used since anchoring
};

}  // namespace Heron::Planner

#endif  // HERON_PLANNER_COURSE_KEEPER_H
