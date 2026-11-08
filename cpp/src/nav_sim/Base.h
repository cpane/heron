// File        : Base.h
// Author      : Chris Pane
// Description : The simulator's Heron: a mecanum base that follows a twist
//               after a lag, within its acceleration limits.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// How the real Heron answers a command, roughly: the motor link and the
// motors take a moment to respond (the lag), and the wheels cannot change
// speed at once (the acceleration limits). A mecanum base can drive, strafe
// and turn at once, so a command is a full twist in the robot frame.
//
// The truth: the simulated OTOS and LiDAR read the pose and velocity from
// here, and tests compare what the libraries believe with it.
//
// The numbers are starting values, not measured: [geometry] accel and
// brake are measured on the floor in N1 (docs/navigation-design.md §10),
// and the lag is a guess for the motor link and motors together.
//
// Usage
// -----
//   using namespace std::chrono_literals;
//   Heron::TimePoint now{};
//   Heron::NavSim::Base base(BaseConfig{}, Pose2{0.0, 0.0, 0.0}, now);
//   base.command(Twist{0.25, 0.0, 0.0}, now);
//   for (int i = 0; i < 100; ++i) {
//     now += 10ms;
//     base.advance(now);
//   }
//   base.truePose();   // about 0.18 m along x: 60 ms lag, 0.42 s to reach speed

#ifndef HERON_NAV_SIM_BASE_H
#define HERON_NAV_SIM_BASE_H

#include <chrono>
#include <deque>

#include "heron/geometry/Pose2.h"
#include "heron/lidar/Scan.h"

namespace Heron::NavSim {

/** Mecanum motion with an acceleration limit and a lag. */
struct BaseConfig {
  double accel{0.6};      // m/s^2, on each of vx and vy, speeding up and braking
  double turnAccel{3.0};  // rad/s^2
  Duration lag{std::chrono::milliseconds(60)};
};

/**
 * Thread Safety: Not thread-safe; one test owns it.
 */
class Base {
  public:
    Base(BaseConfig config, Geometry::Pose2 start, TimePoint startTime);

    /** Asks for a twist, robot frame; it starts to take effect at now + lag. */
    void command(const Geometry::Twist &twist, TimePoint now);

    /** Moves the base on to now, in steps of at most 1 ms. A now in the
     *  past does nothing. */
    void advance(TimePoint now);

    /** @return Where the base is, odom frame */
    [[nodiscard]] Geometry::Pose2 truePose() const noexcept { return m_pose; }

    /** @return How fast it moves, robot frame */
    [[nodiscard]] Geometry::Twist trueVelocity() const noexcept { return m_velocity; }

  private:
    struct Pending {
      TimePoint applyAt;
      Geometry::Twist twist;
    };

    void step(double dtS);

    BaseConfig m_config;
    Geometry::Pose2 m_pose;
    Geometry::Twist m_velocity;
    Geometry::Twist m_target;
    std::deque<Pending> m_pending;
    TimePoint m_now;
};

}  // namespace Heron::NavSim

#endif  // HERON_NAV_SIM_BASE_H
