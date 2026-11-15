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
// Moving as the robot moves (design §19, P17): given the floor's motion
// table, commandFractions() takes what heron_robot sends, fractions of
// full drive, and aims for what the table says that command makes, drift
// included: a full strafe right on the bench rig's hardwood goes 0.25 m/s
// and turns the robot as it goes, a diagonal loses half its sideways part.
// Then the same lag and acceleration limits as command(). The table has
// no turn, so the turn stick's wz is added as the robot does it: while
// translating every wheel is past the motors' dead band and a unit of wz
// adds (maxTurnRate - turnFloor) rad/s; turning on the spot, any wz is
// lifted to the floor first (RobotCore.h, "Units"). The table's stalls
// read as moving (MotionTable::predict()), which a rolling robot does but a
// still one does not: the simulated base starts small commands the real
// one cannot, the side a planner test does not lose by.
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
//
//   BaseConfig onTheFloor;
//   onTheFloor.motion = std::make_shared<const Heron::Motion::MotionTable>(points, false);
//   Heron::NavSim::Base robot(onTheFloor, Pose2{}, now);
//   robot.commandFractions(0.0, -1.0, 0.0, now);   // a full strafe right, as measured

#ifndef HERON_NAV_SIM_BASE_H
#define HERON_NAV_SIM_BASE_H

#include <chrono>
#include <deque>
#include <memory>

#include "heron/geometry/Pose2.h"
#include "heron/motion/MotionTable.h"
#include "heron/lidar/Scan.h"

namespace Heron::NavSim {

/** Mecanum motion with an acceleration limit and a lag. */
struct BaseConfig {
  double accel{0.6};      // m/s^2, on each of vx and vy, speeding up and braking
  double turnAccel{3.0};  // rad/s^2
  Duration lag{std::chrono::milliseconds(60)};
  /** The floor's motion table, for commandFractions() (P17); nullptr: only
   *  command() works. */
  std::shared_ptr<const Motion::MotionTable> motion;
  double maxTurnRateRadps{3.4};  // [geometry] max_turn_rate: full turn stick on the spot
  double turnFloorRadps{1.3};    // the turn's dead band floor: max_turn_rate x min/max speed
};

/**
 * Thread Safety: Not thread-safe; one test owns it.
 */
class Base {
  public:
    Base(BaseConfig config, Geometry::Pose2 start, TimePoint startTime);

    /** Asks for a twist, robot frame; it starts to take effect at now + lag. */
    void command(const Geometry::Twist &twist, TimePoint now);

    /**
     * Asks for what heron_robot would send, fractions of full drive, robot
     * frame: the translation moves as the motion table predicts, drift
     * included, the turn as the header says; then as command().
     *
     * @throws std::logic_error if the config has no motion table
     */
    void commandFractions(double vx, double vy, double wz, TimePoint now);

    /** @return What commandFractions() aims for, before lag and limits */
    [[nodiscard]] Geometry::Twist fractionsToTwist(double vx, double vy, double wz) const;

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
