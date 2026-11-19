// File        : MotionTable.h
// Author      : Chris Pane
// Description : What each drive command really does on one floor: a table
//               of measured points, read both ways.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// On the bench rig's hardwood, a command does not make the motion its
// numbers suggest: forward has a floor, a strafe needs 0.4 of full before
// it moves at all, goes half as fast as forward and turns the robot as it
// goes, and left is not right mirrored (docs/navigation-design.md §19.1).
// Formulas cannot describe that, so the table stores what was measured:
// for each command, given as fractions of full drive, the twist the robot
// made (design §19, P13; docs/navigation-cpp-design.md §7.5, I17, I18).
//
//   command (fractions)          what the robot made (robot frame)
//   direction 0 deg, size 0.6 -> vx 0.31 m/s, vy 0,     wz +0.05 rad/s
//   direction -80,   size 0.6 -> vx 0.04,     vy -0.15, wz -0.20
//
// Points are kept as measured; the stick does not send the angles aimed
// for, so no even grid is required (I18). Points each within 10 degrees
// of the next, in direction, form one column, at their mean direction. A command is read in the two columns either side of its
// direction, each linear in size (below the smallest size as the
// smallest, since the motors' dead band lifts any command to it; above the
// largest as the largest), and then linear in angle between them. A column
// of one point gives that point at every size.
//
// A point that made almost nothing (under kStallMps) is a stall: the
// command could not start a still robot, as the table session measured
// each point from rest. A rolling robot does not stall there: on check 1
// (2026-11-14), commands the table put at 0.02 to 0.05 m/s kept the robot,
// already moving, at 0.20 to 0.25 m/s, and it stopped 8 cm from the box.
// So for every reading, a size at or below a column's last stall reads as
// the first point above it that moved: a small command is taken to move
// the robot, which is the safe side for the limiter. A column where
// nothing moved is read as measured.
//
// The other way round, two questions: which command makes this speed
// along a direction (commandFor()), and which command comes nearest to a
// given motion without exceeding it on either axis (nearestCommand()),
// which is how heron_robot turns the limiter's answer back into wheels.
// nearestCommand() looks only within kSubstituteWidthRad of the direction
// it is given, the stick's, so a substitute never swerves the robot (on
// check 1 a diagonal stood in for forward and drove it to the right).
//
// Pure: no clock, no thread, no files. heron_robot builds one from the
// profile's surface (heron_profile's onSurface(), I22).
//
// Usage
// -----
//   Heron::Motion::MotionTable table(points, false);
//   const Twist made = table.predict(0.0, -0.6);       // strafe right at 0.6
//   if (const auto c = table.nearestCommand(Vec2{0.0, -0.10}, -std::numbers::pi / 2.0)) {
//     // c->first, c->second: the fractions to drive
//   }

#ifndef HERON_MOTION_MOTION_TABLE_H
#define HERON_MOTION_MOTION_TABLE_H

#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "heron/geometry/Pose2.h"

namespace Heron::Motion {

/** One measured command and what the robot made of it. */
struct MotionPoint {
  double directionRad{0.0};  // the command's, robot frame: 0 forward, -pi/2 right
  double size{0.0};          // its length, hypot of the vx and vy fractions, > 0
  Geometry::Twist made{};    // vx, vy in m/s, wz in rad/s: what it did
};

/** Points each within this of the next, in direction, form one column (I18). */
inline constexpr double kColumnWidthRad = 10.0 * Geometry::kDegToRad;

/** A point that made less than this, m/s, is a stall: measured from rest. */
inline constexpr double kStallMps = 0.02;

/** nearestCommand() looks this far either side of the direction it is given. */
inline constexpr double kSubstituteWidthRad = 15.0 * Geometry::kDegToRad;

/**
 * A command seen not to start the robot, though the table says it should:
 * heron_robot's StallWatch adds one when the wheels were driven with it and
 * the pose did not move for a second. On 2026-11-16 the table, measured two
 * days and a set of batteries before, put a right strafe at 0.5 at
 * 0.11 m/s; driven by the limiter beside a box, the robot stood for 9 s.
 * A dead command covers every size up to its own within
 * kSubstituteWidthRad of its direction, from rest only (isDead()).
 */
struct DeadCommand {
  double directionRad{0.0};  // command space, robot frame
  double size{0.0};          // the largest size seen not to start it
};

/** @return whether the command (fractions) is at or below a dead one's
 *          size within kSubstituteWidthRad of its direction */
[[nodiscard]] bool isDead(std::span<const DeadCommand> dead, double vxFraction, double vyFraction) noexcept;

/**
 * The measured motion table for one floor.
 *
 * Thread Safety: a plain value; const methods may be called from any thread.
 */
class MotionTable {
  public:
    /**
     * @param points what was measured, in any order
     * @param mirror a column with no counterpart on the other side (within
     *        kColumnWidthRad of its mirror direction) is copied there, vy
     *        and wz negated
     * @throws std::invalid_argument for fewer than 2 columns, or a point
     *         with a size not above 0
     */
    MotionTable(std::vector<MotionPoint> points, bool mirror);

    /** What a translation command makes; zero at zero. Stalls read as the
     *  first point above them that moved (header). */
    [[nodiscard]] Geometry::Twist predict(double vxFraction, double vyFraction) const;

    /** What a translation command makes from rest: the points as measured,
     *  stalls included, so a command the table could not start the robot
     *  with predicts (almost) nothing. Below the first size in a column
     *  seen to move (after its last stall), it reads nothing, not a blend
     *  and not that point: the dead band is a step somewhere short of it.
     *  On 2026-11-16 a command halfway between a stall and a move (55 deg
     *  column, 0.5 a stall, 0.7 0.13 m/s) read as 0.07 m/s, and forward at
     *  0.1, below the smallest point measured (0.2, 0.19 m/s), read as
     *  0.19: the wheels were sent 0.1 and the robot stood both times.
     *  For "commanded but not moving" (design §19.4 step 4), which must not
     *  trip at the dead band, and the planner's nearly-still rule; never
     *  for the limiter, which wants predict(). */
    [[nodiscard]] Geometry::Twist predictFromRest(double vxFraction, double vyFraction) const;

    /**
     * The command along directionRad (command space) whose predicted speed
     * is as near speedMps as it can be without going over it.
     *
     * @return the vx and vy fractions; no value when even the smallest
     *         command along it is faster than speedMps
     */
    [[nodiscard]] std::optional<std::pair<double, double>> commandFor(double directionRad,
                                                                      double speedMps) const;

    /**
     * The command whose predicted motion comes nearest to want without
     * exceeding it: on each axis the same sign as want (or zero) and no
     * larger. Searched over directions every 5 degrees within withinRad of
     * aroundRad, and sizes every 0.05 of full.
     *
     * @param aroundRad the direction to stay near, command space: the stick's
     * @param withinRad how far either side of it to look
     * @param dead commands to pass over (isDead()): the caller gives them
     *        only with the robot nearly still
     * @return the vx and vy fractions; no value when no command near
     *         aroundRad moves the robot within want (the caller stops)
     */
    [[nodiscard]] std::optional<std::pair<double, double>> nearestCommand(
        Geometry::Vec2 want, double aroundRad, double withinRad = kSubstituteWidthRad,
        std::span<const DeadCommand> dead = {}) const;

    /** The least speed a command along directionRad moves the robot at,
     *  stalls lifted; 0 where no point either side moved. */
    [[nodiscard]] double leastSpeed(double directionRad) const;

    /** How many columns the points formed, mirrors included. */
    [[nodiscard]] std::size_t columns() const noexcept { return m_columns.size(); }

  private:
    struct Column {
      double directionRad{0.0};
      std::vector<MotionPoint> points;  // by size, rising
      std::size_t firstRead{0};         // the first point read, stalls lifted
    };

    [[nodiscard]] Geometry::Twist inColumn(const Column &column, double size, bool liftStalls) const;
    [[nodiscard]] Geometry::Twist read(double vxFraction, double vyFraction, bool liftStalls) const;

    std::vector<Column> m_columns;  // by direction, rising, in (-pi, pi]
};

}  // namespace Heron::Motion

#endif  // HERON_MOTION_MOTION_TABLE_H
