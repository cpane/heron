// File        : Limiter.cpp
// Author      : Chris Pane
// Description : The safety limiter: placing the points, the horizon, and the
//               rule per axis.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/limiter/Limiter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "heron/lidar/SectorCheck.h"

namespace Heron::Limiter {

namespace {

using Geometry::Outline;
using Geometry::Twist;
using Geometry::Vec2;

// Slower than this, an axis counts as not commanded; and a point whose
// depth changes slower than this is not being driven into.
constexpr double kStill = 1e-6;

// While something is in the stop margin, the sweep's outline stops this
// short of the closest point (rule 4).
constexpr double kSlackM = 0.001;

// A gap this little below the closest still counts as level with it
// (closesIn()), so float noise on a point beside a wall cannot decide it.
constexpr double kSameGapM = 0.001;

// How wide a sector must have been observed round a direction of travel:
// the outline's front spans a wide angle close up.
constexpr float kTravelHalfWidthRad = 0.7853982f;  // 45 degrees

double seconds(Duration d) noexcept {
  return std::chrono::duration<double>(d).count();
}

double halfDiagonal(const Outline &o) noexcept {
  return std::hypot(o.halfLength, o.halfWidth);
}

/** An upper bound on how fast any part of the outline moves, m/s. */
double outlineSpeed(const Twist &t, double halfDiag) noexcept {
  return std::hypot(t.vx, t.vy) + std::abs(t.wz) * halfDiag;
}

/** How far outside the outline p is; 0 inside. */
double gapTo(const Outline &o, Vec2 p) noexcept {
  return std::hypot(std::max(std::abs(p.x) - o.halfLength, 0.0),
                    std::max(std::abs(p.y) - o.halfWidth, 0.0));
}

/**
 * How far outside the outline p is, measured as grown() grows it: p is
 * inside grown(o, m) exactly when this is m or less. Negative inside.
 */
double rectGap(const Outline &o, Vec2 p) noexcept {
  return std::max(std::abs(p.x) - o.halfLength, std::abs(p.y) - o.halfWidth);
}

/**
 * Does the motion bring p, a point in the stop margin, closer to the body
 * than `closest`, the closest anything is now? A point's distance is
 * rectGap(), the larger of its gaps along x and y. A motion that shrinks
 * only one of them can bring the point no nearer than the other, which it
 * leaves alone; so it counts only if that other gap is below `closest`.
 * Driving into a wall shrinks the gap along x of the points in front of
 * the body, whose gap along y is below zero: stopped. Sliding along it
 * shrinks the gap along y of the wall's points past the side, whose gap
 * along x is the wall's own distance: allowed. A motion that shrinks both
 * counts always.
 *
 * Not linear in the twist: two axes each allowed alone can shrink both
 * gaps together, so limit() tests the cut motion as a whole again.
 */
bool closesIn(const Outline &o, Vec2 p, const Twist &t, double closest) noexcept {
  // The point's velocity in the robot frame, as the robot moves.
  const double vx = -t.vx + t.wz * p.y;
  const double vy = -t.vy - t.wz * p.x;
  const double gx = std::abs(p.x) - o.halfLength;
  const double gy = std::abs(p.y) - o.halfWidth;
  const bool shrinksX = (p.x < 0.0 ? -1.0 : 1.0) * vx < -kStill;
  const bool shrinksY = (p.y < 0.0 ? -1.0 : 1.0) * vy < -kStill;
  if (shrinksX && shrinksY) {
    return true;
  }
  if (!shrinksX && !shrinksY) {
    return false;
  }
  const double floor = shrinksX ? gy : gx;
  return floor < closest - kSameGapM;
}

/**
 * The earliest contact of the outline with any point, within horizonS.
 * Points the motion cannot reach are skipped before the sweep: the outline
 * stays within its half-diagonal of its centre, which moves at most |v| T.
 */
std::optional<double> earliest(const std::vector<Vec2> &points, const Outline &o,
                               const Twist &t, double horizonS,
                               const Geometry::SweepStep &step) {
  const double reach = halfDiagonal(o) + std::hypot(t.vx, t.vy) * horizonS + 0.001;
  const double reachSq = reach * reach;
  const Geometry::Sweep sweep(o, t, horizonS, step);
  std::optional<double> best;
  for (const Vec2 &p : points) {
    if (p.x * p.x + p.y * p.y > reachSq) {
      continue;
    }
    // Each contact found shortens the search for the next.
    if (const auto c = sweep.contact(p, best ? *best : horizonS)) {
      best = c;
    }
  }
  return best;
}

/** Heron's pose at one moment of the scan, seen from its pose now. */
struct Motion {
  Geometry::Pose2 pose{};
  bool ok{false};
};

// The moments a revolution is cut into for moving its points (limit()).
constexpr std::size_t kMotionSteps = 64;

/** How one axis came out. */
struct AxisLimit {
  double scale{1.0};
  bool cut{false};
  bool slowed{false};
  bool unobserved{false};
};

/** Where the scan's points are now, and what each is tested with. */
struct Placed {
  std::vector<Vec2> swept;     // outside the stop margin: tested by the sweep, against `sweep`
  std::vector<Vec2> held;      // inside the stop margin: tested by closesIn()
  Outline sweep{};
  std::optional<Vec2> nearest;
};


}  // namespace

Limiter::Limiter(const LimiterConfig &config) : m_config(config) {}

LimitResult Limiter::limit(const LimitInput &input) const {
  LimitResult result;
  const auto stopAll = [&result](Reason why) {
    result.command = Twist{};
    result.scale = Twist{0.0, 0.0, 0.0};
    result.reason = why;
    return result;
  };

  // 1. Can it see? (design §7.4)
  const Heron::Scan *scan = input.scan;
  if (scan == nullptr) {
    return stopAll(Reason::NoScan);
  }
  const Duration age = input.now - scan->tEnd;
  if (age > m_config.maxScanAge) {
    return stopAll(Reason::Stale);
  }
  if (!scan->isCoverageOk()) {
    return stopAll(Reason::NotCovered);
  }

  // 2. Where the points are now. Moved for Heron's motion only when the
  // pose is fresh and known now; otherwise left where they were seen.
  std::optional<Geometry::Pose2> poseNow;
  if (input.poseFresh && input.history != nullptr) {
    const Pose::PoseLookup now = input.history->at(input.now);
    if (now.ok) {
      poseNow = now.pose;
    }
  }

  // 3. The horizon: the time to stop, and the time the limiter is blind.
  double reactionS = std::max(seconds(age), 0.0) + seconds(m_config.scanPeriod);
  if (!poseNow) {
    reactionS *= 2.0;
  }
  const double speed = std::hypot(input.measured.vx, input.measured.vy);
  const double horizonS =
      (m_config.brakeMps2 > 0.0 ? speed / m_config.brakeMps2 : 0.0) + reactionS;
  result.horizonS = horizonS;

  const Outline stop = Geometry::grown(m_config.outline, m_config.stopMarginM);
  const double halfDiag = halfDiagonal(stop);
  const Twist &command = input.command;
  // No part of any scaled command reaches past this: the outline stays
  // within halfDiag of its centre, which moves at most |v| T, and the slow
  // zone looks slowMargin along the path.
  const double reach = halfDiag + std::max(std::hypot(command.vx, command.vy) * horizonS,
                                           m_config.slowMarginM) + m_config.stopMarginM;

  // How Heron has moved since each moment of the revolution: the pose then,
  // seen from the pose now. Worked out at kMotionSteps + 1 moments and
  // interpolated for each point, rather than looked up per point: a lookup
  // and two rotations per point were most of a call's time. relative() is
  // linear in the pose's x, y and heading, so interpolating it is
  // interpolating the pose, which PoseHistory::at() does anyway between
  // readings. The one difference is a reading that falls between two of
  // these moments, 2 ms apart: a change of velocity over 2 ms, far below a
  // millimetre.
  std::array<Motion, kMotionSteps + 1> since{};
  const double spanS = seconds(scan->tEnd - scan->tStart);
  if (poseNow) {
    for (std::size_t k = 0; k <= kMotionSteps; ++k) {
      const double at = spanS * static_cast<double>(k) / kMotionSteps;
      const Pose::PoseLookup then = input.history->at(
          scan->tStart + std::chrono::duration_cast<Duration>(std::chrono::duration<double>(at)));
      if (then.ok) {
        since[k] = Motion{Geometry::relative(*poseNow, then.pose), true};
      }
    }
  }

  // Every point now, robot frame.
  std::vector<Vec2> points;
  points.reserve(scan->points.size());
  for (const Heron::Point &point : scan->points) {
    Vec2 p = Geometry::polarToRobot(point.bearingRad, point.rangeM, m_config.lidarOffset);
    if (poseNow) {
      const double f = spanS > 0.0
                           ? seconds(scan->timeOf(point.bearingRad) - scan->tStart) / spanS *
                                 kMotionSteps
                           : 0.0;
      const auto k = static_cast<std::size_t>(
          std::clamp(std::floor(f), 0.0, static_cast<double>(kMotionSteps - 1)));
      const Motion &a = since[k];
      const Motion &b = since[k + 1];
      if (a.ok && b.ok) {
        const double u = std::clamp(f - static_cast<double>(k), 0.0, 1.0);
        const Geometry::Pose2 moved{
            a.pose.x + u * (b.pose.x - a.pose.x), a.pose.y + u * (b.pose.y - a.pose.y),
            a.pose.heading + u * Geometry::wrapAngle(b.pose.heading - a.pose.heading)};
        p = Geometry::toParent(moved, p);
      }
    }
    points.push_back(p);
  }

  // Something already inside the stop margin (rule 4). Every point in the
  // margin is held by closesIn(): Heron may move along it or away, never
  // closer. The points outside it are swept against the body grown by the
  // closest distance, not by the margin: a point coming in from beside,
  // as the rest of a wall does when Heron slides along it, may then enter
  // the margin as deep as what is already there, and once in, is held
  // too. So the closest distance never shrinks (but by sensor noise), and
  // a slide along a wall in the margin goes on.
  //
  // The first version held only the closest points and swept the rest
  // against a margin 1 mm short of them. That margin ratcheted: driven at
  // a wall at 55 deg, each wall point further along reached the shrunk
  // margin, became the closest, and shrank it again, and Heron crept into
  // the wall (nav_scenario_test, scenario 2).
  const Outline &body = m_config.outline;
  double closestGap = m_config.stopMarginM;
  for (const Vec2 &p : points) {
    closestGap = std::min(closestGap, rectGap(body, p));
  }
  const bool tooClose = closestGap < m_config.stopMarginM;
  Placed placed;
  // kSlackM short of the closest, so a point beside it, level with it, is
  // not touched by the edge: a contact on the edge counts. A point entering
  // that slack is held from then on, so the slack cannot ratchet. Never
  // shrunk past nothing: a point inside the body itself is a collision
  // already, or a ghost, and is held by closesIn() alone.
  const double floor = -0.9 * std::min(body.halfLength, body.halfWidth);
  placed.sweep = tooClose ? Geometry::grown(body, std::max(closestGap - kSlackM, floor)) : stop;
  double nearestGap = 0.0;
  for (const Vec2 &p : points) {
    const double gap = gapTo(body, p);
    if (!placed.nearest || gap < nearestGap) {
      placed.nearest = p;
      nearestGap = gap;
    }
    // In the margin, edge included, as contains() counts it.
    if (rectGap(body, p) <= m_config.stopMarginM) {
      placed.held.push_back(p);
    } else if (Geometry::norm(p) <= reach) {
      placed.swept.push_back(p);
    }
  }
  result.nearest = placed.nearest;

  // 4. The rule, each axis alone (design §7.3).
  const auto limitAxis = [&](const Twist &alone, float travelRad) {
    AxisLimit a;
    if (outlineSpeed(alone, halfDiag) < kStill) {
      return a;
    }
    // Its direction looked at? A turn needs every direction.
    const bool turning = alone.wz != 0.0;
    const bool observed =
        turning ? scan->unobserved.empty()
                : Heron::Lidar::isSectorObserved(*scan,
                                                 Heron::Lidar::Sector{travelRad, kTravelHalfWidthRad});
    if (!observed) {
      a.scale = 0.0;
      a.unobserved = true;
      return a;
    }
    // Already too close: only motion that brings nothing in the margin closer.
    for (const Vec2 &p : placed.held) {
      if (closesIn(body, p, alone, closestGap)) {
        a.scale = 0.0;
        a.cut = true;
        return a;
      }
    }
    // Cut: the whole axis clear for T, or scaled to reach contact at T.
    if (const auto t = earliest(placed.swept, placed.sweep, alone, horizonS, m_config.step)) {
      a.scale = *t / horizonS;
      a.cut = true;
    }
    // Slow: the distance travelled before contact, against slowMargin.
    const double travelSpeed = outlineSpeed(alone, halfDiag);
    if (m_config.slowMarginM > 0.0) {
      const double slowHorizonS = m_config.slowMarginM / travelSpeed;
      if (const auto t = earliest(placed.swept, placed.sweep, alone, slowHorizonS, m_config.step)) {
        const double slow = *t * travelSpeed / m_config.slowMarginM;
        if (slow < a.scale) {
          a.scale = slow;
          a.slowed = !a.cut;
        }
      }
    }
    a.scale = std::clamp(a.scale, 0.0, 1.0);
    return a;
  };

  const std::array<AxisLimit, 3> axes{
      limitAxis(Twist{command.vx, 0.0, 0.0}, command.vx >= 0.0 ? 0.0f : -Heron::kPi),
      limitAxis(Twist{0.0, command.vy, 0.0}, command.vy >= 0.0 ? Heron::kPi / 2.0f
                                                               : -Heron::kPi / 2.0f),
      limitAxis(Twist{0.0, 0.0, command.wz}, 0.0f)};

  // Then the cut motion together, as a whole.
  const Twist cut{command.vx * axes[0].scale, command.vy * axes[1].scale,
                  command.wz * axes[2].scale};
  double whole = 1.0;
  if (const auto t = earliest(placed.swept, placed.sweep, cut, horizonS, m_config.step)) {
    whole = std::clamp(*t / horizonS, 0.0, 1.0);
  }
  // Two axes each allowed alone may bring a point in the margin closer
  // together, shrinking its gaps along x and y at once: then nothing.
  for (const Vec2 &p : placed.held) {
    if (closesIn(body, p, cut, closestGap)) {
      whole = 0.0;
      break;
    }
  }
  result.command = Twist{cut.vx * whole, cut.vy * whole, cut.wz * whole};
  result.scale = Twist{axes[0].scale * whole, axes[1].scale * whole, axes[2].scale * whole};

  const auto any = [&axes](bool AxisLimit::*flag) {
    return std::any_of(axes.begin(), axes.end(), [flag](const AxisLimit &a) { return a.*flag; });
  };
  if (any(&AxisLimit::unobserved)) {
    result.reason = Reason::Unobserved;
  } else if (tooClose) {
    result.reason = Reason::BackingOff;
  } else if (any(&AxisLimit::cut) || whole < 1.0) {
    result.reason = Reason::Cut;
  } else if (any(&AxisLimit::slowed)) {
    result.reason = Reason::Slowed;
  } else {
    result.reason = Reason::Clear;
  }
  return result;
}

std::string checkBlindSpot(const LimiterConfig &config) {
  const Outline stop = Geometry::grown(config.outline, config.stopMarginM);
  const double reachX = std::abs(config.lidarOffset.x) + kBlindRadiusM;
  const double reachY = std::abs(config.lidarOffset.y) + kBlindRadiusM;
  if (reachX <= stop.halfLength && reachY <= stop.halfWidth) {
    return {};
  }
  char text[320];
  std::snprintf(text, sizeof(text),
                "the LiDAR's blind circle (%.1f mm round its spin axis at %.3f, %.3f m) reaches "
                "%.3f m along x and %.3f m along y, past the stop outline (%.3f by %.3f m each "
                "way): something there could not be seen. Lengthen or widen the outline, or "
                "mount the LiDAR further in",
                kBlindRadiusM * 1000.0, config.lidarOffset.x, config.lidarOffset.y, reachX, reachY,
                stop.halfLength, stop.halfWidth);
  return text;
}

const char *toString(Reason reason) noexcept {
  switch (reason) {
    case Reason::Clear:
      return "CLEAR";
    case Reason::Cut:
      return "CUT";
    case Reason::Slowed:
      return "SLOWED";
    case Reason::BackingOff:
      return "BACKING_OFF";
    case Reason::NoScan:
      return "NO_SCAN";
    case Reason::Stale:
      return "STALE";
    case Reason::NotCovered:
      return "NOT_COVERED";
    case Reason::Unobserved:
      return "UNOBSERVED";
  }
  return "UNKNOWN";
}

}  // namespace Heron::Limiter
