// File        : Planner.cpp
// Author      : Chris Pane
// Description : The local planner's core: candidates, the field, the score.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/planner/Planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "planner/DistanceField.h"

namespace Heron::Planner {

namespace {

// Round the outline's edge, no farther apart than this: half of it is in
// the cell slack.
constexpr double kEdgeSpacingM = 0.025;

// I12 lets a candidate through that brings nothing nearer; sliding along a
// wall reads the same distance give or take rounding, so allow that much.
constexpr double kSameM = 1e-9;

double dot(Geometry::Vec2 a, Geometry::Vec2 b) noexcept {
  return a.x * b.x + a.y * b.y;
}

/** How far b lies to the left of a, for a unit a: the distance off a line. */
double cross(Geometry::Vec2 a, Geometry::Vec2 b) noexcept {
  return a.x * b.y - a.y * b.x;
}

/** Points round the outline's edge, robot frame, at most kEdgeSpacingM apart. */
std::vector<Geometry::Vec2> edgeOf(const Geometry::Outline &o) {
  std::vector<Geometry::Vec2> edge;
  const auto along = [](double length) {
    return std::max(1, static_cast<int>(std::ceil(length / kEdgeSpacingM)));
  };
  const int nx = along(2.0 * o.halfLength);
  const int ny = along(2.0 * o.halfWidth);
  for (int i = 0; i < nx; ++i) {  // each side from one corner, not the next
    const double x = -o.halfLength + 2.0 * o.halfLength * i / nx;
    edge.push_back(Geometry::Vec2{-x, -o.halfWidth});
    edge.push_back(Geometry::Vec2{x, o.halfWidth});
  }
  for (int i = 0; i < ny; ++i) {
    const double y = -o.halfWidth + 2.0 * o.halfWidth * i / ny;
    edge.push_back(Geometry::Vec2{o.halfLength, y});
    edge.push_back(Geometry::Vec2{-o.halfLength, -y});
  }
  return edge;
}

/** The least field value round the edge, the robot at pose. */
double clearanceAt(const DistanceField &field, const std::vector<Geometry::Vec2> &edge,
                   const Geometry::Pose2 &pose) {
  const double c = std::cos(pose.heading);
  const double s = std::sin(pose.heading);
  double least = DistanceField::kFar;
  for (const Geometry::Vec2 &e : edge) {
    least = std::min(least, field.at(Geometry::Vec2{pose.x + c * e.x - s * e.y,
                                                    pose.y + s * e.x + c * e.y}));
  }
  return least;
}

/** One candidate, as imagined. */
struct Candidate {
  Geometry::Twist twist{};
  bool straight{false};
  double clearance{0.0};  // field's least over the steps, less the slack
  double progress{0.0};   // along the course heading, over the horizon
  double offLine{0.0};    // from the course line, at the end
  double way{0.0};        // how far it runs before coming nearer than preferred
  double toward{0.0};     // made toward the point on the line ahead
};

Plan stopped(Reason reason, const PlanInput &input) {
  Plan plan;
  plan.state = State::Stopped;
  plan.reason = reason;
  plan.courseSeq = input.course ? input.course->seq : 0;
  plan.time = input.now;
  return plan;
}

}  // namespace

Planner::Planner(const PlannerConfig &config) : m_config(config), m_edge(edgeOf(config.outline)) {}

void Planner::reset() noexcept {
  m_last.reset();
}

Plan Planner::plan(const PlanInput &input) {
  const PlannerConfig &cfg = m_config;

  // 1. Can it plan?
  Reason why = Reason::None;
  if (!input.course) {
    why = Reason::NoCourse;
  } else if (input.map == nullptr) {
    why = Reason::NoMap;
  } else if (input.now - input.map->scanEnd > cfg.maxMapAge) {
    why = Reason::StaleMap;
  } else if (!input.poseFresh) {
    why = Reason::PoseNotFresh;
  } else if (input.map->epoch != input.epoch || input.course->epoch != input.epoch) {
    why = Reason::OtherFrame;
  }
  if (why != Reason::None) {
    m_last.reset();
    return stopped(why, input);
  }
  const Course &course = *input.course;
  const Geometry::Pose2 &pose = input.pose;

  // 2. The field. What is under Heron and its margin now cannot be seen.
  const Geometry::Outline under = Geometry::grown(cfg.outline, cfg.stopMarginM + kCellSlackM);
  const DistanceField field(keepClear(input.map->grid, pose, under, cfg.unknownRadiusM),
                            input.map->grid.origin());
  const double clearNow = clearanceAt(field, m_edge, pose) - kCellSlackM;

  // 3. Candidates: the window round the measured speed, none faster than
  //    the course asks (I13), and the straight one.
  const double reach = cfg.accelMps2 * cfg.windowS;
  const double speedCap = std::min(course.speedMps, cfg.maxSpeedMps);
  const Geometry::Vec2 dirOdom{std::cos(course.headingRad), std::sin(course.headingRad)};
  const Geometry::Vec2 dir = Geometry::rotate(dirOdom, -pose.heading);  // robot frame
  std::vector<Candidate> candidates;  // the straight one first: it wins a tie
  std::vector<Candidate> grid;
  const int n = std::max(2, cfg.samples);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      const double vx = input.measured.vx - reach + 2.0 * reach * i / (n - 1);
      const double vy = input.measured.vy - reach + 2.0 * reach * j / (n - 1);
      if (std::hypot(vx, vy) <= speedCap + 1e-9) {
        grid.push_back(Candidate{Geometry::Twist{vx, vy, course.turnRateRadps}});
      }
    }
  }
  {
    // Along dir, as fast as the window reaches, up to the cap: the k with
    // |k dir - measured| <= reach on each axis.
    double lo = 0.0;
    double hi = speedCap;
    const double comps[2][2] = {{dir.x, input.measured.vx}, {dir.y, input.measured.vy}};
    for (const auto &a : comps) {
      if (std::abs(a[0]) < 1e-12) {
        if (std::abs(a[1]) > reach) {
          hi = -1.0;
        }
        continue;
      }
      const double k1 = (a[1] - reach) / a[0];
      const double k2 = (a[1] + reach) / a[0];
      lo = std::max(lo, std::min(k1, k2));
      hi = std::min(hi, std::max(k1, k2));
    }
    if (hi >= lo && hi > 0.0) {
      Candidate straight{Geometry::Twist{hi * dir.x, hi * dir.y, course.turnRateRadps}};
      straight.straight = true;
      candidates.push_back(straight);
    }
  }
  candidates.insert(candidates.end(), grid.begin(), grid.end());

  // 4. Imagine each.
  const int steps = std::max(1, static_cast<int>(std::ceil(cfg.horizonS / cfg.stepS - 1e-9)));
  const double dt = cfg.horizonS / steps;
  const Geometry::Vec2 from{pose.x, pose.y};
  const double offNow = std::abs(cross(dirOdom, from - course.anchor));
  const double offLimit = std::max(cfg.maxOffLineM, offNow);
  const double floor = std::min(cfg.stopMarginM, clearNow);  // I12
  // The free way ends where a candidate comes nearer anything than
  // preferred, or, already that near, nearer than Heron is now (as I12):
  // otherwise every way would end at once beside a box, and nothing would
  // stop the pull back to the line from cutting its corner.
  const double wayFloor = std::min(cfg.preferredClearanceM, clearNow);
  // Progress is scored toward the point on the line lookAheadM ahead of
  // where Heron is along it: on the line, straight on; off it, back to it
  // at an angle that eases as it nears, as a driver steers for a point
  // ahead. Measured along the course heading alone, angling back gains
  // too little in 1.5 s to beat going straight on.
  const double along = dot(from - course.anchor, dirOdom);
  const Geometry::Vec2 carrot = course.anchor + (along + cfg.lookAheadM) * dirOdom;
  const Geometry::Vec2 toCarrot =
      (1.0 / std::max(1e-9, Geometry::norm(carrot - from))) * (carrot - from);
  std::vector<Candidate> passed;
  for (Candidate &c : candidates) {
    const double speed = std::hypot(c.twist.vx, c.twist.vy);
    double least = DistanceField::kFar;
    bool ok = true;
    std::optional<double> wayEnds;  // where it first comes inside the margin
    Geometry::Pose2 end = pose;
    for (int k = 1; k <= steps && ok; ++k) {
      end = Geometry::integrate(pose, c.twist, dt * k);
      const double here = clearanceAt(field, m_edge, end) - kCellSlackM;
      least = std::min(least, here);
      ok = least >= floor - kSameM;
      if (!wayEnds && here < wayFloor - kSameM) {
        wayEnds = speed * dt * (k - 1);
      }
    }
    if (!ok) {
      continue;
    }
    const Geometry::Vec2 to{end.x, end.y};
    c.clearance = least;
    c.progress = dot(to - from, dirOdom);
    c.offLine = std::abs(cross(dirOdom, to - course.anchor));
    if (c.progress < cfg.minProgressM || c.offLine > offLimit + 1e-9) {
      continue;
    }
    // The free way, for the score: how far along its way, out to
    // lookAheadM whatever its speed, before it would come nearer anything
    // than preferredClearanceM (the distance term of the dynamic window
    // approach). Without it, two ways that both run into a box within the
    // look score alike, and a slow one that stops short of the box within
    // the horizon outscores going round it. Measured to the preferred
    // clearance (wayFloor), not the margin, so Heron keeps 15 cm where
    // there is room and uses the margin only where there is not.
    c.toward = dot(to - from, toCarrot);
    const double travelled = speed * cfg.horizonS;
    c.way = wayEnds ? *wayEnds : std::min(travelled, cfg.lookAheadM);
    if (!wayEnds && speed > 1e-3 && travelled < cfg.lookAheadM) {
      const double lookDt = kEdgeSpacingM / speed;
      const int more = static_cast<int>(std::ceil((cfg.lookAheadM - travelled) / kEdgeSpacingM));
      c.way = cfg.lookAheadM;
      for (int k = 1; k <= more; ++k) {
        const Geometry::Pose2 on = Geometry::integrate(pose, c.twist, cfg.horizonS + lookDt * k);
        if (clearanceAt(field, m_edge, on) - kCellSlackM < wayFloor - kSameM) {
          c.way = std::min(cfg.lookAheadM, travelled + kEdgeSpacingM * (k - 1));
          break;
        }
      }
    }
    passed.push_back(c);
  }

  Plan plan;
  plan.courseSeq = course.seq;
  plan.time = input.now;
  plan.candidatesTried = static_cast<int>(candidates.size());
  plan.candidatesPassed = static_cast<int>(passed.size());
  if (passed.empty()) {
    m_last.reset();
    plan.state = State::Blocked;
    plan.reason = Reason::NothingSafe;
    plan.twist = Geometry::Twist{0.0, 0.0, course.turnRateRadps};
    plan.clearanceM = clearNow;
    return plan;
  }

  // 5. Score, and choose.
  const ScoreWeights &w = cfg.weights;
  const double fullProgress = std::max(1e-6, speedCap * cfg.horizonS);
  const Candidate *best = nullptr;
  double bestScore = -std::numeric_limits<double>::infinity();
  for (const Candidate &c : passed) {
    double change = 0.0;
    if (m_last) {
      change = std::hypot(c.twist.vx - m_last->vx, c.twist.vy - m_last->vy) /
               (2.0 * cfg.maxSpeedMps);
    }
    const double clear = std::clamp(c.clearance, 0.0, cfg.clearanceCapM);
    const double score = w.clearance * clear / cfg.clearanceCapM + w.way * c.way / cfg.lookAheadM +
                         w.progress * c.toward / fullProgress -
                         w.offLine * c.offLine / cfg.maxOffLineM - w.change * change;
    if (score > bestScore) {
      bestScore = score;
      best = &c;
    }
  }
  m_last = best->twist;
  plan.twist = best->twist;
  plan.clearanceM = best->clearance;
  plan.state = best->straight && offNow <= cfg.onLineM ? State::Following : State::Avoiding;
  plan.reason = Reason::None;
  for (double t = 0.0; t <= cfg.horizonS + 1e-9; t += cfg.pathStepS) {
    plan.path.push_back(Geometry::integrate(pose, best->twist, t));
  }
  return plan;
}

const char *toString(State state) noexcept {
  switch (state) {
    case State::Following:
      return "FOLLOWING";
    case State::Avoiding:
      return "AVOIDING";
    case State::Blocked:
      return "BLOCKED";
    case State::Stopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

const char *toString(Reason reason) noexcept {
  switch (reason) {
    case Reason::None:
      return "NONE";
    case Reason::NothingSafe:
      return "NOTHING_SAFE";
    case Reason::NoCourse:
      return "NO_COURSE";
    case Reason::NoMap:
      return "NO_MAP";
    case Reason::StaleMap:
      return "STALE_MAP";
    case Reason::PoseNotFresh:
      return "POSE_NOT_FRESH";
    case Reason::OtherFrame:
      return "OTHER_FRAME";
  }
  return "UNKNOWN";
}

}  // namespace Heron::Planner
