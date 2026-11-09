// File        : PoseHistory.cpp
// Author      : Chris Pane
// Description : The pose history: a ring of readings, interpolated and
//               extrapolated.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/pose/PoseHistory.h"

namespace Heron::Pose {

namespace {

double seconds(Duration d) noexcept {
  return std::chrono::duration<double>(d).count();
}

PoseLookup found(const Geometry::Pose2 &pose) noexcept {
  return PoseLookup{true, LookupFailure::None, pose};
}

PoseLookup failed(LookupFailure why) noexcept {
  return PoseLookup{false, why, Geometry::Pose2{}};
}

/** a, moved toward b by fraction f; the heading the short way round. */
Geometry::Pose2 between(const Geometry::Pose2 &a, const Geometry::Pose2 &b, double f) noexcept {
  const double turn = Geometry::wrapAngle(b.heading - a.heading);
  return Geometry::Pose2{a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                         Geometry::wrapAngle(a.heading + f * turn)};
}

}  // namespace

void PoseHistory::add(const PoseSample &sample) noexcept {
  if (m_count > 0 && sample.time <= nth(m_count - 1).time) {
    return;
  }
  m_ring[m_next] = sample;
  m_next = (m_next + 1) % kCapacity;
  if (m_count < kCapacity) {
    ++m_count;
  }
}

void PoseHistory::clear() noexcept {
  m_next = 0;
  m_count = 0;
}

const PoseSample &PoseHistory::nth(std::size_t i) const noexcept {
  return m_ring[(m_next + kCapacity - m_count + i) % kCapacity];
}

std::optional<PoseSample> PoseHistory::newest() const noexcept {
  if (m_count == 0) {
    return std::nullopt;
  }
  return nth(m_count - 1);
}

std::optional<PoseSample> PoseHistory::oldest() const noexcept {
  if (m_count == 0) {
    return std::nullopt;
  }
  return nth(0);
}

PoseLookup PoseHistory::at(TimePoint t) const noexcept {
  if (m_count == 0) {
    return failed(LookupFailure::Empty);
  }
  if (t < nth(0).time) {
    return failed(LookupFailure::BeforeOldest);
  }
  const PoseSample &last = nth(m_count - 1);
  if (t >= last.time) {
    if (t - last.time > kMaxExtrapolation) {
      return failed(LookupFailure::PastExtrapolation);
    }
    // The velocity is in the robot frame, so the exact arc, not a straight
    // step: integrate() turns it with the heading as it goes.
    return found(Geometry::integrate(last.pose, last.velocity, seconds(t - last.time)));
  }

  // The first reading later than t, by bisection; the one before it is at
  // or before t, as nth(0) is.
  std::size_t lo = 0;
  std::size_t hi = m_count - 1;
  while (hi - lo > 1) {
    const std::size_t mid = lo + (hi - lo) / 2;
    if (nth(mid).time <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  const PoseSample &a = nth(lo);
  const PoseSample &b = nth(hi);
  const double f = seconds(t - a.time) / seconds(b.time - a.time);
  return found(between(a.pose, b.pose, f));
}

}  // namespace Heron::Pose
