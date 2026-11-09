// File        : PoseTest.cpp
// Author      : Chris Pane
// Description : heron_pose: the history's lookups, the feed's epoch and
//               freshness, and the tracker's thread on a fake OTOS.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The N3 row of docs/navigation-cpp-design.md §9: interpolation the short
// way round, extrapolation to 50 ms and not past it, a restart raises the
// epoch. The history and the feed are pure, so their times are made up and
// every check is one call; the tracker runs its thread on a real Odometry
// over the fake bus, as odometry_test does. Same style as the other tests:
// no framework.

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <numbers>
#include <thread>

#include "FakeOtosBus.h"
#include "heron/odometry/Odometry.h"
#include "heron/pose/PoseHistory.h"
#include "heron/pose/PoseTracker.h"
#include "pose/PoseFeed.h"

namespace {

using namespace Heron::Pose;
using Heron::Duration;
using Heron::TimePoint;
using Heron::Geometry::kDegToRad;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::wrapAngle;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b, double tolerance = 1e-9) {
  return std::abs(a - b) <= tolerance;
}

// Headings compared the short way round, so -pi and +pi agree.
bool near(const Pose2 &a, const Pose2 &b, double tolerance = 1e-9) {
  return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
         std::abs(wrapAngle(a.heading - b.heading)) <= tolerance;
}

// A reading as Odometry would hand it over: fresh, OK, at the poll.
Heron::Odometry::OdometrySnapshot reading(std::uint64_t seq, double x) {
  Heron::Odometry::OdometrySnapshot o;
  o.fresh = true;
  o.seq = seq;
  o.age = milliseconds(3);
  o.pose = Heron::Odometry::Planar{x, 0.0, 0.0};
  return o;
}

// As odometry_test: a new fake bus for each start().
struct Fakes {
  Heron::Odometry::FakeOtosBus *current{nullptr};

  Heron::Odometry::Odometry::BusFactory factory() {
    return [this]() -> std::unique_ptr<Heron::Odometry::I2cBus> {
      auto bus = std::make_unique<Heron::Odometry::FakeOtosBus>();
      current = bus.get();
      return bus;
    };
  }
};

}  // namespace

int main() {
  const TimePoint t0{std::chrono::seconds(100)};

  std::printf("PoseHistory: lookups\n");
  {
    PoseHistory history;
    const PoseLookup none = history.at(t0);
    check("empty: fails, Empty", !none.ok && none.why == LookupFailure::Empty);

    history.add(PoseSample{t0, Pose2{0.0, 0.0, 0.0}, Twist{}});
    history.add(PoseSample{t0 + milliseconds(10), Pose2{0.010, -0.004, 0.0}, Twist{}});
    const PoseLookup mid = history.at(t0 + milliseconds(5));
    check("halfway: the midpoint", mid.ok && near(mid.pose, Pose2{0.005, -0.002, 0.0}));
    const PoseLookup quarter = history.at(t0 + nanoseconds(2'500'000));
    check("a quarter of the way: a quarter of the move",
          quarter.ok && near(quarter.pose, Pose2{0.0025, -0.001, 0.0}));
    check("at a reading's own time: that reading",
          history.at(t0).ok && near(history.at(t0).pose, Pose2{}));
    const PoseLookup before = history.at(t0 - nanoseconds(1));
    check("1 ns before the oldest: fails, BeforeOldest",
          !before.ok && before.why == LookupFailure::BeforeOldest);

    history.add(PoseSample{t0 + milliseconds(5), Pose2{9.0, 9.0, 0.0}, Twist{}});
    history.add(PoseSample{t0 + milliseconds(10), Pose2{9.0, 9.0, 0.0}, Twist{}});
    check("a reading not later than the newest is ignored",
          history.size() == 2 && near(history.newest()->pose, Pose2{0.010, -0.004, 0.0}));
  }

  std::printf("PoseHistory: the heading, the short way round\n");
  {
    PoseHistory history;
    history.add(PoseSample{t0, Pose2{0.0, 0.0, 170.0 * kDegToRad}, Twist{}});
    history.add(PoseSample{t0 + milliseconds(10), Pose2{0.0, 0.0, -170.0 * kDegToRad}, Twist{}});
    const PoseLookup mid = history.at(t0 + milliseconds(5));
    std::printf("        170 to -170 deg, halfway: %.3f deg\n", mid.pose.heading / kDegToRad);
    check("170 to -170 deg, halfway: 180, not 0", mid.ok && near(mid.pose, Pose2{0.0, 0.0, std::numbers::pi}));
    const PoseLookup quarter = history.at(t0 + nanoseconds(2'500'000));
    check("a quarter of the way: 175", quarter.ok && near(quarter.pose.heading, 175.0 * kDegToRad));
    const PoseLookup late = history.at(t0 + nanoseconds(7'500'000));
    check("three quarters: -175, wrapped", late.ok && near(late.pose.heading, -175.0 * kDegToRad));

    PoseHistory back;
    back.add(PoseSample{t0, Pose2{0.0, 0.0, -10.0 * kDegToRad}, Twist{}});
    back.add(PoseSample{t0 + milliseconds(10), Pose2{0.0, 0.0, 10.0 * kDegToRad}, Twist{}});
    check("-10 to 10 deg, halfway: 0, not 180",
          back.at(t0 + milliseconds(5)).ok && near(back.at(t0 + milliseconds(5)).pose.heading, 0.0));
  }

  std::printf("PoseHistory: extrapolation, to 50 ms and not past it\n");
  {
    // Facing +y at 0.2 m/s forward: forward is +y in the odom frame.
    PoseHistory history;
    history.add(PoseSample{t0, Pose2{1.0, 2.0, 90.0 * kDegToRad}, Twist{0.2, 0.0, 0.0}});
    const PoseLookup at20 = history.at(t0 + milliseconds(20));
    check("20 ms on: 4 mm further along +y",
          at20.ok && near(at20.pose, Pose2{1.0, 2.004, 90.0 * kDegToRad}));
    const PoseLookup at50 = history.at(t0 + milliseconds(50));
    check("exactly 50 ms on: still extrapolated, 10 mm",
          at50.ok && near(at50.pose, Pose2{1.0, 2.010, 90.0 * kDegToRad}));
    const PoseLookup past = history.at(t0 + milliseconds(50) + nanoseconds(1));
    check("50 ms and 1 ns on: fails, PastExtrapolation",
          !past.ok && past.why == LookupFailure::PastExtrapolation);

    // Strafing left while turning: the exact arc, as integrate() gives.
    PoseHistory arc;
    const Twist twist{0.0, 0.3, 1.0};
    arc.add(PoseSample{t0, Pose2{}, twist});
    const PoseLookup curved = arc.at(t0 + milliseconds(40));
    const Pose2 expected = Heron::Geometry::integrate(Pose2{}, twist, 0.040);
    check("turning while strafing: on the arc", curved.ok && near(curved.pose, expected));
    check("  which is not the straight step",
          curved.ok && !near(curved.pose.x, 0.0, 1e-5) && near(curved.pose.heading, 0.040));
  }

  std::printf("PoseHistory: the ring\n");
  {
    // 300 readings 10 ms apart: the first 44 are overwritten.
    PoseHistory history;
    for (std::int32_t i = 0; i < 300; ++i) {
      history.add(PoseSample{t0 + milliseconds(10 * i), Pose2{0.001 * i, 0.0, 0.0}, Twist{}});
    }
    check("holds 256", history.size() == PoseHistory::kCapacity);
    check("the oldest is reading 44", near(history.oldest()->pose.x, 0.044));
    check("reading 43's time: fails, BeforeOldest",
          history.at(t0 + milliseconds(430)).why == LookupFailure::BeforeOldest);
    bool allRight = true;
    for (std::int32_t i = 44; i < 299; ++i) {
      const PoseLookup p = history.at(t0 + milliseconds(10 * i + 5));
      allRight = allRight && p.ok && near(p.pose.x, 0.001 * i + 0.0005);
    }
    check("every gap interpolated, across the ring's seam", allRight);
    history.clear();
    check("clear(): empty", history.size() == 0 && !history.at(t0).ok);
  }

  std::printf("PoseFeed: the epoch, the gap and freshness\n");
  {
    PoseFeed feed(PoseConfig{});
    feed.update(Heron::Odometry::OdometrySnapshot{}, t0);
    check("before the first reading: epoch 0, not fresh",
          feed.epoch() == 0 && !feed.snapshot().fresh && !feed.snapshot().reason.empty());

    feed.update(reading(1, 0.0), t0);
    check("the first reading opens epoch 1, fresh",
          feed.epoch() == 1 && feed.snapshot().fresh && feed.snapshot().reason.empty());
    check("stored at the poll's time minus its age",
          feed.history().newest()->time == t0 - milliseconds(3));

    feed.update(reading(1, 0.0), t0 + milliseconds(10));
    check("the same seq again: nothing added", feed.history().size() == 1);
    feed.update(reading(12, 0.1), t0 + milliseconds(20));
    check("10 readings missed: still fresh", feed.snapshot().fresh);
    feed.update(reading(24, 0.2), t0 + milliseconds(30));
    std::printf("        reason: \"%s\"\n", feed.snapshot().reason.c_str());
    check("11 missed: not fresh, says so",
          !feed.snapshot().fresh && feed.snapshot().reason == "missed 11 odometry readings");
    feed.update(reading(25, 0.3), t0 + milliseconds(40));
    check("the next reading in step: fresh again", feed.snapshot().fresh);
    check("the pose follows", near(feed.snapshot().newest.pose.x, 0.3));

    // Odometry restarted: seq starts again from 1, in a new frame.
    feed.update(reading(1, 5.0), t0 + milliseconds(50));
    check("a lower seq: epoch 2", feed.epoch() == 2 && feed.snapshot().epoch == 2);
    check("  the old frame's poses are gone",
          feed.history().size() == 1 && near(feed.history().oldest()->pose.x, 5.0));
    check("  and the restart is not a gap", feed.snapshot().fresh);

    Heron::Odometry::OdometrySnapshot stale = reading(2, 5.1);
    stale.fresh = false;
    feed.update(stale, t0 + milliseconds(60));
    check("Odometry stale: not fresh, says so",
          !feed.snapshot().fresh && feed.snapshot().reason == "odometry stale");

    Heron::Odometry::OdometrySnapshot broken = reading(3, 99.0);
    broken.status = 0x80;  // IMU error
    feed.update(broken, t0 + milliseconds(70));
    check("a sensor error: not fresh, and its pose not stored",
          !feed.snapshot().fresh && feed.snapshot().reason == "odometry sensor error" &&
              near(feed.history().newest()->pose.x, 5.1));

    Heron::Odometry::OdometrySnapshot tilted = reading(4, 5.2);
    tilted.status = 0x03;  // tilt and tracking
    feed.update(tilted, t0 + milliseconds(80));
    check("warnings passed on, still fresh",
          feed.snapshot().fresh && feed.snapshot().tiltWarning && feed.snapshot().trackingWarning);
  }

  std::printf("PoseTracker: the thread, on a fake OTOS\n");
  {
    Fakes fakes;
    Heron::Odometry::Odometry odometry(Heron::Odometry::OtosConfig{}, fakes.factory());
    PoseTracker tracker(odometry);
    check("start() before Odometry: fine", tracker.start());
    check("  nothing yet, epoch 0", tracker.getLatest().epoch == 0 && !tracker.getLatest().fresh);

    check("Odometry starts", odometry.start());
    std::this_thread::sleep_for(milliseconds(150));
    const PoseSnapshot first = tracker.getLatest();
    const auto [history, epoch] = tracker.copyHistory();
    std::printf("        after 150 ms: epoch %" PRIu64 ", %zu readings kept\n", epoch,
                history.size());
    check("fresh, in epoch 1", first.fresh && first.epoch == 1 && epoch == 1);
    check("readings kept, about 100 a second", history.size() >= 5 && history.size() <= 25);
    check("a lookup at the newest reading works",
          history.at(history.newest()->time).ok);

    odometry.stop();
    check("Odometry restarts", odometry.start());
    std::this_thread::sleep_for(milliseconds(100));
    const auto [after, epochAfter] = tracker.copyHistory();
    check("a restart raises the epoch", epochAfter == 2 && tracker.getLatest().epoch == 2);
    check("  and every reading kept is the new run's",
          after.oldest().has_value() && after.oldest()->time > history.newest()->time);
    tracker.stop();
    tracker.stop();
    check("stop() twice: fine", true);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
