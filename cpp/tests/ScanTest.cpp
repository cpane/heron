// File        : ScanTest.cpp
// Author      : Chris Pane
// Description : Compile-and-behave check for the Scan boundary type.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Deliberately not a test framework. Picking one is its own decision, and
// this exists mainly so the header is actually compiled by the build -- an
// interface nothing includes rots silently, and this one is meant to be
// argued with and changed.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "heron/lidar/Scan.h"

namespace {

std::int32_t s_failures = 0;

using Heron::kPi;
using Heron::kTwoPi;
constexpr float kDeg = kPi / 180.0f;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

// Widest gap between consecutive VALID points, closing the circle. Not part
// of the type, on purpose: it is the wrong measure of coverage, and exists
// here only to show that it is.
float widestValidGap(const Heron::Scan &scan) {
  const std::vector<Heron::Point> &p = scan.points;
  if (p.size() < 2) {
    return kTwoPi;
  }
  float widest = p.front().bearingRad + kTwoPi - p.back().bearingRad;
  for (std::size_t i = 1; i < p.size(); ++i) {
    widest = std::max(widest, p[i].bearingRad - p[i - 1].bearingRad);
  }
  return widest;
}

// Shaped like revolution 0 of captures/room_20260916T025707Z: 263 samples,
// 82 valid, widest sampling gap 8.672 deg. The returns come in clumps --
// walls and furniture -- with long runs of no-return between, which is what
// a real room looks like and what the first draft of this test did not
// model.
Heron::Scan healthyRoom() {
  Heron::Scan scan;
  scan.seq = 42;
  scan.tStart = Heron::Clock::now();
  scan.tEnd = scan.tStart + std::chrono::microseconds(134382);
  scan.rawSampleCount = 263;
  scan.worstSamplingGapRad = 8.672f * kDeg;
  // The sync flag fires at ~358.6 deg in the sensor frame; negated, that is
  // just left of straight ahead.
  scan.sweepStartRad = Heron::wrapBearing(-358.6f * kDeg);

  // Four clumps of returns, 1.4 deg apart, 90 deg apart from each other.
  for (std::int32_t clump = 0; clump < 4; ++clump) {
    const std::int32_t n = clump < 2 ? 21 : 20;  // 82 in all
    for (std::int32_t i = 0; i < n; ++i) {
      const float deg =
          -170.0f + 90.0f * static_cast<float>(clump) + 1.4f * static_cast<float>(i);
      scan.points.push_back({deg * kDeg, 1.5f});
    }
  }
  return scan;
}

}  // namespace

int main() {
  std::printf("Scan boundary type\n");

  {
    const Heron::Scan scan;
    check("default is empty", scan.isEmpty() && scan.size() == 0);
    check("default has no coverage", !scan.isCoverageOk());
    check("default observed nothing", !scan.isObserved(0.0f));
    check("default reports no rate", scan.rateHz() == 0.0f);
    check("default reports no invalid fraction", scan.invalidFraction() == 0.0f);
  }

  {
    const Heron::Scan scan = healthyRoom();
    check("rate matches a ~134 ms revolution", std::fabs(scan.rateHz() - 7.44f) < 0.05f);
    check("invalid fraction matches 82 valid of 263",
          std::fabs(scan.invalidFraction() - 0.688f) < 0.01f);

    // The bug this replaced: on valid points alone, a healthy room has a
    // ~60 deg hole and would have been reported as data loss.
    check("valid-point gaps are wide on a healthy revolution",
          widestValidGap(scan) > 30.0f * kDeg);
    check("...yet coverage is ok, because it is measured on all samples", scan.isCoverageOk());
    check("every bearing observed when no arc is unobserved",
          scan.isObserved(0.0f) && scan.isObserved(3.1f) && scan.isObserved(-3.1f));
    check("a caller can be stricter than the default", !scan.isCoverageOk(8.0f * kDeg));
  }

  {
    // Looked everywhere, nothing in range: full coverage, zero points.
    Heron::Scan scan = healthyRoom();
    scan.points.clear();
    check("all no-return is empty", scan.isEmpty());
    check("all no-return still has coverage", scan.isCoverageOk());
    check("all no-return reports invalid fraction 1", scan.invalidFraction() == 1.0f);
  }

  {
    // Overrun, as measured: a 174 deg hole. Placed so it straddles the seam
    // behind the robot -- from +60 deg round through 180 to -126.
    Heron::Scan scan = healthyRoom();
    scan.worstSamplingGapRad = 174.2f * kDeg;
    scan.unobserved.push_back({60.0f * kDeg, 174.2f * kDeg});

    check("overrun fails coverage, despite plenty of points",
          !scan.isCoverageOk() && scan.size() > 80);
    check("caller can widen the threshold", scan.isCoverageOk(3.1f));
    check("straight ahead was observed", scan.isObserved(0.0f));
    check("left of the hole start was observed", scan.isObserved(59.0f * kDeg));
    check("the arc's own start sample was observed", scan.isObserved(60.0f * kDeg));
    check("inside the hole, positive side, was not", !scan.isObserved(100.0f * kDeg));
    check("inside the hole, across the seam, was not", !scan.isObserved(-150.0f * kDeg));
    check("past the hole's end was observed", scan.isObserved(-120.0f * kDeg));
  }

  {
    // The half-open range. Each of these is a way a hand-rolled wrap lands on
    // exactly +pi, or outside the range altogether.
    using Heron::wrapBearing;
    check("wrap: +pi folds to -pi", wrapBearing(kPi) == -kPi);
    check("wrap: -pi stays -pi", wrapBearing(-kPi) == -kPi);
    check("wrap: 3pi folds to -pi", wrapBearing(3.0f * kPi) == -kPi);
    check("wrap: just under -pi lands just under +pi, not on it",
          wrapBearing(std::nextafter(-kPi, -4.0f)) < kPi);
    check("wrap: sensor 90 deg clockwise is robot -90",
          std::fabs(wrapBearing(-90.0f * kDeg) + 90.0f * kDeg) < 1e-6f);

    bool allInRange = true;
    for (std::int32_t i = -20000; i <= 20000; ++i) {
      const float r = wrapBearing(static_cast<float>(i) * 0.001f);
      if (!(r >= -kPi && r < kPi)) {
        allInRange = false;
      }
    }
    for (float x = -kPi - 1e-5f; x < -kPi + 1e-5f; x = std::nextafter(x, 0.0f)) {
      const float r = wrapBearing(x);
      if (!(r >= -kPi && r < kPi)) {
        allInRange = false;
      }
    }
    check("wrap: every result in [-pi, +pi), including every float near the seam",
          allInRange);
  }

  {
    // Time runs toward DECREASING robot-frame bearing, from the sweep start,
    // one full turn between tStart and tEnd.
    Heron::Scan scan;
    scan.tStart = Heron::Clock::now();
    scan.tEnd = scan.tStart + std::chrono::milliseconds(136);
    scan.sweepStartRad = 0.0f;
    const auto msAfterStart = [&](float bearingDeg) {
      const Heron::Duration t = scan.timeOf(bearingDeg * kDeg) - scan.tStart;
      return std::chrono::duration<float, std::milli>(t).count();
    };
    check("timeOf: the sweep start is tStart", msAfterStart(0.0f) == 0.0f);
    check("timeOf: a quarter turn clockwise (robot -90) is a quarter span",
          std::fabs(msAfterStart(-90.0f) - 34.0f) < 0.01f);
    check("timeOf: robot +90 is three quarters, not one quarter",
          std::fabs(msAfterStart(90.0f) - 102.0f) < 0.01f);
    check("timeOf: just left of the start is nearly a full turn later",
          msAfterStart(1.0f) > 135.0f && msAfterStart(1.0f) < 136.0f);

    // A start that is not 0 -- the real case -- across the seam.
    scan.sweepStartRad = Heron::wrapBearing(-358.6f * kDeg);  // +1.4 deg
    check("timeOf: with a real sweep start, straight ahead is ~0.5 ms in",
          std::fabs(msAfterStart(0.0f) - 136.0f * 1.4f / 360.0f) < 0.01f);
    check("timeOf: behind the robot is half a turn in, either side of the seam",
          std::fabs(msAfterStart(179.0f) - 136.0f * 182.4f / 360.0f) < 0.05f &&
              std::fabs(msAfterStart(-179.0f) - 136.0f * 180.4f / 360.0f) < 0.05f);
  }

  {
    // Bearing convention: forward arc must be a single |bearing| test.
    Heron::Scan scan;
    for (std::int32_t i = 0; i < 360; ++i) {
      const float deg = -180.0f + static_cast<float>(i);
      scan.points.push_back({deg * kDeg, 1.5f});
    }
    const auto inForward60 = [](const Heron::Point &p) {
      return std::fabs(p.bearingRad) < 30.0f * kDeg;
    };
    const std::ptrdiff_t forward =
        std::count_if(scan.points.begin(), scan.points.end(), inForward60);
    check("forward arc is one comparison, and selects 1/6 of a sweep", forward == 59);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
