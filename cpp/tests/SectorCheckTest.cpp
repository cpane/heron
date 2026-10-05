// File        : SectorCheckTest.cpp
// Author      : Chris Pane
// Description : Behaviour of checkSector() and isSectorObserved().
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "heron/lidar/SectorCheck.h"

namespace {

using Heron::kPi;
using Heron::Lidar::Sector;
using Heron::Lidar::SectorReason;

std::int32_t s_failures = 0;
constexpr float kDeg = kPi / 180.0f;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

// A healthy, freshly ended revolution: a point every degree at 3 m, plus
// whatever the test adds.
Heron::Scan freshScan(Heron::TimePoint now) {
  Heron::Scan scan;
  scan.seq = 100;
  scan.tEnd = now - std::chrono::milliseconds(30);
  scan.tStart = scan.tEnd - std::chrono::milliseconds(137);
  scan.rawSampleCount = 530;
  scan.worstSamplingGapRad = 8.0f * kDeg;
  for (std::int32_t d = -180; d < 180; ++d) {
    scan.points.push_back({static_cast<float>(d) * kDeg, 3.0f});
  }
  return scan;
}

}  // namespace

int main() {
  using namespace Heron::Lidar;
  std::printf("Sector check\n");
  const Heron::TimePoint now = Heron::Clock::now();
  const Sector forward;  // 60 degrees ahead

  {
    check("no scan yet is unknown (NoScan)",
          !checkSector(nullptr, forward, now).known &&
              checkSector(nullptr, forward, now).reason == SectorReason::NoScan);

    Heron::Scan scan = freshScan(now);
    scan.points.push_back({10.0f * kDeg, 0.8f});   // inside the arc
    scan.points.push_back({50.0f * kDeg, 0.3f});   // closer, but outside it
    const SectorResult r = checkSector(&scan, forward, now);
    check("a healthy scan gives a known answer", r.known && r.reason == SectorReason::None);
    check("nearest is the closest point INSIDE the sector, not the closest overall",
          std::fabs(r.nearestM - 0.8f) < 1e-6f &&
              std::fabs(r.nearestBearingRad - 10.0f * kDeg) < 1e-6f);
    check("the result carries seq and age", r.seq == 100 &&
                                                r.age == std::chrono::milliseconds(30));
  }

  {
    Heron::Scan scan = freshScan(now);
    scan.tEnd = now - std::chrono::milliseconds(400);
    check("rule 1: an old scan is unknown (Stale)",
          checkSector(&scan, forward, now).reason == SectorReason::Stale);
    check("rule 1: the caller can allow an older scan",
          checkSector(&scan, forward, now, std::chrono::milliseconds(500)).known);
  }

  {
    Heron::Scan scan = freshScan(now);
    scan.worstSamplingGapRad = 40.0f * kDeg;  // a hole somewhere
    check("rule 2: a revolution with a hole anywhere is unknown (NotCovered)",
          checkSector(&scan, forward, now).reason == SectorReason::NotCovered);
  }

  {
    // Rule 3 on its own: coverage passes at a laxer threshold is not
    // possible through checkSector, so test isSectorObserved directly.
    Heron::Scan scan = freshScan(now);
    scan.unobserved.push_back({-10.0f * kDeg, 15.0f * kDeg});  // -10 to +5, inside the arc
    check("rule 3: an unobserved arc inside the sector",
          !isSectorObserved(scan, forward));

    scan.unobserved = {{27.0f * kDeg, 20.0f * kDeg}};  // 27 to 47: clips the +30 edge by 3
    check("rule 3: an arc that only clips the sector's edge is still caught",
          !isSectorObserved(scan, forward));

    scan.unobserved = {{100.0f * kDeg, 30.0f * kDeg}};  // well to the left
    check("rule 3: an arc outside the sector does not matter",
          isSectorObserved(scan, forward));

    scan.unobserved = {{170.0f * kDeg, 30.0f * kDeg}};  // 170 round to -160
    const Sector behind{kPi, 20.0f * kDeg};              // 160 round to -160
    check("rule 3: an arc across the seam overlaps a sector behind the robot",
          !isSectorObserved(scan, behind));
    check("rule 3: ...and not the forward sector",
          isSectorObserved(scan, forward));
  }

  {
    Heron::Scan scan = freshScan(now);
    scan.points.clear();  // looked everywhere, nothing in range
    const SectorResult r = checkSector(&scan, forward, now);
    check("an empty but observed sector is KNOWN, with infinite distance",
          r.known && std::isinf(r.nearestM));
  }

  {
    Heron::Scan scan = freshScan(now);
    scan.points.push_back({179.5f * kDeg, 1.2f});
    scan.points.push_back({-179.5f * kDeg, 0.9f});
    const SectorResult r = checkSector(&scan, Sector{kPi, 5.0f * kDeg}, now);
    check("a sector behind the robot sees points either side of the seam",
          r.known && std::fabs(r.nearestM - 0.9f) < 1e-6f);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
