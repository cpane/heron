// File        : ScanBuilderTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the scan builder: transform, gap detection and
//               assembly, with synthetic revolutions and no hardware.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "lidar/ScanBuilder.h"

namespace {

using Heron::kPi;
using Heron::Lidar::RawSample;

std::int32_t s_failures = 0;
constexpr float kDeg = kPi / 180.0f;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(float a, float b, float tol = 1e-4f) {
  return std::fabs(a - b) < tol;
}

std::uint16_t q14(float degrees) {
  return static_cast<std::uint16_t>(std::lround(degrees * 65536.0f / 360.0f) % 65536);
}

// One sample per whole degree in [fromDeg, toDeg], wrapping past 360.
void addSamples(std::vector<RawSample> &out, std::int32_t fromDeg, std::int32_t toDeg,
                std::uint32_t distQ2) {
  for (std::int32_t d = fromDeg; d <= toDeg; ++d) {
    out.push_back({q14(static_cast<float>(((d % 360) + 360) % 360)), distQ2, false});
  }
}

}  // namespace

int main() {
  using namespace Heron::Lidar;
  std::printf("Scan builder\n");

  {
    check("q14: 0 is 0", near(q14ToRad(0), 0.0f));
    check("q14: 16384 is pi/2", near(q14ToRad(16384), kPi / 2.0f));
    check("q14: 32768 is pi", near(q14ToRad(32768), kPi));
    check("q14: 65535 is just under 2pi", q14ToRad(65535) < 2.0f * kPi);
  }

  {
    // The transform table: the sign is the whole point.
    check("transform: sensor 0 (front) is robot 0", near(sensorToRobot(0.0f, 0.0f), 0.0f));
    check("transform: sensor 90 (right) is robot -pi/2",
          near(sensorToRobot(90.0f * kDeg, 0.0f), -kPi / 2.0f));
    check("transform: sensor 270 (left) is robot +pi/2",
          near(sensorToRobot(270.0f * kDeg, 0.0f), kPi / 2.0f));
    check("transform: sensor 315 (ahead-left) is robot +pi/4",
          near(sensorToRobot(315.0f * kDeg, 0.0f), kPi / 4.0f));
    check("transform: sensor 180 (behind) is robot -pi, never +pi",
          sensorToRobot(180.0f * kDeg, 0.0f) == -kPi);
    // Mounted with its front facing the robot's left: the sensor's right
    // then points straight ahead.
    check("transform: offset +pi/2 maps sensor front to robot left",
          near(sensorToRobot(0.0f, kPi / 2.0f), kPi / 2.0f));
    check("transform: offset +pi/2 maps sensor right to robot forward",
          near(sensorToRobot(90.0f * kDeg, kPi / 2.0f), 0.0f));
  }

  {
    std::vector<RawSample> full;
    addSamples(full, 0, 359, 4000);
    const GapReport healthy = measureGaps(full);
    check("gaps: 1 degree spacing reads 1 degree, no arcs, nothing dropped",
          near(healthy.widestRad, 1.0f * kDeg, 1e-3f) && healthy.sensorArcs.empty() &&
              healthy.droppedEstimate == 0);

    // The first-draft bug: valid returns with a hole, but no-returns filling it.
    std::vector<RawSample> filled;
    addSamples(filled, 0, 39, 4000);
    addSamples(filled, 40, 99, 0);
    addSamples(filled, 100, 359, 4000);
    check("gaps: no-returns count as looking (40-99 all no-return, still 1 degree)",
          near(measureGaps(filled).widestRad, 1.0f * kDeg, 1e-3f));

    std::vector<RawSample> holed;
    addSamples(holed, 0, 39, 4000);
    addSamples(holed, 100, 359, 0);
    const GapReport hole = measureGaps(holed);
    check("gaps: a real hole 39-100 reads 61 degrees",
          near(hole.widestRad, 61.0f * kDeg, 1e-3f));
    check("gaps: one arc, starting at 39 degrees, 61 wide",
          hole.sensorArcs.size() == 1 && near(hole.sensorArcs[0].startRad, 39.0f * kDeg, 1e-3f) &&
              near(hole.sensorArcs[0].widthRad, 61.0f * kDeg, 1e-3f));
    check("gaps: about 60 samples estimated lost in it", hole.droppedEstimate == 60);

    std::vector<RawSample> seam;
    addSamples(seam, 21, 339, 4000);
    const GapReport seamHole = measureGaps(seam);
    check("gaps: a hole across the seam (339 to 21) reads 42 degrees",
          near(seamHole.widestRad, 42.0f * kDeg, 1e-3f) && seamHole.sensorArcs.size() == 1 &&
              near(seamHole.sensorArcs[0].startRad, 339.0f * kDeg, 1e-3f));

    std::vector<RawSample> shuffled = holed;
    std::mt19937 rng(7);
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    check("gaps: input order does not matter (the wire is not angular order)",
          near(measureGaps(shuffled).widestRad, hole.widestRad));

    check("gaps: fewer than two samples is a full-circle gap",
          near(measureGaps(std::vector<RawSample>{}).widestRad, 2.0f * kPi));
  }

  {
    // Assembly: one express-like revolution with a hole at sensor 40-100.
    std::vector<RawSample> rev;
    addSamples(rev, 0, 39, 4000);       // 1 m
    addSamples(rev, 100, 359, 0);       // no-returns
    rev[0].sync = false;
    rev.insert(rev.begin(), RawSample{q14(358.6f), 8000, true});  // the sync sample
    BuildInput in;
    in.samples = rev;
    in.tStart = Heron::Clock::now();
    in.period = std::chrono::milliseconds(137);
    in.seq = 9;
    BuildConfig cfg;
    cfg.usPerSample = 254.0f;
    const BuiltScan built = buildScan(in, cfg);
    const Heron::Scan &scan = built.scan;

    check("build: seq and times carried; tEnd is one period on",
          scan.seq == 9 && scan.tEnd - scan.tStart == std::chrono::milliseconds(137));
    check("build: every sample counted, only returns kept",
          scan.rawSampleCount == rev.size() && scan.size() == 41);
    check("build: ranges in metres (4000 q2 = 1 m)",
          std::all_of(scan.points.begin() + 1, scan.points.end(),
                      [](const Heron::Point &p) {
                        return near(p.rangeM, 1.0f) || near(p.rangeM, 2.0f);
                      }));
    check("build: points sorted by robot bearing",
          std::is_sorted(scan.points.begin(), scan.points.end(),
                         [](const Heron::Point &a, const Heron::Point &b) {
                           return a.bearingRad < b.bearingRad;
                         }));
    check("build: sweep starts at the sync sample, robot +1.4 degrees",
          near(scan.sweepStartRad, 1.4f * kDeg, 1e-3f));
    check("build: the sensor hole becomes one robot-frame arc, 61 degrees wide",
          scan.unobserved.size() == 1 && near(scan.unobserved[0].widthRad, 61.0f * kDeg, 1e-3f));
    check("build: inside the hole (sensor 70) is unobserved, outside (sensor 20) observed",
          !scan.isObserved(sensorToRobot(70.0f * kDeg, 0.0f)) &&
              scan.isObserved(sensorToRobot(20.0f * kDeg, 0.0f)));
    check("build: a 61 degree hole fails coverage", !scan.isCoverageOk());
    check("build: expected samples from 137 ms at 254 us", built.expectedSamples == 539);
  }

  {
    // The count check, on the counts measured in the starved-reader runs.
    BuildConfig cfg;
    cfg.usPerSample = 254.0f;
    const auto withCount = [&](std::size_t n) {
      std::vector<RawSample> rev(n);
      for (std::size_t i = 0; i < n; ++i) {
        rev[i] = {static_cast<std::uint16_t>((i * 65536) / n), 4000, i == 0};
      }
      BuildInput in;
      in.samples = rev;
      in.period = std::chrono::milliseconds(137);
      return buildScan(in, cfg);
    };
    check("count: 535 samples is an ordinary revolution", !withCount(535).merged);
    check("count: 929 samples (measured merge) is flagged merged", withCount(929).merged);
    check("count: 827 samples (second measured merge) is flagged merged", withCount(827).merged);
    const BuiltScan shortRev = withCount(471);
    check("count: 471 samples is not merged but counts as dropped",
          !shortRev.merged && shortRev.scan.droppedEstimate == 539 - 471);
    check("count: evenly spread merge has full coverage, so only the count catches it",
          withCount(929).scan.isCoverageOk());
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
