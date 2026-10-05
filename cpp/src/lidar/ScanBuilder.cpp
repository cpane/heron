// File        : ScanBuilder.cpp
// Author      : Chris Pane
// Description : Gap detection, transform and assembly of one revolution.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include <algorithm>
#include <chrono>
#include <cmath>

#include "ScanBuilder.h"

namespace Heron::Lidar {

namespace {

// Median of the inner (non-seam) spacings, in Q14. Used to turn a gap into an
// estimate of how many samples it swallowed.
std::uint32_t medianSpacingQ14(const std::vector<std::uint32_t> &sortedQ14) {
  std::vector<std::uint32_t> spacing;
  spacing.reserve(sortedQ14.size());
  for (std::size_t i = 1; i < sortedQ14.size(); ++i) {
    spacing.push_back(sortedQ14[i] - sortedQ14[i - 1]);
  }
  if (spacing.empty()) {
    return 0;
  }
  const std::size_t mid = spacing.size() / 2;
  std::nth_element(spacing.begin(), spacing.begin() + static_cast<std::ptrdiff_t>(mid),
                   spacing.end());
  return spacing[mid];
}

}  // namespace

float q14ToRad(std::uint32_t angleQ14) noexcept {
  constexpr float kRadPerQ14 = kTwoPi / static_cast<float>(kFullCircleQ14);
  return static_cast<float>(angleQ14 % kFullCircleQ14) * kRadPerQ14;
}

float sensorToRobot(float sensorRad, float mountingOffsetRad) noexcept {
  return wrapBearing(-sensorRad + mountingOffsetRad);
}

GapReport measureGaps(std::span<const RawSample> samples, float minArcRad) {
  GapReport report;
  if (samples.size() < 2) {
    report.widestRad = kTwoPi;
    return report;
  }

  std::vector<std::uint32_t> angles;
  angles.reserve(samples.size());
  for (const RawSample &sample : samples) {
    angles.push_back(sample.angleQ14 % kFullCircleQ14);
  }
  std::sort(angles.begin(), angles.end());

  const std::uint32_t typical = medianSpacingQ14(angles);
  std::uint32_t widest = 0;
  // Each gap runs from angles[i] to the next sample clockwise; the last one
  // closes the circle across the seam.
  for (std::size_t i = 0; i < angles.size(); ++i) {
    const bool seam = (i + 1 == angles.size());
    const std::uint32_t next = seam ? angles.front() + kFullCircleQ14 : angles[i + 1];
    const std::uint32_t gap = next - angles[i];
    widest = std::max(widest, gap);
    const float gapRad = q14ToRad(gap % kFullCircleQ14) + (gap == kFullCircleQ14 ? kTwoPi : 0.0f);
    if (gapRad > minArcRad) {
      report.sensorArcs.push_back({q14ToRad(angles[i]), gapRad});
      if (typical > 0) {
        const std::uint32_t steps = (gap + typical / 2) / typical;  // rounded
        report.droppedEstimate += steps > 1 ? steps - 1 : 0;
      }
    }
  }
  report.widestRad = (widest == kFullCircleQ14) ? kTwoPi : q14ToRad(widest);
  return report;
}

BuiltScan buildScan(const BuildInput &input, const BuildConfig &config) {
  BuiltScan built;
  Scan &scan = built.scan;
  scan.seq = input.seq;
  scan.tStart = input.tStart;
  scan.tEnd = input.tStart + input.period;
  scan.rawSampleCount = static_cast<std::uint32_t>(input.samples.size());

  // 1. Gaps, on every raw sample, before anything is filtered or reordered.
  const GapReport gaps = measureGaps(input.samples);
  scan.worstSamplingGapRad = gaps.widestRad;
  scan.droppedEstimate = gaps.droppedEstimate;
  // A sensor arc runs clockwise from s to s + w. Negation reverses direction,
  // so in the robot frame the same region runs anticlockwise from the image
  // of s + w back to the image of s.
  for (const Arc &arc : gaps.sensorArcs) {
    scan.unobserved.push_back(
        {sensorToRobot(arc.startRad + arc.widthRad, config.mountingOffsetRad), arc.widthRad});
  }

  // 2. Where the sweep began: the sync sample, normally the first.
  const auto syncIt = std::find_if(input.samples.begin(), input.samples.end(),
                                   [](const RawSample &s) { return s.sync; });
  if (!input.samples.empty()) {
    const RawSample &first = (syncIt != input.samples.end()) ? *syncIt : input.samples.front();
    scan.sweepStartRad = sensorToRobot(q14ToRad(first.angleQ14), config.mountingOffsetRad);
  }

  // 3. Valid returns only, in the robot frame, sorted by bearing.
  scan.points.reserve(input.samples.size());
  for (const RawSample &sample : input.samples) {
    if (sample.distQ2 == 0) {
      continue;
    }
    scan.points.push_back({sensorToRobot(q14ToRad(sample.angleQ14), config.mountingOffsetRad),
                           static_cast<float>(sample.distQ2) / 4000.0f});
  }
  std::sort(scan.points.begin(), scan.points.end(),
            [](const Point &a, const Point &b) { return a.bearingRad < b.bearingRad; });

  // 4. The count check: the second, independent overrun signal.
  const double periodUs =
      std::chrono::duration<double, std::micro>(input.period).count();
  if (config.usPerSample > 0.0f && periodUs > 0.0) {
    built.expectedSamples =
        static_cast<std::uint32_t>(std::lround(periodUs / static_cast<double>(config.usPerSample)));
    const double expected = static_cast<double>(built.expectedSamples);
    const double count = static_cast<double>(scan.rawSampleCount);
    built.merged = count > static_cast<double>(config.mergedFactor) * expected;
    if (count < static_cast<double>(config.shortFactor) * expected) {
      const std::uint32_t shortBy = built.expectedSamples - scan.rawSampleCount;
      scan.droppedEstimate = std::max(scan.droppedEstimate, shortBy);
    }
  }
  return built;
}

}  // namespace Heron::Lidar
