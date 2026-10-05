// File        : ScanBuilder.h
// Author      : Chris Pane
// Description : Turns one revolution of raw sensor samples into a Scan: gap
//               detection, sensor-to-robot transform, filtering and sorting.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Pure functions, no SDK and no threads, so every decision here is testable on
// the host against recordings. The adapter copies each SDK node into a
// RawSample and calls buildScan() once per revolution.
//
// The adapter does NOT call the SDK's ascendScanData(). Its only job is
// sorting, and it overwrites the angle of every no-return sample with an
// evenly spaced guess, which is the data the gap detector needs. Measuring
// gaps on the raw samples and sorting our own robot-frame copy removes that
// ordering trap instead of working around it (docs/lidar-cpp-design.md, §6).
//
// Overrun detection uses two independent signals, because measured overruns
// reached the SDK's output either as a wide gap or as two revolutions merged
// into one scan with no gap at all (docs/rplidar-a1m8-findings.md, trap 10):
//
//   gap     measureGaps(): the widest angular gap between ANY two samples,
//           no-returns included
//   count   buildScan(): the sample count against what one revolution should
//           hold at this rotation rate

#ifndef HERON_LIDAR_SCAN_BUILDER_H
#define HERON_LIDAR_SCAN_BUILDER_H

#include <cstdint>
#include <span>
#include <vector>

#include "heron/lidar/Scan.h"

namespace Heron::Lidar {

/**
 * One sample as the sensor delivered it, in the SENSOR frame.
 *
 * Mirrors the fields of the SDK's sl_lidar_response_measurement_node_hq_t that
 * matter, so this file needs no SDK header.
 */
struct RawSample {
  std::uint16_t angleQ14{0};  // 0..65535 spans 0..360 degrees, clockwise
  std::uint32_t distQ2{0};    // quarter millimetres; 0 means no return
  bool sync{false};           // first sample of a revolution
};

/** 360 degrees in the SDK's Q14 angle units: 90 degrees is 16384. */
inline constexpr std::uint32_t kFullCircleQ14 = 65536;

/**
 * @param angleQ14 An angle in the SDK's Q14 units
 * @return The same angle in radians, in [0, 2pi)
 */
[[nodiscard]] float q14ToRad(std::uint32_t angleQ14) noexcept;

/**
 * Converts a sensor-frame angle to a robot-frame bearing.
 *
 * The sensor measures clockwise from its front; the robot (REP 103) measures
 * anticlockwise from its front. So the conversion negates, then adds where
 * the sensor's front points in the robot frame, then wraps. A wrong sign here
 * mirrors every obstacle with no error, which is why this has its own test.
 *
 * @param sensorRad Sensor-frame angle, radians, clockwise from the sensor's front
 * @param mountingOffsetRad Robot-frame bearing of the sensor's front; 0 until measured
 * @return Robot-frame bearing in [-pi, +pi)
 */
[[nodiscard]] float sensorToRobot(float sensorRad, float mountingOffsetRad) noexcept;

/**
 * What the gap detector found in one revolution.
 */
struct GapReport {
  /** Widest gap between any two consecutive samples, closing the seam. */
  float widestRad{0.0f};

  /** Every gap wider than the threshold, in the SENSOR frame: each starts at
   *  a sample and extends clockwise (increasing sensor angle). */
  std::vector<Arc> sensorArcs;

  /** Samples estimated lost inside those gaps, from the median spacing. */
  std::uint32_t droppedEstimate{0};
};

/**
 * Measures the angular gaps in one revolution.
 *
 * Every sample counts, no-returns included: a no-return still proves the beam
 * looked there. Works in integer Q14, sorting a copy, so the input order (not
 * angular on this sensor) does not matter. Its reference is
 * Revolution.sampling_gap_q6() in python/heron/lidar/scan.py.
 *
 * @param samples One revolution, any order
 * @param minArcRad Gaps wider than this are reported as unobserved arcs
 * @return The widest gap, the arcs, and an estimate of samples lost.
 *         Fewer than two samples report a full-circle gap and no arcs.
 */
[[nodiscard]] GapReport measureGaps(std::span<const RawSample> samples,
                                    float minArcRad = kUnobservedGapRad);

/**
 * Settings that stay fixed for a session.
 */
struct BuildConfig {
  float mountingOffsetRad{0.0f};
  float usPerSample{0.0f};  // from the SDK's LidarScanMode; 0 disables the count check
  float mergedFactor{1.5f}; // more samples than this times the expected count = merged
  float shortFactor{0.9f};  // fewer than this times expected counts toward droppedEstimate
};

/**
 * One revolution's input.
 */
struct BuildInput {
  std::span<const RawSample> samples;  // as the SDK delivered them
  TimePoint tStart{};                  // the SDK's scan timestamp
  Duration period{};                   // this revolution's length, estimated by the caller
  std::uint64_t seq{0};
};

/**
 * A built scan, plus what the adapter needs to decide whether to publish it.
 */
struct BuiltScan {
  Scan scan;
  /** More samples than one revolution can hold: two revolutions merged after
   *  an overrun. Its points are real but its geometry and times are not one
   *  turn's, so it must not be treated as an ordinary scan. */
  bool merged{false};
  /** Samples one revolution should hold at this period; 0 if unknown. */
  std::uint32_t expectedSamples{0};
};

/**
 * Builds a Scan from one revolution.
 *
 * Order: gap detection on the raw samples, then filter no-returns, transform
 * to the robot frame, and sort by bearing.
 *
 * @param input One revolution and its timing
 * @param config Session settings
 * @return The scan and its merged flag
 */
[[nodiscard]] BuiltScan buildScan(const BuildInput &input, const BuildConfig &config);

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_SCAN_BUILDER_H
