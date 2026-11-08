// File        : Lidar.h
// Author      : Chris Pane
// Description : The simulator's RPLIDAR A1M8: turning while Heron moves, and
//               producing Scans through the real adapter's scan builder.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// A LiDAR that behaves like ours, as measured (docs/rplidar-a1m8-findings.md):
//
//   - It turns clockwise at 7.3 rev/s, each revolution's period varying by
//     0.8 ms (one standard deviation), and samples at a steady rate, so a
//     revolution holds about 531 samples, more or fewer as the period varies.
//   - Each revolution starts at 358.6 degrees in the sensor frame, where the
//     real sync flag fires on average.
//   - Every revolution has one gap with no samples, at a fixed sensor angle,
//     as every real one does (cause not understood). The samples are about
//     0.66 degrees apart, so a gap is a whole number of spacings: 7.3
//     degrees leaves out 11 samples, and the widest gap reads about 7.95
//     (12 spacings), against 8.0 to 8.1 measured in express mode. The 531
//     samples are packed into the rest of the circle.
//   - Most samples return nothing: a share of those that would hit, and
//     every one with nothing in range or nearer than 126.8 mm.
//   - Each sample is taken at its own time, from where the base was then, so
//     a scan taken on the move is skewed exactly as a real one is.
//
// Each sample is made as the sensor would send it (RawSample: a Q14 angle,
// a Q2 distance, the sync flag) and each revolution goes through
// buildScan(), the adapter's own code. So a simulated Scan keeps every
// invariant a real one does (sorted, wrapped, ranges above zero, gaps and
// unobserved arcs measured), by construction rather than by imitation.
//
// Its no-returns are scattered at random; real ones cluster by surface. A
// test that cares about clustering needs a recording, not this.
//
// The random numbers come from std::mt19937 and the standard distributions,
// whose output differs between standard libraries: a test may rely on a
// seed giving the same run twice on one machine, never on exact values.
//
// Usage
// -----
// Call advance() with the base, more often than once a revolution (every
// 5 to 20 ms is right): between two calls the base's pose is interpolated.
//
//   Heron::NavSim::Lidar lidar(world, LidarConfig{}, now);
//   while (...) {
//     now += 10ms;
//     base.advance(now);
//     if (const auto scan = lidar.advance(now, base)) {
//       ...                                 // a revolution has completed
//     }
//   }

#ifndef HERON_NAV_SIM_LIDAR_H
#define HERON_NAV_SIM_LIDAR_H

#include <cstdint>
#include <optional>
#include <random>
#include <vector>

#include "heron/geometry/Pose2.h"
#include "heron/lidar/Scan.h"
#include "lidar/ScanBuilder.h"
#include "nav_sim/Base.h"
#include "nav_sim/World.h"

namespace Heron::NavSim {

/** The A1M8 as measured [ours]. */
struct LidarConfig {
  Geometry::Vec2 offset{0.06, 0.0};  // spin axis in the robot frame, m
  double yawRad{0.0};                // the housing's front, robot frame (mountingOffsetRad)
  double revolutionsHz{7.3};
  // The revolution time's true spread. The findings' 0.8 ms comes from SDK
  // timestamps, which jitter; the sample count, at a steady 254 us a
  // sample, varies by 0.75 samples, so the period by about 0.2 ms.
  double periodStdevS{0.0002};
  std::int32_t samplesPerRevolution{531};
  double syncDeg{358.6};             // sensor frame, where each revolution starts
  double gapStartDeg{40.0};          // sensor frame, the per-revolution gap
  double gapWidthDeg{7.3};
  double minRangeM{0.1268};
  double maxRangeM{12.0};
  double noReturnFraction{0.7};      // of samples that would hit
  double rangeNoiseM{0.005};         // standard deviation
  std::uint32_t seed{1};
};

/**
 * Thread Safety: Not thread-safe; one test owns it. Holds a reference to
 * the world, which must outlive it.
 */
class Lidar {
  public:
    Lidar(const World &world, LidarConfig config, TimePoint start);

    /**
     * Samples the world along the robot's path up to now.
     *
     * @return A Scan each time a revolution completes; if more than one
     *         completed since the last call, the newest, and the others are
     *         missing from seq, as when a real reader falls behind
     */
    [[nodiscard]] std::optional<Heron::Scan> advance(TimePoint now, const Base &base);

  private:
    [[nodiscard]] Geometry::Pose2 poseAt(TimePoint t, TimePoint now,
                                         const Geometry::Pose2 &nowPose) const;
    [[nodiscard]] Heron::Lidar::RawSample sample(double sensorDeg, const Geometry::Pose2 &pose);
    [[nodiscard]] Duration nextPeriod();
    [[nodiscard]] Heron::Scan finishRevolution();

    const World &m_world;
    LidarConfig m_config;
    Heron::Lidar::BuildConfig m_build;
    Duration m_sampleInterval{};
    std::mt19937 m_random;

    TimePoint m_revolutionStart;
    Duration m_period{};
    TimePoint m_nextSample;
    std::vector<Heron::Lidar::RawSample> m_samples;
    std::uint64_t m_seq{0};

    TimePoint m_lastTime{};
    Geometry::Pose2 m_lastPose;
    bool m_havePose{false};
};

}  // namespace Heron::NavSim

#endif  // HERON_NAV_SIM_LIDAR_H
