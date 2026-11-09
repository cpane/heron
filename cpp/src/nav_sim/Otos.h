// File        : Otos.h
// Author      : Chris Pane
// Description : The simulator's OTOS: the base's true motion, with a heading
//               that drifts and a little noise on the position.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// An odometry sensor that is nearly right, as ours is (design §12.2): its
// heading drifts slowly, 0.2 to 0.9 deg/min still, as measured
// (docs/navigation-design.md §12.2; the runs are in
// docs/odometry-otos-reference.md §8), and the position it reports carries
// a millimetre of noise.
//
// The drift is integrated, not added at the end, because that is how a
// real one goes wrong: each step of the base's true motion is applied in
// the sensor's own, drifted heading, so the position error grows with the
// distance driven. Driving 2 m straight with the heading 1 degree out ends
// about 35 mm to one side. The noise is added to each reading and not
// kept, so it does not accumulate.
//
// Its frame is the world's: the first read() starts it at the base's true
// pose, as if the odom frame and the world were one, so tests compare a
// believed pose with the truth directly.
//
// A reading is a Pose::PoseSample, what PoseTracker would store, at the
// time of the read; the velocity is the base's true one, robot frame.
//
// Usage
// -----
//   Heron::NavSim::Otos otos(OtosConfig{}, now);
//   Heron::Pose::PoseHistory history;
//   while (...) {
//     now += 10ms;
//     base.advance(now);
//     history.add(otos.read(now, base));   // 100 a second, as the real one
//   }

#ifndef HERON_NAV_SIM_OTOS_H
#define HERON_NAV_SIM_OTOS_H

#include <cstdint>
#include <random>

#include "heron/geometry/Pose2.h"
#include "heron/pose/PoseHistory.h"
#include "nav_sim/Base.h"

namespace Heron::NavSim {

/** The OTOS's errors. Drift in the measured range [ours], noise a guess. */
struct OtosConfig {
  double driftDegPerMin{0.5};  // heading drift, positive anticlockwise
  double noiseM{0.001};        // standard deviation, on x and on y
  std::uint32_t seed{1};
};

/**
 * Thread Safety: Not thread-safe; one test owns it.
 */
class Otos {
  public:
    Otos(OtosConfig config, TimePoint start);

    /**
     * One reading, as PoseTracker would store it. Call it with the base
     * already advanced to now; a now earlier than the last read gives the
     * last reading's pose again, at now.
     */
    [[nodiscard]] Pose::PoseSample read(TimePoint now, const Base &base);

  private:
    OtosConfig m_config;
    std::mt19937 m_random;
    TimePoint m_lastTime;
    Geometry::Pose2 m_lastTrue{};
    Geometry::Pose2 m_believed{};  // the drifted pose, without the noise
    bool m_started{false};
};

}  // namespace Heron::NavSim

#endif  // HERON_NAV_SIM_OTOS_H
