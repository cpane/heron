// File        : Lidar.cpp
// Author      : Chris Pane
// Description : The simulator's RPLIDAR A1M8.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "nav_sim/Lidar.h"

#include <algorithm>
#include <cmath>
#include <span>

namespace Heron::NavSim {

using Geometry::Pose2;
using Geometry::Vec2;

namespace {

constexpr double kDegPerRad = 57.29577951308232;

double seconds(Duration d) noexcept {
  return std::chrono::duration<double>(d).count();
}

Duration toDuration(double s) noexcept {
  return std::chrono::duration_cast<Duration>(std::chrono::duration<double>(s));
}

/** An angle in degrees folded into [0, 360). */
double fold360(double deg) noexcept {
  const double r = std::fmod(deg, 360.0);
  return r < 0.0 ? r + 360.0 : r;
}

}  // namespace

Lidar::Lidar(const World &world, LidarConfig config, TimePoint start)
    : m_world(world),
      m_config(config),
      m_random(config.seed),
      m_revolutionStart(start),
      m_nextSample(start) {
  // The real sensor delivers ~531 samples a revolution AND leaves its ~8
  // degree gap, so the samples are packed into the rest of the circle: the
  // sampling runs faster by 360 / (360 - gap), and those in the gap are not
  // taken. The scan builder's count check still expects 531 a revolution.
  const double samplesPerSecond =
      config.revolutionsHz * static_cast<double>(config.samplesPerRevolution);
  const double packed = 360.0 / (360.0 - config.gapWidthDeg);
  m_sampleInterval = toDuration(1.0 / (samplesPerSecond * packed));
  m_build.mountingOffsetRad = static_cast<float>(config.yawRad);
  m_build.usPerSample = static_cast<float>(1e6 / samplesPerSecond);
  m_period = nextPeriod();
}

std::optional<Heron::Scan> Lidar::advance(TimePoint now, const Base &base) {
  const Pose2 nowPose = base.truePose();
  std::optional<Heron::Scan> newest;
  while (m_nextSample <= now) {
    const TimePoint t = m_nextSample;
    while (t >= m_revolutionStart + m_period) {
      newest = finishRevolution();
    }
    // The sensor turns clockwise, increasing its own angle, from the sync
    // angle at the revolution's start.
    const double turned = seconds(t - m_revolutionStart) / seconds(m_period);
    const double sensorDeg = fold360(m_config.syncDeg + 360.0 * turned);
    const bool inGap = fold360(sensorDeg - m_config.gapStartDeg) < m_config.gapWidthDeg;
    if (!inGap) {
      Heron::Lidar::RawSample raw = sample(sensorDeg, poseAt(t, now, nowPose));
      raw.sync = m_samples.empty();
      m_samples.push_back(raw);
    }
    m_nextSample += m_sampleInterval;
  }
  m_lastTime = now;
  m_lastPose = nowPose;
  m_havePose = true;
  return newest;
}

Pose2 Lidar::poseAt(TimePoint t, TimePoint now, const Pose2 &nowPose) const {
  // Between the last call and this one the base moved from m_lastPose to
  // nowPose; a sample in between is taken from the straight-line blend,
  // the heading the short way round.
  if (!m_havePose || now <= m_lastTime) {
    return nowPose;
  }
  const double f = std::clamp(seconds(t - m_lastTime) / seconds(now - m_lastTime), 0.0, 1.0);
  const double turn = Geometry::wrapAngle(nowPose.heading - m_lastPose.heading);
  return Pose2{m_lastPose.x + f * (nowPose.x - m_lastPose.x),
               m_lastPose.y + f * (nowPose.y - m_lastPose.y),
               Geometry::wrapAngle(m_lastPose.heading + f * turn)};
}

Heron::Lidar::RawSample Lidar::sample(double sensorDeg, const Pose2 &pose) {
  // Where the beam points: the sensor's clockwise angle to a robot-frame
  // bearing as the adapter converts it (ScanBuilder.h, sensorToRobot()),
  // then into the odom frame with the robot's heading.
  const double bearing = -sensorDeg / kDegPerRad + m_config.yawRad;
  const Vec2 from = Geometry::toParent(pose, m_config.offset);
  const auto hit = m_world.cast(from, pose.heading + bearing, m_config.maxRangeM);

  double rangeM = 0.0;
  if (hit) {
    std::normal_distribution<double> noise(0.0, m_config.rangeNoiseM);
    std::bernoulli_distribution noReturn(m_config.noReturnFraction);
    rangeM = *hit + noise(m_random);
    if (rangeM < m_config.minRangeM || rangeM > m_config.maxRangeM || noReturn(m_random)) {
      rangeM = 0.0;
    }
  }

  Heron::Lidar::RawSample raw;
  const double turns = sensorDeg / 360.0;
  const auto q14 = static_cast<std::uint32_t>(
      std::lround(turns * static_cast<double>(Heron::Lidar::kFullCircleQ14)));
  raw.angleQ14 = static_cast<std::uint16_t>(q14 % Heron::Lidar::kFullCircleQ14);
  raw.distQ2 = static_cast<std::uint32_t>(std::lround(rangeM * 4000.0));  // quarter millimetres
  return raw;
}

Duration Lidar::nextPeriod() {
  std::normal_distribution<double> jitter(0.0, m_config.periodStdevS);
  return toDuration(1.0 / m_config.revolutionsHz + jitter(m_random));
}

Heron::Scan Lidar::finishRevolution() {
  Heron::Lidar::BuildInput input;
  input.samples = std::span<const Heron::Lidar::RawSample>(m_samples);
  input.tStart = m_revolutionStart;
  input.period = m_period;
  input.seq = m_seq++;
  Heron::Scan scan = Heron::Lidar::buildScan(input, m_build).scan;

  m_revolutionStart += m_period;
  m_period = nextPeriod();
  m_samples.clear();
  return scan;
}

}  // namespace Heron::NavSim
