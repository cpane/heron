// File        : OdometryAdapter.cpp
// Author      : Chris Pane
// Description : Probe O6. Runs Odometry live and reports what a 50 Hz robot
//               loop would see, once a second.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> odometry_adapter
//     scripts/deploy_cpp.sh <user>@<pi> odometry_adapter 30
//     scripts/deploy_cpp.sh <user>@<pi> odometry_adapter 30 0.995 1.002
//
// The odometry counterpart of lidar_adapter and remote_adapter. Uses only
// Odometry.h and links only heron_odometry, as robot code will, so it also
// proves that library stands on its own. Polls getLatest() at 50 Hz, like a
// control loop, and counts how often a reading was not OK at a poll.
//
// start() calibrates the IMU: keep the sensor still and level until the
// first line prints. After that, move it: the pose and totalHeading follow,
// and totalHeading keeps counting past half a turn. Ctrl-C or SIGTERM ends
// the run early.

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <thread>

#include "heron/odometry/Odometry.h"

namespace {

using namespace Heron::Odometry;

constexpr double kDegPerRad = 180.0 / std::numbers::pi;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

double ms(Duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

}  // namespace

int main(int argc, char **argv) {
  const std::int32_t seconds = (argc > 1) ? std::atoi(argv[1]) : 20;
  OtosConfig config;
  if (argc > 3) {
    config.linearScalar = std::atof(argv[2]);
    config.angularScalar = std::atof(argv[3]);
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" Probe O6 -- Odometry, live\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s\n", config.device.c_str());
  std::printf("  scalars         linear %.3f, angular %.3f\n", config.linearScalar,
              config.angularScalar);
  std::printf("  Starting: keep the sensor still...\n");
  std::fflush(stdout);

  Odometry odometry(config);
  const auto startBegin = Clock::now();
  if (!odometry.start()) {
    std::fprintf(stderr, "ERROR: %s\n", odometry.getHealth().lastError.c_str());
    return 1;
  }
  const OdometryHealth up = odometry.getHealth();
  std::printf("  started in %.0f ms (calibration %.0f ms); HW %u.%u, FW %u.%u\n",
              ms(Clock::now() - startBegin), ms(up.calibrationTime), up.hardwareMajor,
              up.hardwareMinor, up.firmwareMajor, up.firmwareMinor);
  std::printf("  reporting each second for %d s; move it now\n\n", seconds);
  std::printf("    t  state    ok  +reads  age ms        x m       y m   heading   total deg"
              "  status\n");

  std::uint64_t polls = 0;
  std::uint64_t notOk = 0;
  std::uint64_t lastSeq = odometry.getLatest().seq;
  const auto begin = Clock::now();
  auto nextPoll = begin;
  auto nextReport = begin + std::chrono::seconds(1);
  std::int32_t reported = 0;
  while (s_stopRequested == 0 && reported < seconds) {
    nextPoll += std::chrono::milliseconds(20);  // 50 Hz
    std::this_thread::sleep_until(nextPoll);
    const OdometrySnapshot o = odometry.getLatest();
    ++polls;
    if (!o.isOk()) {
      ++notOk;
    }
    if (Clock::now() >= nextReport) {
      nextReport += std::chrono::seconds(1);
      ++reported;
      std::printf("  %3d  %-7s  %-3s  %6" PRIu64 "  %6.1f  %+8.4f  %+8.4f  %+8.2f  %+10.2f    0x%02X\n",
                  reported, toString(odometry.getHealth().state), o.isOk() ? "yes" : "NO",
                  o.seq - lastSeq, ms(o.age), o.pose.x, o.pose.y, o.pose.h * kDegPerRad,
                  o.totalHeading * kDegPerRad, o.status);
      std::fflush(stdout);
      lastSeq = o.seq;
    }
  }

  const auto stopBegin = Clock::now();
  odometry.stop();
  const OdometryHealth h = odometry.getHealth();
  std::printf("\n  %s; stop() took %.0f ms\n", s_stopRequested != 0 ? "stopped by signal" : "time up",
              ms(Clock::now() - stopBegin));
  std::printf("  polls %" PRIu64 ", not OK at %" PRIu64 "\n", polls, notOk);
  std::printf("  health: state %s, readings %" PRIu64 ", failed %" PRIu64 ", tilt %" PRIu64
              ", tracking %" PRIu64 ", status seen 0x%02X%s%s\n",
              toString(h.state), h.readings, h.failedReads, h.tiltSnapshots, h.trackingWarnings,
              h.statusSeen, h.lastError.empty() ? "" : ", error: ", h.lastError.c_str());
  return h.state == OdometryState::Failed ? 1 : 0;
}
