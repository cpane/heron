// File        : LidarAdapter.cpp
// Author      : Chris Pane
// Description : Probe C3. Runs the Lidar adapter live and reports what a
//               robot loop would see, once a second.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh cpane@<pi> lidar_adapter 20
//
// Uses only the public interface (Lidar.h, Scan.h), as robot code will, so it
// also proves the boundary: this file sees no SDK type. Polls getLatest() at
// 50 Hz. Ctrl-C or SIGTERM ends the run early; the adapter then stops the
// scan and the motor, which is part of what this probe checks.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>

#include "heron/lidar/Lidar.h"

namespace {

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

const char *stateName(Heron::Lidar::LidarState state) {
  switch (state) {
    case Heron::Lidar::LidarState::Stopped:
      return "Stopped";
    case Heron::Lidar::LidarState::Running:
      return "Running";
    case Heron::Lidar::LidarState::Failed:
      return "Failed";
  }
  return "?";
}

}  // namespace

int main(int argc, char **argv) {
  const std::int32_t seconds = (argc > 1) ? static_cast<std::int32_t>(std::atoi(argv[1])) : 20;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" Probe C3 -- Lidar adapter, live\n");
  std::printf("========================================\n\n");

  Heron::Lidar::LidarConfig config;  // /dev/rplidar, Express, offset 0
  Heron::Lidar::Lidar lidar(config);
  const Heron::Clock::time_point begin = Heron::Clock::now();
  if (!lidar.start()) {
    std::fprintf(stderr, "ERROR: %s\n", lidar.getHealth().lastError.c_str());
    return 1;
  }
  const double startMs =
      std::chrono::duration<double, std::milli>(Heron::Clock::now() - begin).count();
  std::printf("  started in %.0f ms; reporting each second for %" PRId32 " s\n\n", startMs,
              seconds);
  std::printf("    t   scans  rate Hz  points  cov  worst gap  age ms  seq skips\n");

  std::uint64_t lastSeq = 0;
  std::uint64_t totalScans = 0;
  std::uint64_t totalSkips = 0;
  double worstAgeEver = 0.0;
  for (std::int32_t t = 1; t <= seconds && s_stopRequested == 0; ++t) {
    std::uint32_t scans = 0;
    std::uint64_t skips = 0;
    double worstAge = 0.0;
    std::shared_ptr<const Heron::Scan> newest;
    for (std::int32_t tick = 0; tick < 50 && s_stopRequested == 0; ++tick) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      std::shared_ptr<const Heron::Scan> scan = lidar.getLatest();
      if (scan && scan->seq != lastSeq) {
        if (lastSeq != 0 && scan->seq > lastSeq + 1) {
          skips += scan->seq - lastSeq - 1;
        }
        lastSeq = scan->seq;
        ++scans;
        newest = scan;
        worstAge = std::max(worstAge, std::chrono::duration<double, std::milli>(
                                          Heron::Clock::now() - scan->tEnd)
                                          .count());
      }
    }
    totalScans += scans;
    totalSkips += skips;
    worstAgeEver = std::max(worstAgeEver, worstAge);
    if (newest) {
      std::printf("  %3" PRId32 "  %6" PRIu32 "  %7.2f  %6zu  %3s  %7.1f deg  %6.0f  %9" PRIu64
                  "\n",
                  t, scans, static_cast<double>(newest->rateHz()), newest->size(),
                  newest->isCoverageOk() ? "ok" : "NO",
                  static_cast<double>(newest->worstSamplingGapRad) * 180.0 / 3.14159265358979,
                  worstAge, skips);
    } else {
      std::printf("  %3" PRId32 "  no scan yet\n", t);
    }
  }

  const bool interrupted = s_stopRequested != 0;
  const Heron::Clock::time_point stopBegin = Heron::Clock::now();
  lidar.stop();
  const double stopMs =
      std::chrono::duration<double, std::milli>(Heron::Clock::now() - stopBegin).count();
  const Heron::Lidar::LidarHealth health = lidar.getHealth();

  std::printf("\n  %s; stop() took %.0f ms\n", interrupted ? "interrupted by signal" : "time up",
              stopMs);
  std::printf("  scans seen %" PRIu64 ", seq skips %" PRIu64 ", worst age %.0f ms\n", totalScans,
              totalSkips, worstAgeEver);
  std::printf("  health: published %" PRIu64 ", dropped merged %" PRIu64
              ", dropped backlog %" PRIu64 ", with loss %" PRIu64 ", grab timeouts %" PRIu64
              ", state %s\n",
              health.scansPublished, health.scansDroppedMerged, health.scansDroppedBacklog,
              health.scansWithLoss, health.grabTimeouts, stateName(health.state));
  if (!health.lastError.empty()) {
    std::printf("  last error: %s\n", health.lastError.c_str());
  }
  return health.state == Heron::Lidar::LidarState::Stopped ? 0 : 1;
}
