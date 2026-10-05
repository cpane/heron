// File        : LidarReplayCheck.cpp
// Author      : Chris Pane
// Description : Runs the Lidar adapter against a recording and checks what a
//               consumer receives.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Not a ctest: recordings are git-ignored. Run per recording:
//
//     build-host/bin/lidar_replay_check captures/<stem>.rpraw
//
// Polls getLatest() at 50 Hz, as a robot loop would, and checks every new
// Scan as it arrives: invariants, and its age when first seen (which proves
// the SDK-to-steady_clock conversion on this platform). Writes the published
// scans to <stem>.adapter.dump for python/tools/lidar_adaptercheck.py, which
// compares every point against the Python decoder.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "heron/lidar/Lidar.h"

namespace {

using Heron::Clock;

constexpr auto kPoll = std::chrono::milliseconds(20);
constexpr auto kQuietEnd = std::chrono::seconds(2);  // no new scan this long: recording over
constexpr auto kCap = std::chrono::seconds(90);

bool invariantsHold(const Heron::Scan &scan) {
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const Heron::Point &p = scan.points[i];
    if (!(p.bearingRad >= -Heron::kPi && p.bearingRad < Heron::kPi) || !(p.rangeM > 0.0f)) {
      return false;
    }
    if (i > 0 && scan.points[i - 1].bearingRad > p.bearingRad) {
      return false;
    }
  }
  return scan.tStart <= scan.tEnd && scan.sweepStartRad >= -Heron::kPi &&
         scan.sweepStartRad < Heron::kPi;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: lidar_replay_check FILE.rpraw\n");
    return 2;
  }
  const std::string raw = argv[1];
  const std::string stem = raw.size() > 6 ? raw.substr(0, raw.size() - 6) : raw;

  Heron::Lidar::LidarConfig config;
  config.replayFile = raw;
  config.scanMode = 1;  // overridden below from the file name for other modes
  const std::size_t at = stem.rfind("-sdk");
  if (at != std::string::npos && at + 4 < stem.size()) {
    config.scanMode = stem[at + 4] - '0';
  }

  Heron::Lidar::Lidar lidar(config);
  if (!lidar.start()) {
    std::fprintf(stderr, "ERROR: %s\n", lidar.getHealth().lastError.c_str());
    return 1;
  }

  std::vector<std::shared_ptr<const Heron::Scan>> seen;
  std::vector<double> ageAtSight;
  double worstAgeMs = 0.0;
  double bestAgeMs = 1e9;
  std::int32_t badInvariants = 0;
  std::uint64_t seqGaps = 0;
  const Clock::time_point begin = Clock::now();
  Clock::time_point lastNew = begin;
  while (Clock::now() - begin < kCap) {
    std::this_thread::sleep_for(kPoll);
    std::shared_ptr<const Heron::Scan> scan = lidar.getLatest();
    if (scan && (seen.empty() || scan->seq != seen.back()->seq)) {
      const double ageMs =
          std::chrono::duration<double, std::milli>(Clock::now() - scan->tEnd).count();
      worstAgeMs = std::max(worstAgeMs, ageMs);
      bestAgeMs = std::min(bestAgeMs, ageMs);
      badInvariants += invariantsHold(*scan) ? 0 : 1;
      if (!seen.empty() && scan->seq > seen.back()->seq + 1) {
        seqGaps += scan->seq - seen.back()->seq - 1;
      }
      seen.push_back(scan);
      ageAtSight.push_back(ageMs);
      lastNew = Clock::now();
    }
    if (!seen.empty() && Clock::now() - lastNew > kQuietEnd) {
      break;
    }
  }
  lidar.stop();
  const Heron::Lidar::LidarHealth health = lidar.getHealth();

  std::printf("  scans seen        %zu (seq skipped: %" PRIu64 ")\n", seen.size(), seqGaps);
  std::printf("  health            published %" PRIu64 ", dropped merged %" PRIu64
              ", dropped backlog %" PRIu64 ", with loss %" PRIu64 ", grab timeouts %" PRIu64
              "\n",
              health.scansPublished, health.scansDroppedMerged, health.scansDroppedBacklog,
              health.scansWithLoss, health.grabTimeouts);
  std::printf("  invariants        %s\n", badInvariants == 0 ? "hold on every scan" : "VIOLATED");
  const std::size_t staleOnArrival = static_cast<std::size_t>(
      std::count_if(ageAtSight.begin(), ageAtSight.end(), [](double a) { return a > 300.0; }));
  std::printf("  age when seen     %.0f to %.0f ms (tEnd to first sight); stale on arrival "
              "(>300 ms): %zu\n",
              bestAgeMs, worstAgeMs, staleOnArrival);
  std::printf("  state after stop  %s\n",
              health.state == Heron::Lidar::LidarState::Stopped ? "Stopped" : "NOT Stopped");

  std::FILE *dump = std::fopen((stem + ".adapter.dump").c_str(), "w");
  if (dump == nullptr) {
    std::fprintf(stderr, "ERROR: cannot write %s.adapter.dump\n", stem.c_str());
    return 1;
  }
  for (std::size_t k = 0; k < seen.size(); ++k) {
    const std::shared_ptr<const Heron::Scan> &scan = seen[k];
    std::fprintf(dump, "scan %" PRIu64 " %" PRIu32 " %zu %.7f %.7f %" PRIu32 " %.1f\n", scan->seq,
                 scan->rawSampleCount, scan->size(), static_cast<double>(scan->sweepStartRad),
                 static_cast<double>(scan->worstSamplingGapRad), scan->droppedEstimate,
                 ageAtSight[k]);
    for (const Heron::Point &p : scan->points) {
      std::fprintf(dump, "%.7f %.7f\n", static_cast<double>(p.bearingRad),
                   static_cast<double>(p.rangeM));
    }
  }
  std::fclose(dump);

  // A scan whose end lies in the future would fool a consumer's freshness
  // check: the symptom of publishing a post-stall backlog. 10 ms allows for
  // clock-reading jitter. A scan that is OLD on arrival is legitimate: the
  // last revolution before a stall can only be published after it, with true
  // timestamps, and a consumer's freshness check then rejects it.
  const bool ok = badInvariants == 0 && bestAgeMs > -10.0 && !seen.empty() &&
                  health.state == Heron::Lidar::LidarState::Stopped;
  std::printf("\n  %s\n", ok ? "OK" : "FAILED");
  return ok ? 0 : 1;
}
