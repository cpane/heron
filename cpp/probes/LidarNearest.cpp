// File        : LidarNearest.cpp
// Author      : Chris Pane
// Description : Probe C4. Reports the nearest obstacle in a sector around the
//               sensor, live, with the consumer rules applied.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> lidar_nearest
//     scripts/deploy_cpp.sh <user>@<pi> lidar_nearest --arc 90 --stop 0.4
//     build-host/bin/lidar_nearest --replay captures/<stem>.rpraw
//
// Ten times a second it asks checkSector() about the sector and shows one of:
//
//   CLEAR    nearest 1.42 m at +12 deg     known, nothing inside the stop distance
//   STOP     nearest 0.31 m at  -4 deg     known, something inside it
//   UNKNOWN  stale scan (age 1508 ms)      a consumer rule failed: do not trust
//
// Bearings are in the robot frame: 0 straight ahead, positive to the LEFT.
// Until the sensor is mounted, "ahead" is the sensor's own front (the
// symmetry axis of its housing).
//
// Uses only the public headers, as robot code will. Ctrl-C ends it; the
// adapter then stops the scan and the motor.

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <unistd.h>

#include "heron/lidar/Lidar.h"
#include "heron/lidar/SectorCheck.h"

namespace {

using Heron::Clock;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

constexpr float kRadToDeg = 180.0f / Heron::kPi;
constexpr auto kTick = std::chrono::milliseconds(100);
constexpr auto kReplayQuiet = std::chrono::seconds(2);  // replay over: no new scan this long

struct Options {
  float arcDeg{60.0f};
  float centerDeg{0.0f};
  float stopM{0.5f};
  std::int32_t seconds{0};  // 0 = until Ctrl-C (or the end of a replay)
  std::string replay;
  std::int32_t mode{1};
};

[[noreturn]] void usage() {
  std::fprintf(stderr,
               "Usage: lidar_nearest [--arc DEG] [--center DEG] [--stop METRES]\n"
               "                     [--seconds N] [--mode N] [--replay FILE.rpraw]\n"
               "  --arc     sector width, degrees (default 60)\n"
               "  --center  sector centre, degrees, positive to the left (default 0)\n"
               "  --stop    report STOP when something is closer than this (default 0.5)\n");
  std::exit(2);
}

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {  // int: matches argc
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
      }
      return argv[++i];
    };
    if (arg == "--arc") {
      options.arcDeg = std::stof(next());
    } else if (arg == "--center") {
      options.centerDeg = std::stof(next());
    } else if (arg == "--stop") {
      options.stopM = std::stof(next());
    } else if (arg == "--seconds") {
      options.seconds = static_cast<std::int32_t>(std::stoi(next()));
    } else if (arg == "--mode") {
      options.mode = static_cast<std::int32_t>(std::stoi(next()));
    } else if (arg == "--replay") {
      options.replay = next();
    } else {
      usage();
    }
  }
  if (!(options.arcDeg > 0.0f && options.arcDeg <= 360.0f) || !(options.stopM > 0.0f)) {
    usage();
  }
  return options;
}

const char *reasonText(Heron::Lidar::SectorReason reason) {
  switch (reason) {
    case Heron::Lidar::SectorReason::None:
      return "";
    case Heron::Lidar::SectorReason::NoScan:
      return "no scan yet";
    case Heron::Lidar::SectorReason::Stale:
      return "stale scan";
    case Heron::Lidar::SectorReason::NotCovered:
      return "revolution not fully observed";
    case Heron::Lidar::SectorReason::SectorNotObserved:
      return "sector not observed";
  }
  return "?";
}

enum class Verdict : std::uint8_t { Clear, Stop, Unknown };

Verdict verdictOf(const Heron::Lidar::SectorResult &r, float stopM) {
  if (!r.known) {
    return Verdict::Unknown;
  }
  return r.nearestM < stopM ? Verdict::Stop : Verdict::Clear;
}

std::string describe(const Heron::Lidar::SectorResult &r, Verdict verdict) {
  char line[160];
  const double ageMs = std::chrono::duration<double, std::milli>(r.age).count();
  switch (verdict) {
    case Verdict::Unknown:
      if (r.reason == Heron::Lidar::SectorReason::NoScan) {
        std::snprintf(line, sizeof line, "UNKNOWN  %s", reasonText(r.reason));
      } else {
        std::snprintf(line, sizeof line, "UNKNOWN  %s (age %.0f ms)", reasonText(r.reason),
                      ageMs);
      }
      break;
    case Verdict::Clear:
    case Verdict::Stop:
      if (std::isinf(r.nearestM)) {
        std::snprintf(line, sizeof line, "CLEAR    nothing in the sector         (seq %" PRIu64
                      ", age %3.0f ms)", r.seq, ageMs);
      } else {
        std::snprintf(line, sizeof line,
                      "%-8s nearest %5.2f m at %+4.0f deg   (seq %" PRIu64 ", age %3.0f ms)",
                      verdict == Verdict::Stop ? "STOP" : "CLEAR", static_cast<double>(r.nearestM),
                      static_cast<double>(r.nearestBearingRad * kRadToDeg), r.seq, ageMs);
      }
      break;
  }
  return line;
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    const Options options = parse(argc, argv);

    Heron::Lidar::LidarConfig config;
    config.scanMode = options.mode;
    config.replayFile = options.replay;
    const Heron::Lidar::Sector sector{options.centerDeg / kRadToDeg,
                                         options.arcDeg / 2.0f / kRadToDeg};

    std::printf("========================================\n");
    std::printf(" Probe C4 -- nearest obstacle in a sector\n");
    std::printf("========================================\n\n");
    std::printf("  sector   %.0f deg wide, centred %+.0f deg (positive = left)\n",
                static_cast<double>(options.arcDeg), static_cast<double>(options.centerDeg));
    std::printf("  stop     closer than %.2f m\n", static_cast<double>(options.stopM));
    std::printf("  source   %s\n\n", options.replay.empty() ? config.port.c_str()
                                                             : options.replay.c_str());

    Heron::Lidar::Lidar lidar(config);
    if (!lidar.start()) {
      std::fprintf(stderr, "ERROR: %s\n", lidar.getHealth().lastError.c_str());
      return 1;
    }

    const bool tty = isatty(fileno(stdout)) != 0;
    std::uint64_t ticks[3] = {0, 0, 0};
    std::uint64_t changes = 0;
    Verdict last = Verdict::Unknown;
    bool first = true;
    std::uint64_t lastSeq = 0;
    const Clock::time_point begin = Clock::now();
    Clock::time_point lastPrint = begin;
    Clock::time_point lastNewScan = begin;

    while (s_stopRequested == 0) {
      std::this_thread::sleep_for(kTick);
      const Clock::time_point now = Clock::now();
      if (options.seconds > 0 && now - begin > std::chrono::seconds(options.seconds)) {
        break;
      }
      const std::shared_ptr<const Heron::Scan> scan = lidar.getLatest();
      const Heron::Lidar::SectorResult result =
          Heron::Lidar::checkSector(scan.get(), sector, now);
      const Verdict verdict = verdictOf(result, options.stopM);
      ++ticks[static_cast<std::size_t>(verdict)];
      if (scan && scan->seq != lastSeq) {
        lastSeq = scan->seq;
        lastNewScan = now;
      }
      const bool changed = first || verdict != last;
      changes += (changed && !first) ? 1 : 0;
      if (tty) {
        std::printf("\r  %-72s", describe(result, verdict).c_str());
        std::fflush(stdout);
      } else if (changed || now - lastPrint >= std::chrono::seconds(1)) {
        const double t = std::chrono::duration<double>(now - begin).count();
        std::printf("  %6.1f s  %s\n", t, describe(result, verdict).c_str());
        lastPrint = now;
      }
      first = false;
      last = verdict;
      if (!options.replay.empty() && lastSeq != 0 && now - lastNewScan > kReplayQuiet) {
        break;
      }
    }

    lidar.stop();
    const Heron::Lidar::LidarHealth health = lidar.getHealth();
    const double total = static_cast<double>(ticks[0] + ticks[1] + ticks[2]);
    std::printf("%s\n  %s. Time spent: CLEAR %.0f%%, STOP %.0f%%, UNKNOWN %.0f%%; %" PRIu64
                " changes\n",
                tty ? "\n" : "", s_stopRequested != 0 ? "Stopped by signal" : "Finished",
                100.0 * static_cast<double>(ticks[0]) / total,
                100.0 * static_cast<double>(ticks[1]) / total,
                100.0 * static_cast<double>(ticks[2]) / total, changes);
    std::printf("  adapter: published %" PRIu64 ", dropped merged %" PRIu64
                ", dropped backlog %" PRIu64 "\n",
                health.scansPublished, health.scansDroppedMerged, health.scansDroppedBacklog);
    return 0;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
