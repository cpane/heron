// File        : ScanBuilderCheck.cpp
// Author      : Chris Pane
// Description : Runs every scan of a recorded .sdkdump through buildScan()
//               and prints what it found, for comparison with the Python
//               reference by python/tools/lidar_gapcheck.py.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Not a ctest: captures are git-ignored, so it runs only where they exist.
//
//     build-host/bin/scan_builder_check captures/<stem>.sdkdump 254
//
// One line per scan:  index rawCount validCount widestGapDeg droppedEstimate merged
//
// The rotation period comes from the MEDIAN interval between scan timestamps.
// The SDK publishes latest-wins, so a consumer sometimes skips a revolution
// and the interval between two delivered scans is two periods; a median is
// not dragged by those, a mean or a plain difference would be.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "lidar/ScanBuilder.h"

namespace {

struct DumpScan {
  std::uint64_t tUs{0};
  std::vector<Heron::Lidar::RawSample> samples;
};

std::vector<DumpScan> readDump(const std::string &path) {
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "ERROR: cannot open %s\n", path.c_str());
    std::exit(1);
  }
  std::vector<DumpScan> scans;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream fields(line);
    if (line.rfind("scan ", 0) == 0) {
      std::string word;
      std::uint64_t index = 0;
      std::uint64_t count = 0;
      DumpScan scan;
      fields >> word >> index >> count >> scan.tUs;
      scans.push_back(std::move(scan));
      continue;
    }
    std::uint32_t angle = 0;
    std::uint32_t dist = 0;
    std::uint32_t quality = 0;
    std::uint32_t flag = 0;
    fields >> angle >> dist >> quality >> flag;
    scans.back().samples.push_back(
        {static_cast<std::uint16_t>(angle), dist, (flag & 1U) != 0});
  }
  return scans;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "Usage: scan_builder_check FILE.sdkdump US_PER_SAMPLE\n");
    return 2;
  }
  const std::vector<DumpScan> scans = readDump(argv[1]);
  if (scans.size() < 2) {
    std::fprintf(stderr, "ERROR: need at least two scans\n");
    return 1;
  }

  std::vector<std::uint64_t> intervals;
  for (std::size_t i = 1; i < scans.size(); ++i) {
    intervals.push_back(scans[i].tUs - scans[i - 1].tUs);
  }
  std::nth_element(intervals.begin(),
                   intervals.begin() + static_cast<std::ptrdiff_t>(intervals.size() / 2),
                   intervals.end());
  const std::uint64_t periodUs = intervals[intervals.size() / 2];

  Heron::Lidar::BuildConfig config;
  config.usPerSample = std::strtof(argv[2], nullptr);
  std::printf("# period_us %" PRIu64 "\n", periodUs);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    Heron::Lidar::BuildInput input;
    input.samples = scans[i].samples;
    input.tStart = Heron::TimePoint(std::chrono::microseconds(scans[i].tUs));
    input.period = std::chrono::microseconds(periodUs);
    input.seq = i;
    const Heron::Lidar::BuiltScan built = Heron::Lidar::buildScan(input, config);
    const double gapDeg =
        static_cast<double>(built.scan.worstSamplingGapRad) * 180.0 / 3.14159265358979;
    std::printf("%zu %" PRIu32 " %zu %.4f %" PRIu32 " %d\n", i, built.scan.rawSampleCount,
                built.scan.size(), gapDeg, built.scan.droppedEstimate, built.merged ? 1 : 0);
  }
  return 0;
}
