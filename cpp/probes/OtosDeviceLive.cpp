// File        : OtosDeviceLive.cpp
// Author      : Chris Pane
// Description : Probe O5. OtosDevice on the real sensor: the library's
//               start-up sequence, then a few seconds of snapshots.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> otos_device
//     scripts/deploy_cpp.sh <user>@<pi> otos_device 10
//     scripts/deploy_cpp.sh <user>@<pi> otos_device 10 0.995 1.002
//
// What otos_start did by hand, now through heron_otos_device: LinuxI2cBus
// and OtosDevice::start() (docs/odometry-cpp-design.md §3). Checks on the
// real board what the unit tests can only fake: that the scalars read back
// as written, and how long calibration and reset take. Optional scalars
// make the read-back check non-trivial; they are not kept, since nothing
// persists across a power cycle (docs/odometry-otos-reference.md §4).
//
// Keep the sensor still and level: start() calibrates the IMU.

#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <numbers>
#include <thread>

#include "I2cBus.h"
#include "OtosDevice.h"

namespace {

using namespace Heron::Odometry;

constexpr double kDegPerRad = 180.0 / std::numbers::pi;

double ms(Duration d) {
  return std::chrono::duration<double, std::milli>(d).count();
}

void printSnapshot(const char *label, const OtosSnapshot &s) {
  std::printf("%-7s status 0x%02X  pose %+8.4f %+8.4f m %+8.3f deg  vel %+7.4f %+7.4f m/s"
              " %+8.3f deg/s\n",
              label, s.status, s.pose.x, s.pose.y, s.pose.h * kDegPerRad, s.velocity.x,
              s.velocity.y, s.velocity.h * kDegPerRad);
}

}  // namespace

int main(int argc, char **argv) {
  const std::int64_t seconds = (argc > 1) ? std::atoll(argv[1]) : 5;
  OtosConfig config;
  if (argc > 3) {
    config.linearScalar = std::atof(argv[2]);
    config.angularScalar = std::atof(argv[3]);
  }

  std::printf("========================================\n");
  std::printf(" Probe O5 -- OtosDevice, live\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s, address 0x%02X\n", config.device.c_str(), kOtosAddress);
  std::printf("  scalars         linear %.3f, angular %.3f\n", config.linearScalar,
              config.angularScalar);
  std::printf("  watch           %" PRId64 " s\n\n", seconds);

  try {
    LinuxI2cBus bus(config.device, kOtosAddress);
    OtosDevice otos(bus);

    std::printf("  Starting: keep the sensor still...\n");
    std::fflush(stdout);
    const OtosStartup up = otos.start(config);
    std::printf("  hardware        %u.%u\n", up.hardware.major, up.hardware.minor);
    std::printf("  firmware        %u.%u\n", up.firmware.major, up.firmware.minor);
    std::printf("  calibration     %.0f ms\n", ms(up.calibrationTime));
    std::printf("  reset           %.0f ms\n", ms(up.resetTime));
    std::printf("  scalars         read back as written\n\n");
    printSnapshot("first", up.first);

    std::uint64_t reads = 0;
    std::uint64_t failed = 0;
    std::uint8_t statusSeen = up.first.status;
    OtosSnapshot last = up.first;
    const auto begin = Clock::now();
    auto nextReport = begin + std::chrono::seconds(1);
    while (Clock::now() - begin < std::chrono::seconds(seconds)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      OtosSnapshot s;
      if (otos.readSnapshot(s)) {
        ++failed;
        continue;
      }
      ++reads;
      statusSeen = static_cast<std::uint8_t>(statusSeen | s.status);
      last = s;
      if (Clock::now() >= nextReport) {
        nextReport += std::chrono::seconds(1);
        char label[16];
        std::snprintf(label, sizeof label, "%3.0f s",
                      std::chrono::duration<double>(Clock::now() - begin).count());
        printSnapshot(label, s);
      }
    }
    std::printf("\n  %" PRIu64 " reads, %" PRIu64 " failed; status bits seen 0x%02X"
                " (bit 0 tilt, 1 optical tracking, 6 optical chip, 7 IMU)\n",
                reads, failed, statusSeen);
    std::printf("  heading change  %+.3f deg\n", (last.pose.h - up.first.pose.h) * kDegPerRad);
    return failed == 0 ? 0 : 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
