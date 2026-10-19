// File        : OtosDeviceTest.cpp
// Author      : Chris Pane
// Description : OtosDevice against a fake bus: the start-up sequence, its
//               order and failures, and snapshot reads.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <numbers>
#include <stdexcept>
#include <string>
#include <system_error>

#include "FakeOtosBus.h"
#include "OtosDevice.h"

namespace {

using namespace Heron::Odometry;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

// Short timeouts, so the failure cases finish quickly.
const OtosTiming kFast{std::chrono::milliseconds(1), std::chrono::milliseconds(50),
                       std::chrono::milliseconds(50)};

// Runs fn and reports which exception it threw: "" for none, else a name and
// whether the message carries the expected context.
template <typename E>
bool throwsWith(const std::function<void()> &fn, const char *needle) {
  try {
    fn();
  } catch (const E &e) {
    const std::string what = e.what();
    return what.find("Heron::Odometry::") != std::string::npos &&
           what.find(needle) != std::string::npos;
  } catch (...) {
    return false;
  }
  return false;
}

}  // namespace

int main() {
  std::printf("Start-up, the happy path\n");
  {
    FakeOtosBus bus;
    OtosDevice otos(bus, kFast);
    OtosConfig config;
    config.linearScalar = 0.995;
    config.angularScalar = 1.002;
    config.offset = Planar{0.1, -0.05, std::numbers::pi / 2.0};
    const OtosStartup up = otos.start(config);

    check("hardware version 1.1", up.hardware.major == 1 && up.hardware.minor == 1);
    check("firmware version 1.0", up.firmware.major == 1 && up.firmware.minor == 0);
    check("one reset", bus.resets == 1);

    // The writes, in order: calibration, scalars, offset, reset.
    const auto &w = bus.writes;
    check("four writes", w.size() == 4);
    if (w.size() == 4) {
      check("1: calibration, 255 samples", w[0].reg == 0x06 && w[0].data.size() == 1 &&
                                               w[0].data[0] == 255);
      check("2: both scalars in one write", w[1].reg == 0x04 && w[1].data.size() == 2 &&
                                                w[1].data[0] == static_cast<std::uint8_t>(-5) &&
                                                w[1].data[1] == 2);
      check("3: offset, six bytes in one write", w[2].reg == 0x10 && w[2].data.size() == 6);
      check("3: offset x is 328 LSB", w[2].data.size() == 6 && w[2].data[0] == 0x48 &&
                                          w[2].data[1] == 0x01);
      check("4: reset tracking", w[3].reg == 0x07 && w[3].data.size() == 1 &&
                                     w[3].data[0] == 0x01);
    }
    check("calibration time measured", up.calibrationTime > Duration::zero());
  }

  {
    // The first snapshot is decoded from 0x1F onwards.
    FakeOtosBus bus;
    bus.set(0x1F, 0x01);
    bus.setWord(0x20, 3277);    // x, about 1 m
    bus.setWord(0x24, 16384);   // heading pi/2
    OtosDevice otos(bus, kFast);
    const OtosStartup up = otos.start(OtosConfig{});
    check("first snapshot: status", up.first.status == 0x01);
    check("first snapshot: x", std::fabs(up.first.pose.x - 3277.0 * 10.0 / 32768.0) < 1e-9);
    check("first snapshot: heading",
          std::fabs(up.first.pose.h - std::numbers::pi / 2.0) < 1e-9);
  }

  std::printf("Start-up failures\n");
  {
    FakeOtosBus bus;
    bus.set(0x00, 0x42);
    OtosDevice otos(bus, kFast);
    check("wrong product ID is refused",
          throwsWith<std::runtime_error>([&] { (void)otos.start(OtosConfig{}); }, "not an OTOS"));
    check("nothing written to a stranger", bus.writes.empty());
  }
  {
    FakeOtosBus bus;
    OtosDevice otos(bus, kFast);
    OtosConfig bad;
    bad.linearScalar = 1.2;
    check("bad scalar is refused",
          throwsWith<std::invalid_argument>([&] { (void)otos.start(bad); }, "encodeScalar"));
    check("before the bus is touched", bus.reads == 0 && bus.writes.empty());
  }
  {
    FakeOtosBus bus;
    OtosDevice otos(bus, kFast);
    OtosConfig bad;
    bad.offset.y = 12.0;
    check("offset out of range is refused",
          throwsWith<std::invalid_argument>([&] { (void)otos.start(bad); }, "encodeOffset"));
  }
  {
    FakeOtosBus bus;
    OtosDevice otos(bus, kFast);
    OtosConfig bad;
    bad.pollPeriod = Duration::zero();
    check("zero poll period is refused",
          throwsWith<std::invalid_argument>([&] { (void)otos.start(bad); }, "positive"));
  }
  {
    FakeOtosBus bus;
    bus.calibrationStuck = true;
    OtosDevice otos(bus, kFast);
    check("calibration that never clears times out",
          throwsWith<std::runtime_error>([&] { (void)otos.start(OtosConfig{}); },
                                         "IMU calibration"));
    check("and no reset follows", bus.resets == 0);
  }
  {
    FakeOtosBus bus;
    bus.corruptScalars = true;
    OtosDevice otos(bus, kFast);
    check("scalars that read back wrong are refused",
          throwsWith<std::runtime_error>([&] { (void)otos.start(OtosConfig{}); },
                                         "scalars read back"));
  }
  {
    FakeOtosBus bus;
    bus.failRegister = 0x10;
    OtosDevice otos(bus, kFast);
    check("a failed offset write is a system_error",
          throwsWith<std::system_error>([&] { (void)otos.start(OtosConfig{}); }, "offset"));
  }
  {
    FakeOtosBus bus;
    bus.failRegister = 0x00;
    OtosDevice otos(bus, kFast);
    check("a failed identity read is a system_error",
          throwsWith<std::system_error>([&] { (void)otos.start(OtosConfig{}); }, "product ID"));
  }

  std::printf("Snapshots\n");
  {
    FakeOtosBus bus;
    OtosDevice otos(bus, kFast);
    (void)otos.start(OtosConfig{});
    bus.setWord(0x22, -16384);   // y, -5 m
    OtosSnapshot s;
    check("read succeeds", !otos.readSnapshot(s));
    check("y decoded", std::fabs(s.pose.y + 5.0) < 1e-9);

    bus.failReadsFrom = static_cast<long>(bus.reads);
    OtosSnapshot kept = s;
    bus.setWord(0x22, 0);
    const std::error_code ec = otos.readSnapshot(kept);
    check("a failed read reports its error", ec == std::errc::io_error);
    check("and leaves the snapshot unchanged", kept.pose.y == s.pose.y);
  }

  std::printf("LinuxI2cBus\n");
  {
#if defined(__linux__)
    check("a missing device throws system_error",
          throwsWith<std::system_error>([] { LinuxI2cBus bus("/dev/i2c-does-not-exist", 0x17); },
                                        "cannot open"));
#else
    check("off Linux, opening throws system_error",
          throwsWith<std::system_error>([] { LinuxI2cBus bus("/dev/i2c-1", 0x17); },
                                        "not Linux"));
#endif
  }

  std::printf("\n%s: %d failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
