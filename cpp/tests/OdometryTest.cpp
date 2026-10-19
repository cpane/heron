// File        : OdometryTest.cpp
// Author      : Chris Pane
// Description : Odometry against a fake bus: start-up, publication,
//               freshness, the unwrapped heading, failures, stop and restart.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.
//
// The fake bus stands in for the sensor; each start() asks the factory for a
// new one, and the test keeps a pointer to the current one to change its
// registers while the reader runs (under its mutex). Timing margins are
// generous, several poll periods, so a loaded host does not make the test
// flaky.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <system_error>
#include <thread>

#include "FakeOtosBus.h"
#include "heron/odometry/Odometry.h"

namespace {

using namespace Heron::Odometry;
using std::chrono::milliseconds;

constexpr double kPi = std::numbers::pi;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool contains(const std::string &text, const char *needle) {
  return text.find(needle) != std::string::npos;
}

// The heading register word for an angle in [-pi, pi).
std::int16_t headingWord(double rad) {
  const double raw = std::round(rad / kPi * 32768.0);
  return static_cast<std::int16_t>(raw >= 32768.0 ? -32768.0 : raw);
}

// A factory handing out a new fake per start(), remembering the latest.
struct Fakes {
  FakeOtosBus *current{nullptr};
  std::uint8_t productId{0x5F};

  Odometry::BusFactory factory() {
    return [this]() -> std::unique_ptr<I2cBus> {
      auto bus = std::make_unique<FakeOtosBus>();
      bus->set(0x00, productId);
      current = bus.get();
      return bus;
    };
  }
};

}  // namespace

int main() {
  std::printf("Start\n");
  Fakes fakes;
  Odometry odometry(OtosConfig{}, fakes.factory());
  {
    const OdometrySnapshot before = odometry.getLatest();
    check("before start: nothing, not fresh", before.seq == 0 && !before.fresh && !before.isOk());
    check("start() succeeds", odometry.start());
    const OdometryHealth h = odometry.getHealth();
    check("state RUNNING", h.state == OdometryState::Running);
    check("versions 1.1 and 1.0", h.hardwareMajor == 1 && h.hardwareMinor == 1 &&
                                       h.firmwareMajor == 1 && h.firmwareMinor == 0);
    check("calibration time recorded", h.calibrationTime > Duration::zero());
    check("start() again while running is a no-op", odometry.start());
  }

  std::printf("Publication\n");
  {
    const OdometrySnapshot a = odometry.getLatest();
    check("fresh and ok right after start", a.fresh && a.isOk() && a.seq >= 1);
    std::this_thread::sleep_for(milliseconds(200));
    const OdometrySnapshot b = odometry.getLatest();
    // 100 Hz for 200 ms is about 20; allow a loaded host half of that.
    check("about 100 readings a second", b.seq - a.seq >= 10 && b.seq - a.seq <= 25);
    check("age is under a few periods", b.age < milliseconds(50));

    {
      const std::lock_guard<std::mutex> lock(fakes.current->mutex);
      fakes.current->setWord(0x20, 3277);    // x, about 1 m
      fakes.current->setWord(0x22, -1638);   // y, about -0.5 m
      fakes.current->setWord(0x26, 6554);    // vx, 1 m/s
    }
    std::this_thread::sleep_for(milliseconds(60));
    const OdometrySnapshot c = odometry.getLatest();
    check("pose x follows the sensor", std::fabs(c.pose.x - 3277.0 * 10.0 / 32768.0) < 1e-9);
    check("pose y follows the sensor", std::fabs(c.pose.y + 1638.0 * 10.0 / 32768.0) < 1e-9);
    check("velocity follows the sensor", std::fabs(c.velocity.x - 6554.0 * 5.0 / 32768.0) < 1e-9);

    const OdometrySnapshot late = odometry.getLatest(Clock::now() + milliseconds(500));
    check("judged 500 ms later: not fresh", !late.fresh && !late.isOk());
    check("and its age says so", late.age >= milliseconds(500));
  }

  std::printf("Status bits\n");
  {
    {
      const std::lock_guard<std::mutex> lock(fakes.current->mutex);
      fakes.current->set(0x1F, 0x03);
    }
    std::this_thread::sleep_for(milliseconds(60));
    const OdometrySnapshot w = odometry.getLatest();
    check("tilt and tracking warnings read", w.tiltWarning() && w.trackingWarning());
    check("warnings do not fail isOk()", w.isOk());
    const OdometryHealth h = odometry.getHealth();
    check("warnings counted", h.tiltSnapshots > 0 && h.trackingWarnings > 0 &&
                                  (h.statusSeen & 0x03) == 0x03);

    {
      const std::lock_guard<std::mutex> lock(fakes.current->mutex);
      fakes.current->set(0x1F, 0x80);
    }
    std::this_thread::sleep_for(milliseconds(60));
    const OdometrySnapshot e = odometry.getLatest();
    check("IMU error is a sensor error", e.sensorError());
    check("and fails isOk() though fresh", e.fresh && !e.isOk());
    {
      const std::lock_guard<std::mutex> lock(fakes.current->mutex);
      fakes.current->set(0x1F, 0x00);
    }
  }

  std::printf("Unwrapped heading\n");
  {
    // Two turns positive in 30-degree steps, slower than the reader, then
    // one turn back: the register wraps every half turn, totalHeading not.
    const double start = odometry.getLatest().totalHeading;
    double angle = 0.0;
    for (std::int32_t step = 1; step <= 24; ++step) {
      angle = step * kPi / 6.0;
      {
        const std::lock_guard<std::mutex> lock(fakes.current->mutex);
        fakes.current->setWord(0x24, headingWord(std::remainder(angle, 2.0 * kPi)));
      }
      std::this_thread::sleep_for(milliseconds(25));
    }
    std::this_thread::sleep_for(milliseconds(40));
    const OdometrySnapshot two = odometry.getLatest();
    check("two turns read as 4 pi", std::fabs(two.totalHeading - start - 4.0 * kPi) < 1e-3);
    check("while the pose heading is wrapped", std::fabs(two.pose.h) < 1e-3);

    for (std::int32_t step = 23; step >= 12; --step) {
      angle = step * kPi / 6.0;
      {
        const std::lock_guard<std::mutex> lock(fakes.current->mutex);
        fakes.current->setWord(0x24, headingWord(std::remainder(angle, 2.0 * kPi)));
      }
      std::this_thread::sleep_for(milliseconds(25));
    }
    std::this_thread::sleep_for(milliseconds(40));
    const OdometrySnapshot back = odometry.getLatest();
    check("one turn back reads as 2 pi", std::fabs(back.totalHeading - start - 2.0 * kPi) < 1e-3);
  }

  std::printf("Failures\n");
  {
    {
      const std::lock_guard<std::mutex> lock(fakes.current->mutex);
      fakes.current->failReadsFrom = static_cast<long>(fakes.current->reads);
    }
    std::this_thread::sleep_for(milliseconds(300));
    const OdometrySnapshot s = odometry.getLatest();
    const OdometryHealth h = odometry.getHealth();
    check("failed reads counted", h.failedReads >= 10);
    check("still RUNNING before a second", h.state == OdometryState::Running);
    check("but the reading has gone stale", !s.fresh && !s.isOk());

    std::this_thread::sleep_for(milliseconds(1000));
    const OdometryHealth f = odometry.getHealth();
    check("FAILED after a second without a good read", f.state == OdometryState::Failed);
    check("lastError says why", contains(f.lastError, "no good read for 1 s"));
    check("a failed reader is not fresh", !odometry.getLatest().fresh);
  }

  std::printf("Restart\n");
  {
    check("start() after a failure succeeds", odometry.start());
    const OdometryHealth h = odometry.getHealth();
    check("RUNNING again, error cleared", h.state == OdometryState::Running && h.lastError.empty());
    check("counters start again", h.failedReads == 0 && h.readings < 10);
    check("totalHeading starts again", std::fabs(odometry.getLatest().totalHeading) < 1e-9);

    odometry.stop();
    check("stop(): STOPPED", odometry.getHealth().state == OdometryState::Stopped);
    check("stop(): not fresh", !odometry.getLatest().fresh);
    odometry.stop();
    check("stop() twice is harmless", odometry.getHealth().state == OdometryState::Stopped);
  }

  std::printf("Start-up failures\n");
  {
    Fakes stranger;
    stranger.productId = 0x42;
    Odometry wrong(OtosConfig{}, stranger.factory());
    check("wrong product ID: start() fails", !wrong.start());
    const OdometryHealth h = wrong.getHealth();
    check("FAILED, with the reason", h.state == OdometryState::Failed &&
                                         contains(h.lastError, "not an OTOS"));
  }
  {
    OtosConfig bad;
    bad.angularScalar = 0.5;
    Fakes fine;
    Odometry odo(bad, fine.factory());
    check("bad config: start() fails", !odo.start());
    check("before any bus is opened", fine.current == nullptr);
    check("with the reason", contains(odo.getHealth().lastError, "encodeScalar"));
  }
  {
    Odometry odo(OtosConfig{}, []() -> std::unique_ptr<I2cBus> {
      throw std::system_error(std::make_error_code(std::errc::no_such_device), "test bus");
    });
    check("a bus that will not open: start() fails", !odo.start());
    check("with its error", contains(odo.getHealth().lastError, "test bus"));
  }
  {
    OtosConfig config;
    config.device = "/dev/i2c-does-not-exist";
    Odometry real(config);
    check("the real bus, missing device: start() fails", !real.start());
    check("with a system error in lastError", !real.getHealth().lastError.empty());
  }

  std::printf("\n%s: %d failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
