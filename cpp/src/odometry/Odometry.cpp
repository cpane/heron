// File        : Odometry.cpp
// Author      : Chris Pane
// Description : The odometry reader thread and its latest-wins publication.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/odometry/Odometry.h"

#include <chrono>
#include <exception>
#include <mutex>
#include <stop_token>
#include <thread>
#include <utility>

#include "I2cBus.h"
#include "OtosCodec.h"
#include "OtosDevice.h"

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Odometry {

namespace {

// A sensor gone this long may have lost power, and with it its calibration,
// scalars and offset (docs/odometry-otos-reference.md §4), so the reader
// stops rather than publish uncalibrated data (design §4).
constexpr std::chrono::seconds kFailAfter(1);

// OdometrySnapshot's warning tests spell the status bits out, so the public
// header needs no register names; these keep the two in step.
static_assert(kStatusTilt == 0x01 && kStatusOpticalTracking == 0x02);
static_assert((kStatusOpticalChipError | kStatusImuError) == 0xC0);

}  // namespace

struct Odometry::Impl {
  Impl(const OtosConfig &config, BusFactory openBus)
      : m_config(config), m_openBus(std::move(openBus)) {}

  void run(std::stop_token stop);
  // Takes a reading into the published state; m_mutex must be held.
  void publish(const OtosSnapshot &s, TimePoint stamp);

  const OtosConfig m_config;
  const BusFactory m_openBus;

  // Opened by start(), used only by the reader thread while it runs, closed
  // by stop() after the join. Never touched by two threads at once.
  std::unique_ptr<I2cBus> m_bus;
  std::unique_ptr<OtosDevice> m_device;

  // Everything below is published: written by the reader (and by start()
  // before it launches), read by any thread, always under m_mutex.
  mutable std::mutex m_mutex;
  OtosSnapshot m_latest{};
  TimePoint m_stamp{};
  std::uint64_t m_seq{0};
  double m_totalHeading{0.0};
  OdometryHealth m_health{};

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

void Odometry::Impl::publish(const OtosSnapshot &s, TimePoint stamp) {
  // The short way round between two reads: right while the sensor turns
  // less than half a turn per period, 18 000 deg/s at 100 Hz.
  m_totalHeading += m_seq == 0 ? s.pose.h : headingDelta(m_latest.pose.h, s.pose.h);
  m_latest = s;
  m_stamp = stamp;
  ++m_seq;
  ++m_health.readings;
  m_health.statusSeen = static_cast<std::uint8_t>(m_health.statusSeen | s.status);
  if ((s.status & kStatusTilt) != 0) {
    ++m_health.tiltSnapshots;
  }
  if ((s.status & kStatusOpticalTracking) != 0) {
    ++m_health.trackingWarnings;
  }
}

void Odometry::Impl::run(std::stop_token stop) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "heron-otos");
#endif
  // Kept by deadline: a sleep after each read would add the read time,
  // about 5.8 ms, to every period (reference §4, probe O5).
  TimePoint next = Clock::now();
  TimePoint lastGood = next;
  while (!stop.stop_requested()) {
    next += m_config.pollPeriod;
    OtosSnapshot s;
    const std::error_code ec = m_device->readSnapshot(s);
    const TimePoint now = Clock::now();
    {
      const std::lock_guard<std::mutex> lock(m_mutex);
      if (!ec) {
        publish(s, now);
        lastGood = now;
      } else {
        ++m_health.failedReads;
        if (now - lastGood >= kFailAfter) {
          m_health.state = OdometryState::Failed;
          m_health.lastError =
              "Heron::Odometry::Odometry::run() : no good read for 1 s, last error: " +
              ec.message();
          return;
        }
      }
    }
    // Behind by more than a period (a slow read, a busy host): start again
    // from now rather than fire a burst of reads to catch up.
    if (next < now) {
      next = now;
    }
    std::this_thread::sleep_until(next);
  }
}

Odometry::Odometry(const OtosConfig &config)
    : Odometry(config, [device = config.device]() -> std::unique_ptr<I2cBus> {
        return std::make_unique<LinuxI2cBus>(device, kOtosAddress);
      }) {}

Odometry::Odometry(const OtosConfig &config, BusFactory openBus)
    : m_impl(std::make_unique<Impl>(config, std::move(openBus))) {}

Odometry::~Odometry() {
  stop();
}

bool Odometry::start() {
  Impl &impl = *m_impl;
  if (impl.m_thread.joinable()) {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_health.state == OdometryState::Running) {
      return true;
    }
  }
  stop();  // joins a reader that failed, so it can be restarted

  std::unique_ptr<I2cBus> bus;
  std::unique_ptr<OtosDevice> device;
  OtosStartup up;
  try {
    checkOtosConfig(impl.m_config);
    bus = impl.m_openBus();
    device = std::make_unique<OtosDevice>(*bus);
    up = device->start(impl.m_config);  // ~0.65 s: calibration
  } catch (const std::exception &e) {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    impl.m_health.state = OdometryState::Failed;
    impl.m_health.lastError = e.what();
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    impl.m_health = OdometryHealth{};
    impl.m_health.state = OdometryState::Running;
    impl.m_health.hardwareMajor = up.hardware.major;
    impl.m_health.hardwareMinor = up.hardware.minor;
    impl.m_health.firmwareMajor = up.firmware.major;
    impl.m_health.firmwareMinor = up.firmware.minor;
    impl.m_health.calibrationTime = up.calibrationTime;
    impl.m_latest = OtosSnapshot{};
    impl.m_seq = 0;
    impl.m_totalHeading = 0.0;
    impl.publish(up.first, Clock::now());
  }
  impl.m_bus = std::move(bus);
  impl.m_device = std::move(device);
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });
  return true;
}

void Odometry::stop() {
  Impl &impl = *m_impl;
  if (impl.m_thread.joinable()) {
    impl.m_thread.request_stop();
    impl.m_thread.join();
  }
  impl.m_device.reset();
  impl.m_bus.reset();  // closes the bus
  const std::lock_guard<std::mutex> lock(impl.m_mutex);
  if (impl.m_health.state == OdometryState::Running) {
    impl.m_health.state = OdometryState::Stopped;
  }
}

OdometrySnapshot Odometry::getLatest() const {
  return getLatest(Clock::now());
}

OdometrySnapshot Odometry::getLatest(TimePoint now) const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  const Impl &impl = *m_impl;
  OdometrySnapshot o;
  o.seq = impl.m_seq;
  if (impl.m_seq == 0) {
    return o;
  }
  o.age = now > impl.m_stamp ? now - impl.m_stamp : Duration::zero();
  o.fresh = impl.m_health.state == OdometryState::Running && o.age <= impl.m_config.staleAfter;
  o.status = impl.m_latest.status;
  o.pose = impl.m_latest.pose;
  o.velocity = impl.m_latest.velocity;
  o.acceleration = impl.m_latest.acceleration;
  o.poseStdDev = impl.m_latest.poseStdDev;
  o.totalHeading = impl.m_totalHeading;
  return o;
}

OdometryHealth Odometry::getHealth() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_health;
}

const char *toString(OdometryState state) noexcept {
  switch (state) {
    case OdometryState::Stopped:
      return "STOPPED";
    case OdometryState::Running:
      return "RUNNING";
    case OdometryState::Failed:
      return "FAILED";
  }
  return "?";
}

}  // namespace Heron::Odometry
