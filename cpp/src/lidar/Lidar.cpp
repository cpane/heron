// File        : Lidar.cpp
// Author      : Chris Pane
// Description : The LiDAR adapter: owns the SDK driver and a reader thread,
//               and publishes one Scan per revolution, latest-wins.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The only production file that includes SDK headers.
//
// The SDK call sequence is deliberately the same as cpp/probes/LidarRecord.cpp
// (connect, getDeviceInfo, startScanExpress, grabs, stop, setMotorSpeed(0),
// disconnect), so any session recorded by lidar_record replays through this
// class in lockstep. Changing the sequence means re-recording the fixtures.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <time.h>

#if defined(__linux__)
#include <pthread.h>
#endif

#include <sl_lidar.h>

#include "heron/lidar/Lidar.h"

#include "ReplayChannel.h"
#include "Rpraw.h"
#include "ScanBuilder.h"

namespace Heron::Lidar {

namespace {

constexpr std::size_t kMaxNodes = 8192;      // the SDK's own demo uses 8192
constexpr sl_u32 kGrabTimeoutMs = 500;       // bounds how long stop() waits
constexpr std::size_t kPeriodWindow = 15;    // revolutions in the period median
constexpr double kStallFactor = 1.5;          // interval > 1.5 periods: the reader stalled
constexpr double kNormalLow = 0.5;            // a normal interval is 0.5-1.5 periods
constexpr const char *kContext = "Heron::Lidar::Lidar";

// The SDK stamps scans with CLOCK_MONOTONIC (its arch/*/timer.cpp, both
// platforms). steady_clock is the same clock on the Pi (measured 2026-10-01)
// but CLOCK_MONOTONIC_RAW on macOS, 2.9 s apart there (measured 2026-10-04).
// So convert by age: how long ago the stamp was on the SDK's clock, applied
// to steady_clock now. Exact on Linux, and correct on the host for replays.
TimePoint sdkToSteady(std::uint64_t sdkUs) {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  const TimePoint steadyNow = Clock::now();
  const std::int64_t monoUs = static_cast<std::int64_t>(now.tv_sec) * 1000000 +
                              static_cast<std::int64_t>(now.tv_nsec) / 1000;
  const std::int64_t ageUs = monoUs - static_cast<std::int64_t>(sdkUs);
  return steadyNow - std::chrono::microseconds(ageUs);
}

std::uint64_t medianOf(std::deque<std::uint64_t> values) {
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  return values[mid];
}

}  // namespace

struct Lidar::Impl {
    explicit Impl(const LidarConfig &config) : m_config(config) {}

    bool start();
    void stop();
    void run(std::stop_token stop);
    bool failStart(const std::string &message);
    void teardown();
    void publish(Scan &&scan, bool withLoss);

    const LidarConfig m_config;

    // The driver is declared after the channel so it is destroyed first.
    std::unique_ptr<sl::IChannel> m_channel;
    std::unique_ptr<sl::ILidarDriver> m_driver;
    float m_usPerSample{0.0f};

    mutable std::mutex m_mutex;  // guards m_latest, m_health and m_info
    std::shared_ptr<const Scan> m_latest;
    LidarHealth m_health;
    LidarInfo m_info;

    // Declared last: destroyed (and so joined) first, before anything it uses.
    std::jthread m_reader;
};

bool Lidar::Impl::failStart(const std::string &message) {
  teardown();
  std::lock_guard<std::mutex> lock(m_mutex);
  m_health.state = LidarState::Failed;
  m_health.lastError = std::string(kContext) + "::start() : " + message;
  return false;
}

void Lidar::Impl::teardown() {
  if (m_driver) {
    if (m_driver->isConnected()) {
      m_driver->stop();
      // On the A1, stop() leaves the motor spinning: motor off is DTR, via
      // setMotorSpeed(0). A no-op against a replay channel.
      m_driver->setMotorSpeed(0);
      m_driver->disconnect();
    }
    m_driver.reset();
  }
  m_channel.reset();
}

bool Lidar::Impl::start() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_health.state == LidarState::Running) {
      return true;
    }
    m_health = LidarHealth{};
    m_latest.reset();
  }

  if (m_config.replayFile.empty()) {
    sl::Result<sl::IChannel *> serial =
        sl::createSerialPortChannel(m_config.port, static_cast<int>(m_config.baud));
    if (!serial) {
      return failStart("cannot create a serial channel for " + m_config.port);
    }
    m_channel.reset(*serial);  // adopting the SDK's allocation
  } else {
    try {
      Rpraw raw = readRpraw(m_config.replayFile);
      m_channel = std::make_unique<ReplayChannel>(std::move(raw.chunks), m_config.replaySpeed);
    } catch (const std::exception &e) {
      return failStart(e.what());
    }
  }

  sl::Result<sl::ILidarDriver *> driver = sl::createLidarDriver();
  if (!driver) {
    return failStart("createLidarDriver failed");
  }
  m_driver.reset(*driver);

  if (SL_IS_FAIL(m_driver->connect(m_channel.get()))) {
    return failStart("connect failed");
  }
  // connect() succeeds even with no device attached (cpp-build.md), so a real
  // round trip is the connectivity test.
  sl_lidar_response_device_info_t info;
  if (SL_IS_FAIL(m_driver->getDeviceInfo(info))) {
    return failStart("no answer to GET_INFO from " + m_config.port);
  }
  sl::LidarScanMode used{};
  if (SL_IS_FAIL(m_driver->startScanExpress(false, static_cast<sl_u16>(m_config.scanMode), 0,
                                            &used))) {
    return failStart("cannot start scan mode " + std::to_string(m_config.scanMode));
  }
  m_usPerSample = used.us_per_sample;

  LidarInfo identity;
  identity.model = info.model;
  identity.firmware = std::to_string(info.firmware_version >> 8) + "." +
                      (((info.firmware_version & 0xFF) < 10) ? "0" : "") +
                      std::to_string(info.firmware_version & 0xFF);
  identity.hardware = info.hardware_version;
  for (const sl_u8 byte : info.serialnum) {
    char hex[3];
    std::snprintf(hex, sizeof hex, "%02X", byte);
    identity.serialHex += hex;
  }
  identity.scanModeId = static_cast<std::int32_t>(used.id);
  identity.scanModeName = used.scan_mode;
  identity.usPerSample = used.us_per_sample;
  identity.maxDistanceM = used.max_distance;
  identity.answerType = used.ans_type;

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_info = std::move(identity);
    m_health.state = LidarState::Running;
  }
  m_reader = std::jthread([this](std::stop_token stop) { run(stop); });
  return true;
}

void Lidar::Impl::stop() {
  if (m_reader.joinable()) {
    m_reader.request_stop();
    m_reader.join();  // at most one grab timeout
  }
  teardown();
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_health.state == LidarState::Running) {
    m_health.state = LidarState::Stopped;
  }
}

void Lidar::Impl::publish(Scan &&scan, bool withLoss) {
  std::shared_ptr<const Scan> ready = std::make_shared<const Scan>(std::move(scan));
  std::lock_guard<std::mutex> lock(m_mutex);
  m_latest = std::move(ready);
  ++m_health.scansPublished;
  if (withLoss) {
    ++m_health.scansWithLoss;
  }
}

void Lidar::Impl::run(std::stop_token stop) {
#if defined(__linux__)
  pthread_setname_np(pthread_self(), "lidar-reader");
#endif
  std::vector<sl_lidar_response_measurement_node_hq_t> nodes(kMaxNodes);
  std::vector<RawSample> raw;
  raw.reserve(kMaxNodes);
  std::deque<std::uint64_t> intervalsUs;
  std::uint64_t lastUs = 0;
  bool haveLast = false;
  bool inBacklog = false;
  std::uint64_t seq = 0;
  std::int32_t discarded = 0;

  BuildConfig config;
  config.mountingOffsetRad = m_config.mountingOffsetRad;
  config.usPerSample = m_usPerSample;

  while (!stop.stop_requested()) {
    std::size_t count = nodes.size();
    sl_u64 stampUs = 0;
    const sl_result res =
        m_driver->grabScanDataHqWithTimeStamp(nodes.data(), count, stampUs, kGrabTimeoutMs);
    if (res == SL_RESULT_OPERATION_TIMEOUT) {
      std::lock_guard<std::mutex> lock(m_mutex);
      ++m_health.grabTimeouts;
      continue;
    }
    if (SL_IS_FAIL(res)) {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_health.state = LidarState::Failed;
      m_health.lastError = std::string(kContext) + "::run() : grab failed";
      return;
    }

    raw.clear();
    for (std::size_t i = 0; i < count; ++i) {
      raw.push_back({nodes[i].angle_z_q14, nodes[i].dist_mm_q2, (nodes[i].flag & 1U) != 0});
    }

    // Timing. The period is the median of recent NORMAL intervals, so stalls
    // and bursts cannot drag it. seq advances by the revolutions that elapsed,
    // so a consumer sees skips made by the SDK's own latest-wins buffer too.
    //
    // A stall is an interval over kStallFactor periods. After one, the SDK
    // drains the backlog in a burst and stamps each revolution on ARRIVAL
    // (measured 2026-10-04: intervals of 6 ms, even -4 ms, against 137 ms),
    // so those revolutions claim to be fresh while holding old data. They are
    // dropped until an interval is normal again: the burst has then drained.
    const double nominalUs = static_cast<double>(count) * static_cast<double>(m_usPerSample);
    const double periodUs =
        intervalsUs.empty() ? nominalUs : static_cast<double>(medianOf(intervalsUs));
    bool dropAsBacklog = false;
    if (haveLast) {
      const double intervalUs = static_cast<double>(static_cast<std::int64_t>(stampUs - lastUs));
      const bool normal =
          intervalUs >= kNormalLow * periodUs && intervalUs <= kStallFactor * periodUs;
      if (normal) {
        if (intervalsUs.size() >= kPeriodWindow) {
          intervalsUs.pop_front();
        }
        intervalsUs.push_back(static_cast<std::uint64_t>(intervalUs));
        inBacklog = false;
        seq += 1;
      } else if (intervalUs > kStallFactor * periodUs) {
        inBacklog = true;
        dropAsBacklog = true;
        seq += static_cast<std::uint64_t>(std::llround(intervalUs / periodUs));
      } else {
        dropAsBacklog = inBacklog;  // short or negative: part of a burst
        if (!inBacklog) {
          seq += 1;  // short but not after a stall: an ordinary late grab
        }
      }
    } else {
      seq += 1;
    }
    haveLast = true;
    lastUs = stampUs;

    if (discarded < m_config.discardRevolutions) {
      ++discarded;
      continue;
    }
    if (dropAsBacklog) {
      std::lock_guard<std::mutex> lock(m_mutex);
      ++m_health.scansDroppedBacklog;
      continue;
    }

    BuildInput input;
    input.samples = raw;
    input.tStart = sdkToSteady(stampUs);
    input.period = std::chrono::microseconds(static_cast<std::int64_t>(std::llround(periodUs)));
    input.seq = seq;
    BuiltScan built = buildScan(input, config);
    if (built.merged) {
      // Two revolutions run together after an overrun: times and geometry are
      // not one turn's. Dropped by decision (2026-10-04); seq already moved on.
      std::lock_guard<std::mutex> lock(m_mutex);
      ++m_health.scansDroppedMerged;
      continue;
    }
    const bool withLoss = built.scan.droppedEstimate > 0;
    publish(std::move(built.scan), withLoss);
  }
}

Lidar::Lidar(const LidarConfig &config) : m_impl(std::make_unique<Impl>(config)) {}

Lidar::~Lidar() {
  m_impl->stop();
}

bool Lidar::start() {
  return m_impl->start();
}

void Lidar::stop() {
  m_impl->stop();
}

std::shared_ptr<const Scan> Lidar::getLatest() const {
  std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_latest;
}

LidarHealth Lidar::getHealth() const {
  std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_health;
}

LidarInfo Lidar::getInfo() const {
  std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_info;
}

}  // namespace Heron::Lidar
