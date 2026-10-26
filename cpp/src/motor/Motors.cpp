// File        : Motors.cpp
// Author      : Chris Pane
// Description : The motors' start-up handshake, reader thread and sending.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Three threads touch an instance: the owner (start, stop), the control
// loop (drive, stopMotors) and the reader. The port is read only by the
// reader; it is written by all three, always under m_writeMutex, so frames
// never interleave. Everything the reader publishes is under m_mutex. The
// two locks are never held together.

#include "heron/motor/Motors.h"

#include <cmath>
#include <exception>
#include <mutex>
#include <stop_token>
#include <thread>
#include <utility>

#include "heron/motor/MotorPort.h"

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Motor {

namespace {

// Also the bound on how long stop() waits for the reader to notice.
constexpr std::chrono::milliseconds kReadTimeout(20);
// How long start() waits for the Status that confirms Configure.
constexpr std::chrono::milliseconds kConfigureTimeout(1000);
// A Status without our checksum while running: send Configure again, but no
// more often than this, so a board that keeps refusing is not flooded.
constexpr std::chrono::milliseconds kReconfigureInterval(500);
constexpr std::chrono::milliseconds kStartPoll(10);

// -1.0..1.0 to the wire's -1000..1000, rounded and clamped. Finite only.
std::int16_t toWheel(double value) {
  const double scaled = std::round(value * kWheelMax);
  if (scaled >= kWheelMax) {
    return kWheelMax;
  }
  if (scaled <= kWheelMin) {
    return kWheelMin;
  }
  return static_cast<std::int16_t>(scaled);
}

}  // namespace

struct Motors::Impl {
  explicit Impl(const MotorsConfig &config) : m_config(config) {}

  void run(std::stop_token stop);
  void sendConfigure();
  void fail(const std::string &error);
  bool waitFor(Duration timeout, auto &&done);

  const MotorsConfig m_config;
  const MotorConfig m_wire{toMotorConfig(m_config.motor)};
  const std::uint16_t m_checksum{configChecksum(m_wire)};

  // Opened by start(), closed by stop() after the join. Written under
  // m_writeMutex; read only by the reader thread.
  std::unique_ptr<MotorPort> m_port;
  std::mutex m_writeMutex;
  std::uint8_t m_sequence{0};  // under m_writeMutex

  // The reader's own; touched by no other thread.
  FrameParser m_parser;
  TimePoint m_lastConfigure{};
  bool m_wasFresh{false};

  // Published: written by any thread, always under m_mutex.
  mutable std::mutex m_mutex;
  Status m_status{};
  TimePoint m_statusAt{};
  std::uint64_t m_seq{0};
  bool m_configured{false};
  bool m_reconfigureNow{false};
  MotorsHealth m_health{};

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

void Motors::Impl::sendConfigure() {
  MotorConfig config = m_wire;
  std::uint8_t frame[kMaxFrameLen];
  std::size_t length = 0;
  {
    const std::lock_guard<std::mutex> lock(m_writeMutex);
    config.sequence = m_sequence++;
    length = encodeConfigure(config, frame, sizeof(frame));
    m_port->write(frame, length);
  }
  const std::lock_guard<std::mutex> lock(m_mutex);
  ++m_health.configuresSent;
}

void Motors::Impl::fail(const std::string &error) {
  const std::lock_guard<std::mutex> lock(m_mutex);
  m_health.state = MotorsState::Failed;
  m_health.lastError = error;
}

// Polls done() every kStartPoll until it holds or timeout passes.
bool Motors::Impl::waitFor(Duration timeout, auto &&done) {
  const TimePoint until = Clock::now() + timeout;
  while (Clock::now() < until) {
    {
      const std::lock_guard<std::mutex> lock(m_mutex);
      if (m_health.state == MotorsState::Failed) {
        return false;  // the reader died: lastError says why
      }
      if (done()) {
        return true;
      }
    }
    std::this_thread::sleep_for(kStartPoll);
  }
  return false;
}

void Motors::Impl::run(std::stop_token stop) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "heron-motors");
#endif
  try {
    std::uint8_t buffer[256];
    while (!stop.stop_requested()) {
      const std::size_t n = m_port->read(buffer, sizeof(buffer), kReadTimeout);
      const TimePoint now = Clock::now();
      m_parser.feed(buffer, n, [this, now](const Frame &frame) {
        Status status;
        BootInfo boot;
        const std::lock_guard<std::mutex> lock(m_mutex);
        if (decodeStatus(frame, status)) {
          m_status = status;
          m_statusAt = now;
          ++m_seq;
          m_configured = status.configChecksum == m_checksum;
        } else if (decodeBoot(frame, boot)) {
          m_health.boot = boot;
          ++m_health.boots;
          // The board restarted and lost its calibration. Until a Status
          // shows the checksum again, it is not configured -- the last
          // Status, from before the reset, no longer counts.
          m_configured = false;
          if (m_health.state == MotorsState::Running) {
            ++m_health.resets;
            m_reconfigureNow = true;
          }
        }
      });

      bool resend = false;
      {
        const std::lock_guard<std::mutex> lock(m_mutex);
        m_health.link = m_parser.getStats();
        const bool running = m_health.state == MotorsState::Running;
        const bool fresh = m_seq > 0 && now - m_statusAt <= m_config.staleAfter;
        if (running && m_wasFresh && !fresh) {
          ++m_health.staleEvents;
        }
        m_wasFresh = fresh;
        // Configure again after a reset at once, and whenever a fresh Status
        // shows a checksum that is not ours (a lost or corrupted Configure),
        // at most every kReconfigureInterval.
        if (running && (m_reconfigureNow ||
                        (fresh && !m_configured && now - m_lastConfigure >= kReconfigureInterval))) {
          resend = true;
          m_reconfigureNow = false;
        }
      }
      if (resend) {
        m_lastConfigure = now;
        sendConfigure();
      }
    }
  } catch (const std::exception &e) {
    // The port failed or vanished. No Status will come, so the snapshot goes
    // stale by itself; this records why, and stops drive() from sending.
    fail(e.what());
  }
}

Motors::Motors(const MotorsConfig &config) : m_impl(std::make_unique<Impl>(config)) {}

Motors::~Motors() {
  stop();
}

bool Motors::start() {
  Impl &impl = *m_impl;
  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_thread.joinable() && impl.m_health.state == MotorsState::Running) {
      return true;
    }
  }
  stop();  // joins a reader that failed, so it can be restarted

  const std::string where = "Heron::Motor::Motors::start() : ";
  const std::optional<Platform> platform = toPlatform(impl.m_config.platform);
  if (!platform) {
    impl.fail(where + "unknown platform '" + impl.m_config.platform + "' in the configuration");
    return false;
  }
  if (!isValidConfig(impl.m_wire)) {
    impl.fail(where + "the [motor] section does not make a valid Configure: each terminal "
                      "1..4 once, ramp at most " + std::to_string(kMaxRampPerMs));
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    impl.m_status = Status{};
    impl.m_statusAt = TimePoint{};
    impl.m_seq = 0;
    impl.m_configured = false;
    impl.m_reconfigureNow = false;
    impl.m_health = MotorsHealth{};  // state Stopped until configured
  }
  impl.m_parser = FrameParser{};
  impl.m_lastConfigure = TimePoint{};
  impl.m_wasFresh = false;

  try {
    impl.m_port = std::make_unique<MotorPort>(MotorPortConfig{impl.m_config.device});
  } catch (const std::exception &e) {
    impl.fail(e.what());
    return false;
  }
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });

  // Any failure from here: stop the thread, close the port, report.
  const auto abandon = [this, &impl](const std::string &error) {
    stop();
    if (!error.empty()) {
      impl.fail(error);
    }
    return false;
  };

  // 1. The Boot that opening the port causes.
  if (!impl.waitFor(impl.m_config.startTimeout, [&impl] { return impl.m_health.boots > 0; })) {
    std::string error;
    {
      const std::lock_guard<std::mutex> lock(impl.m_mutex);
      if (impl.m_health.state == MotorsState::Failed) {
        error = impl.m_health.lastError;
      } else if (impl.m_health.link.versionMismatches > 0) {
        error = where + "the board on " + impl.m_config.device +
                " speaks another protocol version (this program speaks " +
                std::to_string(kProtocolVersion) +
                "): reflash it with scripts/flash_firmware.sh";
      } else {
        error = where + "no Boot from " + impl.m_config.device + " within " +
                std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                   impl.m_config.startTimeout)
                                   .count()) +
                " ms: is the Arduino plugged in and its firmware flashed?";
      }
    }
    return abandon(error);
  }

  // 2. Is it the board this profile is for?
  BootInfo boot;
  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    boot = impl.m_health.boot;
  }
  if (boot.protocolVersion != kProtocolVersion) {
    return abandon(where + "the board reports protocol " + std::to_string(boot.protocolVersion) +
                   ", this program speaks " + std::to_string(kProtocolVersion) +
                   ": reflash it with scripts/flash_firmware.sh");
  }
  if (boot.platform != *platform) {
    return abandon(where + "the board is platform '" + toString(boot.platform) +
                   "', but the profile is for '" + impl.m_config.platform +
                   "': wrong board, or wrong firmware");
  }

  // 3. The rig's calibration, confirmed by the checksum in Status.
  try {
    impl.m_lastConfigure = Clock::now();
    impl.sendConfigure();
  } catch (const std::exception &e) {
    return abandon(e.what());
  }
  if (!impl.waitFor(kConfigureTimeout, [&impl] { return impl.m_configured; })) {
    std::string error;
    {
      const std::lock_guard<std::mutex> lock(impl.m_mutex);
      error = impl.m_health.state == MotorsState::Failed
                  ? impl.m_health.lastError
                  : where + "the board did not confirm the calibration within " +
                        std::to_string(kConfigureTimeout.count()) + " ms";
    }
    return abandon(error);
  }

  const std::lock_guard<std::mutex> lock(impl.m_mutex);
  if (impl.m_health.state == MotorsState::Failed) {
    return false;  // the reader died in the last moment; lastError is set
  }
  impl.m_health.state = MotorsState::Running;
  return true;
}

void Motors::stop() {
  Impl &impl = *m_impl;
  bool running = false;
  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    running = impl.m_health.state == MotorsState::Running;
  }
  if (running) {
    (void)stopMotors();  // leave the motors released, not to the watchdog
  }
  {
    // From here drive() sends nothing; one already past its check is
    // stopped by the port being null under the write lock, below.
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_health.state == MotorsState::Running) {
      impl.m_health.state = MotorsState::Stopped;
    }
  }
  if (impl.m_thread.joinable()) {
    impl.m_thread.request_stop();
    impl.m_thread.join();
  }
  const std::lock_guard<std::mutex> lock(impl.m_writeMutex);
  impl.m_port.reset();  // closes the port
}

bool Motors::drive(const WheelValues &wheels) {
  for (const double value : wheels) {
    if (!std::isfinite(value)) {
      (void)stopMotors();  // a bad number upstream: stopping is the safe answer
      return false;
    }
  }
  Impl &impl = *m_impl;
  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_health.state != MotorsState::Running) {
      return false;
    }
  }
  DriveCommand cmd;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    cmd.wheels[i] = toWheel(wheels[i]);
  }
  try {
    std::uint8_t frame[kMaxFrameLen];
    const std::lock_guard<std::mutex> lock(impl.m_writeMutex);
    if (!impl.m_port) {
      return false;  // stop() closed it after the check above
    }
    cmd.sequence = impl.m_sequence++;
    impl.m_port->write(frame, encodeDrive(cmd, frame, sizeof(frame)));
  } catch (const std::exception &e) {
    impl.fail(e.what());
    return false;
  }
  return true;
}

bool Motors::stopMotors() {
  Impl &impl = *m_impl;
  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_health.state != MotorsState::Running) {
      return false;
    }
  }
  try {
    std::uint8_t frame[kMaxFrameLen];
    const std::lock_guard<std::mutex> lock(impl.m_writeMutex);
    if (!impl.m_port) {
      return false;
    }
    impl.m_port->write(frame,
                       encodeStop(StopCommand{impl.m_sequence++}, frame, sizeof(frame)));
  } catch (const std::exception &e) {
    impl.fail(e.what());
    return false;
  }
  return true;
}

MotorsSnapshot Motors::getLatest() const {
  return getLatest(Clock::now());
}

MotorsSnapshot Motors::getLatest(TimePoint now) const {
  const Impl &impl = *m_impl;
  const std::lock_guard<std::mutex> lock(impl.m_mutex);
  MotorsSnapshot snapshot;
  snapshot.seq = impl.m_seq;
  if (impl.m_seq > 0) {
    snapshot.age = now - impl.m_statusAt;
    snapshot.fresh = snapshot.age <= impl.m_config.staleAfter;
  }
  snapshot.configured = impl.m_configured;
  snapshot.status = impl.m_status;
  return snapshot;
}

MotorsHealth Motors::getHealth() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_health;
}

MotorsConfig toMotorsConfig(const Heron::Profile::RigProfile &profile) {
  MotorsConfig config;
  config.platform = profile.platform;
  config.motor = profile.motor;
  return config;
}

MotorConfig toMotorConfig(const Heron::Profile::MotorProfile &motor) {
  static_assert(Heron::Profile::kProfileMotorCount == kMotorCount, "one value per motor");
  static_assert(Heron::Profile::kMaxRampPerMs == kMaxRampPerMs, "the same ramp limit");
  MotorConfig config;
  for (std::size_t i = 0; i < kMotorCount; ++i) {
    config.terminal[i] = motor.terminal[i];
    if (motor.sign[i] < 0) {
      config.reversed = static_cast<std::uint8_t>(config.reversed | (1U << i));
    }
    config.minDutyForward[i] = motor.minDutyForward[i];
    config.minDutyBackward[i] = motor.minDutyBackward[i];
  }
  config.rampPerMs = motor.rampPerMs;
  return config;
}

std::optional<Platform> toPlatform(std::string_view name) {
  if (name == "uno-l293d") {
    return Platform::UnoL293d;
  }
  return std::nullopt;
}

const char *toString(Platform platform) noexcept {
  switch (platform) {
    case Platform::UnoL293d:
      return "uno-l293d";
    case Platform::Unknown:
      break;
  }
  return "unknown";
}

const char *toString(MotorsState state) noexcept {
  switch (state) {
    case MotorsState::Stopped:
      return "STOPPED";
    case MotorsState::Running:
      return "RUNNING";
    case MotorsState::Failed:
      return "FAILED";
  }
  return "?";
}

}  // namespace Heron::Motor
