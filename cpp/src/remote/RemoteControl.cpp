// File        : RemoteControl.cpp
// Author      : Chris Pane
// Description : The remote control's reader thread and its latest-wins
//               publication.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/remote/RemoteControl.h"

#include <chrono>
#include <exception>
#include <mutex>
#include <stop_token>
#include <thread>
#include <utility>

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Remote {

namespace {

// Also the bound on how long stop() waits for the reader to notice.
constexpr std::chrono::milliseconds kReadTimeout(20);

}  // namespace

struct RemoteControl::Impl {
  explicit Impl(const ReceiverConfig &config) : m_config(config) {}

  void run(std::stop_token stop);

  const ReceiverConfig m_config;

  // Opened by start(), used only by the reader thread while it runs, closed
  // by stop() after the join. Never touched by two threads at once.
  std::unique_ptr<IBusReceiver> m_receiver;

  // Everything below is published: written by the reader, read by any
  // thread, always under m_mutex.
  mutable std::mutex m_mutex;
  LinkMonitor m_link{m_config.staleAfter};
  IBusFrame m_latest{};
  std::uint64_t m_seq{0};
  IBusStats m_stats{};
  std::uint64_t m_staleEvents{0};
  RemoteState m_state{RemoteState::Stopped};
  std::string m_lastError;

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

void RemoteControl::Impl::run(std::stop_token stop) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "heron-ibus");
#endif
  LinkState previous = LinkState::NoSignal;
  try {
    while (!stop.stop_requested()) {
      m_receiver->pump(kReadTimeout, [this](const IBusFrame &frame) {
        const std::lock_guard<std::mutex> lock(m_mutex);
        m_latest = frame;
        ++m_seq;
        m_link.onFrame(frame.stamp);
      });
      const std::lock_guard<std::mutex> lock(m_mutex);
      m_stats = m_receiver->getStats();
      const LinkState state = m_link.getState(Clock::now());
      if (state == LinkState::Stale && previous == LinkState::Ok) {
        ++m_staleEvents;
      }
      previous = state;
    }
  } catch (const std::exception &e) {
    // The port failed or vanished. No new frames will come, so consumers see
    // the link go stale on their own; this records why.
    const std::lock_guard<std::mutex> lock(m_mutex);
    m_state = RemoteState::Failed;
    m_lastError = e.what();
  }
}

RemoteControl::RemoteControl(const ReceiverConfig &config)
    : m_impl(std::make_unique<Impl>(config)) {}

RemoteControl::~RemoteControl() {
  stop();
}

bool RemoteControl::start() {
  Impl &impl = *m_impl;
  if (impl.m_thread.joinable()) {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    if (impl.m_state == RemoteState::Running) {
      return true;
    }
  }
  stop();  // joins a reader that failed, so it can be restarted

  std::unique_ptr<IBusReceiver> receiver;
  try {
    receiver = std::make_unique<IBusReceiver>(impl.m_config);
  } catch (const std::exception &e) {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    impl.m_state = RemoteState::Failed;
    impl.m_lastError = e.what();
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(impl.m_mutex);
    impl.m_link = LinkMonitor(impl.m_config.staleAfter);
    impl.m_latest = IBusFrame{};
    impl.m_seq = 0;
    impl.m_stats = IBusStats{};
    impl.m_staleEvents = 0;
    impl.m_lastError.clear();
    impl.m_state = RemoteState::Running;
  }
  impl.m_receiver = std::move(receiver);
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });
  return true;
}

void RemoteControl::stop() {
  Impl &impl = *m_impl;
  if (impl.m_thread.joinable()) {
    impl.m_thread.request_stop();
    impl.m_thread.join();
  }
  impl.m_receiver.reset();  // closes the port
  const std::lock_guard<std::mutex> lock(impl.m_mutex);
  if (impl.m_state == RemoteState::Running) {
    impl.m_state = RemoteState::Stopped;
  }
}

RemoteSnapshot RemoteControl::getLatest() const {
  return getLatest(Clock::now());
}

RemoteSnapshot RemoteControl::getLatest(TimePoint now) const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  RemoteSnapshot snapshot;
  snapshot.link = m_impl->m_link.getState(now);
  snapshot.frame = m_impl->m_latest;
  snapshot.seq = m_impl->m_seq;
  snapshot.age = m_impl->m_link.sinceLastFrame(now);
  return snapshot;
}

RemoteHealth RemoteControl::getHealth() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  RemoteHealth health;
  health.state = m_impl->m_state;
  health.stats = m_impl->m_stats;
  health.staleEvents = m_impl->m_staleEvents;
  health.lastError = m_impl->m_lastError;
  return health;
}

const char *toString(RemoteState state) noexcept {
  switch (state) {
    case RemoteState::Stopped:
      return "STOPPED";
    case RemoteState::Running:
      return "RUNNING";
    case RemoteState::Failed:
      return "FAILED";
  }
  return "?";
}

}  // namespace Heron::Remote
