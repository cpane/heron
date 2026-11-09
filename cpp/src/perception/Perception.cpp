// File        : Perception.cpp
// Author      : Chris Pane
// Description : The perception thread: each new scan into a copy of the map.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/perception/Perception.h"

#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Perception {

struct Perception::Impl {
  Impl(const Lidar::Lidar &lidar, const Pose::PoseTracker &pose, PerceptionConfig config)
      : m_lidar(lidar), m_pose(pose), m_config(config) {}

  void poll();
  void run(std::stop_token stop);

  const Lidar::Lidar &m_lidar;
  const Pose::PoseTracker &m_pose;
  const PerceptionConfig m_config;

  // Used by the thread only.
  std::optional<std::uint64_t> m_lastSeq;

  // Published: written by the thread, read by any, under m_mutex. The map
  // itself is never changed once here, so readers keep it without the lock.
  mutable std::mutex m_mutex;
  std::shared_ptr<const ObstacleMap> m_map;
  PerceptionHealth m_health{};

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

void Perception::Impl::poll() {
  const std::shared_ptr<const Heron::Scan> scan = m_lidar.getLatest();
  if (!scan || (m_lastSeq && *m_lastSeq == scan->seq)) {
    return;
  }
  m_lastSeq = scan->seq;

  std::shared_ptr<const ObstacleMap> last;
  {
    const std::lock_guard<std::mutex> lock(m_mutex);
    last = m_map;
  }
  auto next = last ? std::make_shared<ObstacleMap>(*last)
                   : std::make_shared<ObstacleMap>(ObstacleMap{ObstacleGrid(m_config.grid)});
  const auto [history, epoch] = m_pose.copyHistory();
  const ScanOutcome outcome =
      addScan(*next, *scan, history, epoch, m_config.mount, Clock::now(), m_config.maxScanAge);
  const bool added = outcome == ScanOutcome::Added || outcome == ScanOutcome::NewFrame;
  // A map in an old frame is worse than an empty one: publish the reset.
  const bool reset = last && last->epoch != next->epoch;

  const std::lock_guard<std::mutex> lock(m_mutex);
  m_health.lastOutcome = outcome;
  if (added) {
    ++m_health.scansAdded;
  } else {
    ++m_health.scansSkipped;
    m_health.lastError = std::string("scan ") + std::to_string(scan->seq) + ": " + toString(outcome);
  }
  if (reset) {
    ++m_health.mapResets;
  }
  if (added || reset) {
    m_map = std::move(next);
  }
}

void Perception::Impl::run(std::stop_token stop) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "heron-perceive");
#endif
  // Kept by deadline, as the other readers are.
  TimePoint next = Clock::now();
  while (!stop.stop_requested()) {
    next += m_config.pollPeriod;
    poll();
    const TimePoint now = Clock::now();
    if (next < now) {
      next = now;
    }
    std::this_thread::sleep_until(next);
  }
}

Perception::Perception(const Lidar::Lidar &lidar, const Pose::PoseTracker &pose,
                       PerceptionConfig config)
    : m_impl(std::make_unique<Impl>(lidar, pose, config)) {}

Perception::~Perception() {
  stop();
}

bool Perception::start() {
  if (m_impl->m_thread.joinable()) {
    return true;
  }
  Impl &impl = *m_impl;
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });
  return true;
}

void Perception::stop() {
  if (m_impl->m_thread.joinable()) {
    m_impl->m_thread.request_stop();
    m_impl->m_thread.join();
  }
}

std::shared_ptr<const ObstacleMap> Perception::getLatest() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_map;
}

PerceptionHealth Perception::getHealth() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_health;
}

}  // namespace Heron::Perception
