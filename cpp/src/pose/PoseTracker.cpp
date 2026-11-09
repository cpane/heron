// File        : PoseTracker.cpp
// Author      : Chris Pane
// Description : The pose tracker's thread, and the feed it runs each poll.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/pose/PoseTracker.h"

#include <mutex>
#include <stop_token>
#include <thread>

#include "pose/PoseFeed.h"

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Pose {

// -- the feed ------------------------------------------------------------------

void PoseFeed::update(const Odometry::OdometrySnapshot &o, TimePoint now) {
  if (o.seq == 0) {
    m_snapshot.fresh = false;
    m_snapshot.reason = "no odometry reading yet";
    return;
  }

  // A lower seq: Odometry restarted, and its frame with it (I8). The first
  // reading opens epoch 1 the same way.
  const bool newFrame = m_epoch == 0 || o.seq < m_lastSeq;
  if (newFrame) {
    m_history.clear();
    ++m_epoch;
    m_lastSeq = 0;
    m_lastGap = 0;
  }

  if (o.seq > m_lastSeq) {
    // Readings the poll did not see. After a restart the count starts at
    // the new run's first reading, so none are missed by definition.
    m_lastGap = newFrame ? 0 : o.seq - m_lastSeq - 1;
    m_lastSeq = o.seq;
    // A reading the sensor flags as an error goes no further; its pose is
    // not one to place points with.
    if (!o.sensorError()) {
      m_history.add(PoseSample{now - o.age,
                               Geometry::Pose2{o.pose.x, o.pose.y, o.pose.h},
                               Geometry::Twist{o.velocity.x, o.velocity.y, o.velocity.h}});
    }
  }

  m_snapshot.epoch = m_epoch;
  m_snapshot.newest = m_history.newest().value_or(PoseSample{});
  m_snapshot.tiltWarning = o.tiltWarning();
  m_snapshot.trackingWarning = o.trackingWarning();
  m_snapshot.fresh = o.isOk() && m_lastGap <= m_config.maxSeqGap;
  if (!o.fresh) {
    m_snapshot.reason = "odometry stale";
  } else if (o.sensorError()) {
    m_snapshot.reason = "odometry sensor error";
  } else if (m_lastGap > m_config.maxSeqGap) {
    m_snapshot.reason = "missed " + std::to_string(m_lastGap) + " odometry readings";
  } else {
    m_snapshot.reason.clear();
  }
}

// -- the tracker ---------------------------------------------------------------

struct PoseTracker::Impl {
  Impl(const Odometry::Odometry &odometry, PoseConfig config)
      : m_odometry(odometry), m_config(config), m_feed(config) {}

  void poll() {
    const TimePoint now = Clock::now();
    const Odometry::OdometrySnapshot o = m_odometry.getLatest(now);
    const std::lock_guard<std::mutex> lock(m_mutex);
    m_feed.update(o, now);
  }

  void run(std::stop_token stop) {
#ifdef __linux__
    pthread_setname_np(pthread_self(), "heron-pose");
#endif
    // Kept by deadline, as Odometry's reader is.
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

  const Odometry::Odometry &m_odometry;
  const PoseConfig m_config;

  mutable std::mutex m_mutex;  // guards m_feed
  PoseFeed m_feed;

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

PoseTracker::PoseTracker(const Odometry::Odometry &odometry, PoseConfig config)
    : m_impl(std::make_unique<Impl>(odometry, config)) {}

PoseTracker::~PoseTracker() {
  stop();
}

bool PoseTracker::start() {
  if (m_impl->m_thread.joinable()) {
    return true;
  }
  m_impl->poll();  // so getLatest() has an answer at once
  Impl &impl = *m_impl;
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });
  return true;
}

void PoseTracker::stop() {
  if (m_impl->m_thread.joinable()) {
    m_impl->m_thread.request_stop();
    m_impl->m_thread.join();
  }
}

PoseSnapshot PoseTracker::getLatest() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_feed.snapshot();
}

std::pair<PoseHistory, std::uint64_t> PoseTracker::copyHistory() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return {m_impl->m_feed.history(), m_impl->m_feed.epoch()};
}

}  // namespace Heron::Pose
