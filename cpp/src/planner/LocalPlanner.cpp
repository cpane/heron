// File        : LocalPlanner.cpp
// Author      : Chris Pane
// Description : The planner thread: Planner on the newest map and pose.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/planner/LocalPlanner.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <stop_token>
#include <thread>

#ifdef __linux__
#include <pthread.h>
#endif

namespace Heron::Planner {

struct LocalPlanner::Impl {
  Impl(const Perception::Perception &perception, const Pose::PoseTracker &pose,
       LocalPlannerConfig config)
      : m_perception(perception), m_pose(pose), m_config(config), m_planner(config.planner) {}

  void cycle(const std::optional<Course> &course);
  void run(std::stop_token stop);

  const Perception::Perception &m_perception;
  const Pose::PoseTracker &m_pose;
  const LocalPlannerConfig m_config;

  // Used by the thread only.
  Planner m_planner;

  // Shared, under m_mutex. A published plan is never changed, so readers
  // keep it without the lock.
  mutable std::mutex m_mutex;
  std::condition_variable_any m_wake;  // a new course seq
  std::optional<Course> m_course;
  std::shared_ptr<const Plan> m_plan;
  PlannerHealth m_health{};

  std::jthread m_thread;  // declared last: joined first, before the members it uses
};

void LocalPlanner::Impl::cycle(const std::optional<Course> &course) {
  const std::shared_ptr<const Perception::ObstacleMap> map = m_perception.getLatest();
  const auto [history, epoch] = m_pose.copyHistory();
  const Pose::PoseSnapshot snapshot = m_pose.getLatest();
  const TimePoint now = Clock::now();
  const Pose::PoseLookup at = history.at(now);

  PlanInput input;
  input.map = map.get();
  input.pose = at.pose;
  input.epoch = epoch;
  input.poseFresh = snapshot.fresh && at.ok;
  input.measured = snapshot.newest.velocity;
  input.course = course;
  input.now = now;
  auto plan = std::make_shared<const Plan>(m_planner.plan(input));
  const Duration took = Clock::now() - now;

  const std::lock_guard<std::mutex> lock(m_mutex);
  ++m_health.plans;
  m_health.blocked += plan->state == State::Blocked ? 1u : 0u;
  m_health.stopped += plan->state == State::Stopped ? 1u : 0u;
  m_health.lastReason = plan->reason;
  m_health.lastCycle = took;
  m_health.longestCycle = std::max(m_health.longestCycle, took);
  m_plan = std::move(plan);
}

void LocalPlanner::Impl::run(std::stop_token stop) {
#ifdef __linux__
  pthread_setname_np(pthread_self(), "heron-planner");
#endif
  // Kept by deadline, as the other threads are; a new course seq is
  // planned for at once.
  TimePoint next = Clock::now();
  std::uint64_t plannedSeq = 0;
  while (!stop.stop_requested()) {
    std::optional<Course> course;
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_wake.wait_until(lock, stop, next, [this, plannedSeq]() {
        return m_course && m_course->seq != plannedSeq;
      });
      if (stop.stop_requested()) {
        return;
      }
      course = m_course;
    }
    const TimePoint now = Clock::now();
    if (now >= next) {
      next += m_config.period;
      if (next < now) {
        next = now + m_config.period;
      }
    }
    if (!course) {
      m_planner.reset();  // nothing to plan for: Manual, Assisted, or let go
      continue;
    }
    plannedSeq = course->seq;
    cycle(course);
  }
}

LocalPlanner::LocalPlanner(const Perception::Perception &perception, const Pose::PoseTracker &pose,
                           LocalPlannerConfig config)
    : m_impl(std::make_unique<Impl>(perception, pose, config)) {}

LocalPlanner::~LocalPlanner() {
  stop();
}

bool LocalPlanner::start() {
  if (m_impl->m_thread.joinable()) {
    return true;
  }
  Impl &impl = *m_impl;
  impl.m_thread = std::jthread([&impl](std::stop_token stop) { impl.run(std::move(stop)); });
  return true;
}

void LocalPlanner::stop() {
  if (m_impl->m_thread.joinable()) {
    m_impl->m_thread.request_stop();
    m_impl->m_thread.join();
  }
}

void LocalPlanner::setCourse(const std::optional<Course> &course) {
  bool wake = false;
  {
    const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
    wake = course && (!m_impl->m_course || m_impl->m_course->seq != course->seq);
    m_impl->m_course = course;
  }
  if (wake) {
    m_impl->m_wake.notify_one();
  }
}

std::shared_ptr<const Plan> LocalPlanner::getLatest() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_plan;
}

PlannerHealth LocalPlanner::getHealth() const {
  const std::lock_guard<std::mutex> lock(m_impl->m_mutex);
  return m_impl->m_health;
}

}  // namespace Heron::Planner
