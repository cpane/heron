// File        : LocalPlanner.h
// Author      : Chris Pane
// Description : The planner for the robot: a thread running Planner ten
//               times a second on the newest map and pose.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The thin wrapper round Planner (docs/navigation-design.md §4.1, P2): it
// owns the thread and reads the clock. Every 100 ms, while the control
// loop has set a course, it reads the newest map from Perception and the
// pose from PoseTracker, calls plan(), and publishes the result. A new
// course (a new seq: engaged or re-anchored) is planned for at once rather
// than at the next tick, since the control loop drives no plan made for an
// older one. With no course it plans nothing, so in Manual and Assisted it
// only sleeps.
//
// Shaped like Perception and PoseTracker: a thread, and getLatest() that
// never waits for long. Its health says how long plan() takes on this
// machine (design §18, item 2). Design: docs/navigation-cpp-design.md §8.6.
//
// Usage
// -----
//   Heron::Planner::LocalPlanner planner(perception, tracker, config);
//   planner.start();
//   planner.setCourse(course);                 // every 20 ms, from the loop
//   if (const auto plan = planner.getLatest()) {
//     // drive plan->twist if plan->courseSeq is the course's, and it is fresh
//   }

#ifndef HERON_PLANNER_LOCAL_PLANNER_H
#define HERON_PLANNER_LOCAL_PLANNER_H

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "heron/perception/Perception.h"
#include "heron/planner/CourseKeeper.h"
#include "heron/planner/Planner.h"
#include "heron/pose/PoseTracker.h"

namespace Heron::Planner {

struct LocalPlannerConfig {
  PlannerConfig planner{};
  Duration period{std::chrono::milliseconds(100)};
};

/** Counters, for the robot's health reporting. */
struct PlannerHealth {
  std::uint64_t plans{0};
  std::uint64_t blocked{0};
  std::uint64_t stopped{0};
  Reason lastReason{Reason::NoCourse};
  Duration lastCycle{};     // how long the last plan() took
  Duration longestCycle{};
};

/**
 * Thread Safety: setCourse(), getLatest() and getHealth() may be called
 * from any thread and never block for long. start() and stop() are for one
 * owning thread. The Perception and PoseTracker must outlive it.
 */
class LocalPlanner {
  public:
    LocalPlanner(const Perception::Perception &perception, const Pose::PoseTracker &pose,
                 LocalPlannerConfig config);
    ~LocalPlanner();

    LocalPlanner(const LocalPlanner &) = delete;
    LocalPlanner &operator=(const LocalPlanner &) = delete;

    /** Starts the thread. @return true (also if already running) */
    bool start();

    /** Stops and joins the thread; safe to call twice. */
    void stop();

    /** From the control loop, every 20 ms in Navigate; no value stops Heron. */
    void setCourse(const std::optional<Course> &course);

    /** @return The newest plan, or nullptr before the first */
    [[nodiscard]] std::shared_ptr<const Plan> getLatest() const;

    [[nodiscard]] PlannerHealth getHealth() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Heron::Planner

#endif  // HERON_PLANNER_LOCAL_PLANNER_H
