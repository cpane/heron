// File        : MotorAdapter.cpp
// Author      : Chris Pane
// Description : Runs Motors live and reports what a robot loop would see,
//               once a second.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> motor_adapter 10
//     scripts/deploy_cpp.sh <user>@<pi> motor_adapter 10 0.3 0.3 0.3 0.3
//
// The motors' counterpart of remote_adapter and odometry_adapter. Uses only
// Motors.h and RigProfile.h, and links only heron_motor, as robot code will.
// Loads this Pi's rig profile, starts Motors (which resets the Arduino and
// configures it), then drives at 50 Hz for the given seconds: all wheels 0
// unless four values -1.0..1.0 are given (front-left, front-right,
// back-right, back-left). Once a second it prints the snapshot and the
// health counters; at the end, how many polls were not OK.
//
// With values given the wheels turn: wheels off the ground. Press the motor
// shield's reset button mid-run to see a reset counted and the board
// configured again. Ctrl-C ends the run, and stop() releases the motors.

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <thread>

#include "heron/motor/Motors.h"
#include "heron/profile/RigProfile.h"

namespace {

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

const char *toText(Heron::Motor::StopReason reason) {
  using Heron::Motor::StopReason;
  switch (reason) {
    case StopReason::None:
      return "-";
    case StopReason::Boot:
      return "boot";
    case StopReason::StopMessage:
      return "stop";
    case StopReason::Watchdog:
      return "watchdog";
    case StopReason::Unconfigured:
      return "unconfig";
  }
  return "?";
}

}  // namespace

int main(int argc, char **argv) {
  using namespace Heron::Motor;
  using std::chrono::milliseconds;

  if (argc != 1 && argc != 2 && argc != 6) {
    std::fprintf(stderr, "Usage: motor_adapter [seconds] [FL FR BR BL]   (values -1.0..1.0)\n");
    return 2;
  }
  const std::int32_t seconds = (argc > 1) ? static_cast<std::int32_t>(std::atoi(argv[1])) : 10;
  WheelValues wheels{};
  for (int i = 2; i < argc; ++i) {  // int: argc's type
    wheels[static_cast<std::size_t>(i - 2)] = std::atof(argv[i]);
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" Motors, live\n");
  std::printf("========================================\n\n");

  MotorsConfig config;
  try {
    const auto profile = Heron::Profile::loadRigProfile(Heron::Profile::findRigProfile());
    std::printf("Profile:  %s (rig %s, platform %s)\n", profile.source.c_str(),
                profile.name.c_str(), profile.platform.c_str());
    config = toMotorsConfig(profile);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }

  Motors motors(config);
  const auto startAt = Clock::now();
  if (!motors.start()) {
    std::fprintf(stderr, "ERROR: %s\n", motors.getHealth().lastError.c_str());
    return 1;
  }
  const auto startMs =
      std::chrono::duration_cast<milliseconds>(Clock::now() - startAt).count();
  const MotorsHealth atStart = motors.getHealth();
  std::printf("Started in %lld ms: firmware %u.%u.%u, protocol %u, platform %s\n",
              static_cast<long long>(startMs), atStart.boot.firmwareMajor,
              atStart.boot.firmwareMinor, atStart.boot.firmwarePatch,
              atStart.boot.protocolVersion, toString(atStart.boot.platform));
  std::printf("Driving %.2f %.2f %.2f %.2f at 50 Hz for %d s.\n\n", wheels[0], wheels[1],
              wheels[2], wheels[3], seconds);

  std::uint64_t polls = 0;
  std::uint64_t notOk = 0;
  auto next = Clock::now();
  auto nextReport = next + std::chrono::seconds(1);
  const auto end = next + std::chrono::seconds(seconds);
  while (s_stopRequested == 0 && Clock::now() < end) {
    (void)motors.drive(wheels);
    const auto now = Clock::now();
    const MotorsSnapshot s = motors.getLatest(now);
    ++polls;
    if (!s.isOk()) {
      ++notOk;
    }
    if (now >= nextReport) {
      nextReport += std::chrono::seconds(1);
      const MotorsHealth h = motors.getHealth();
      std::printf("%s  %-8s %-8s applied %5d %5d %5d %5d  age %3lld ms  "
                  "resets %" PRIu64 "  configures %" PRIu64 "  stale %" PRIu64 "  %s\n",
                  s.isOk() ? "OK " : "NOT", s.status.state == MotorState::Running ? "running"
                                                                                : "stopped",
                  toText(s.status.stopReason), s.status.applied[0], s.status.applied[1],
                  s.status.applied[2], s.status.applied[3],
                  static_cast<long long>(std::chrono::duration_cast<milliseconds>(s.age).count()),
                  h.resets, h.configuresSent, h.staleEvents, toString(h.state));
    }
    next += milliseconds(20);
    std::this_thread::sleep_until(next);
  }

  motors.stop();
  const MotorsHealth h = motors.getHealth();
  std::printf("\nStopped. %" PRIu64 " polls, %" PRIu64 " not OK. Resets %" PRIu64
              ", Configures sent %" PRIu64 ", stale events %" PRIu64 ".\n",
              polls, notOk, h.resets, h.configuresSent, h.staleEvents);
  if (!h.lastError.empty()) {
    std::printf("Last error: %s\n", h.lastError.c_str());
  }
  return 0;
}
