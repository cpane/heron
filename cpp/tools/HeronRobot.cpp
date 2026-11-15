// File        : HeronRobot.cpp
// Author      : Chris Pane
// Description : heron_robot: heron_teleop grown up. The remote, the LiDAR,
//               the OTOS and the safety limiter in one 50 Hz loop, SwC
//               choosing Manual, Assisted or Navigate.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The robot program of docs/navigation-design.md §9. Every 20 ms it reads
// the remote, works out the mode from SwC, and drives:
//
//   Manual    exactly heron_teleop: the sticks, mixed. Nothing else is read,
//             so it works whatever else has failed. Switch to it to take
//             over at any time.
//   Assisted  the sticks through the safety limiter, against the newest
//             LiDAR scan: anything that would bring a point inside the stop
//             outline is slowed, cut or stopped, axis by axis, so sliding
//             along a wall and backing away still work.
//   Navigate  not built yet (step 2): stops.
//
// Its decisions are in src/robot/RobotCore.h, tested by robot_test; this
// file is the start-up, the loop and the status line.
//
//     scripts/deploy_cpp.sh <user>@<pi> heron_robot --dry-run
//     scripts/deploy_cpp.sh <user>@<pi> heron_robot
//
// heron_teleop runs at boot (heron-teleop@.service) and holds the Arduino
// and the receiver, so turn it off first, in your own terminal (sudo):
//     scripts/teleop_autostart.sh <user>@<pi> off
//
// Driving
// -------
//   As heron_teleop: SwA down arms, throttle to the bottom, then the
//   throttle sets the speed limit; right stick drives and strafes, left
//   stick left-right turns. SwC chooses the mode: middle Assisted, one end
//   Manual, the other Navigate. SwC up reads Low (design §18 item 4), so
//   by default up is Manual and down Navigate; --manual-is-high swaps
//   them. The mode is printed whenever it changes: check it before arming.
//
// The status line
// ---------------
//   Once a second, or --lines N times: the mode, armed, the throttle, the
//   stick and what went out (fractions of full drive), the four wheels,
//   the measured speed from the OTOS (m/s, x forward, y left; rad/s,
//   left positive), the heading since start-up (degrees, left positive),
//   the OTOS's TRACKING and TILT warnings when set, and in Assisted the
//   limiter's reason, the nearest point and the scan's age.
//   "out" x max_speed is the speed asked for; "speed" is what it did.
//
// Start-up
// --------
//   1. The profile: Assisted needs [geometry], [lidar], [odometry] and
//      [navigation], and the LiDAR's blind circle inside the stop outline.
//   2. The remote.
//   3. The OTOS, first among the sensors: its gyro calibrates at start, so
//      keep the robot still (design §9). Then the pose tracker.
//   4. The LiDAR.
//   5. The motors (resets the Arduino), unless --dry-run.
// A sensor that fails to start is reported and leaves Assisted unavailable:
// Manual still works, and Assisted stops rather than falling back to Manual.
// A LiDAR that stops while running leaves its scans going stale, and the
// limiter stops Assisted (Stale, design §7.4).
//
// --dry-run starts everything but the motors and prints what it would
// send: for checking the sticks, SwC and the limiter with nothing turning.
// Ctrl-C ends it; the motors are released on the way out.

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "heron/drive/Drive.h"
#include "heron/lidar/Lidar.h"
#include "heron/limiter/Limiter.h"
#include "heron/motor/Motors.h"
#include "heron/odometry/Odometry.h"
#include "heron/pose/PoseTracker.h"
#include "heron/profile/RigProfile.h"
#include "heron/remote/Normalize.h"
#include "heron/remote/RemoteControl.h"
#include "robot/RobotCore.h"

namespace {

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

constexpr std::chrono::milliseconds kPeriod(20);  // 50 Hz, the protocol's rate

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: heron_robot [--dry-run] [--manual-is-high] [--lines N] [--profile FILE]\n"
               "                   [--surface NAME] [--remote DEVICE]\n"
               "  --dry-run         everything but the motors; print the wheels\n"
               "  --manual-is-high  SwC High is Manual (default: Low is Manual)\n"
               "  --lines N         status lines a second, 1 to 50 (default 1)\n"
               "  --profile FILE    the rig profile (default this Pi's)\n"
               "  --surface NAME    the floor (default the profile's [rig] surface)\n"
               "  --remote DEVICE   the iBUS receiver's port (default /dev/serial0)\n");
  std::exit(status);
}

}  // namespace

int main(int argc, char **argv) {
  using namespace std::chrono;
  using Heron::Robot::Mode;
  using Heron::Robot::Why;

  bool dryRun = false;
  bool manualIsHigh = false;
  int linesPerSecond = 1;
  std::string profilePath;
  std::string surfaceName;
  Heron::Remote::ReceiverConfig remoteConfig;  // /dev/serial0, stale after 100 ms
  // SwA on CH7, as heron_teleop: 1000 up (disarmed) and the transmitter's
  // failsafe value, so disarming and radio loss read the same.
  remoteConfig.failsafe = Heron::Remote::FailsafeConfig{6, 1000, 50};

  for (int i = 1; i < argc; ++i) {  // int: argc's type
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage(0);
    } else if (arg == "--dry-run") {
      dryRun = true;
    } else if (arg == "--manual-is-high") {
      manualIsHigh = true;
    } else if (arg == "--lines" && i + 1 < argc) {
      char *end = nullptr;
      const long n = std::strtol(argv[++i], &end, 10);
      if (*end != '\0' || n < 1 || n > 50) {
        std::fprintf(stderr, "ERROR: --lines takes 1 to 50, not '%s'\n", argv[i]);
        usage(2);
      }
      linesPerSecond = static_cast<int>(n);
    } else if ((arg == "--profile" || arg == "--remote" || arg == "--surface") && i + 1 < argc) {
      (arg == "--profile" ? profilePath : arg == "--surface" ? surfaceName : remoteConfig.serial.device) =
          argv[++i];
    } else {
      std::fprintf(stderr, "ERROR: unexpected argument '%s'\n", arg.c_str());
      usage(2);
    }
  }
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" heron_robot%s\n", dryRun ? " (dry run: no motors)" : "");
  std::printf("========================================\n\n");

  // 1. The profile.
  Heron::Profile::RigProfile profile;
  try {
    profile = Heron::Profile::loadRigProfile(profilePath.empty()
                                                 ? Heron::Profile::findRigProfile()
                                                 : std::filesystem::path(profilePath));
    // The floor (design §19, P18): its numbers in place before any
    // library's settings are filled (I22).
    if (surfaceName.empty()) {
      surfaceName = profile.surface;
    }
    if (!surfaceName.empty()) {
      profile = Heron::Profile::onSurface(profile, surfaceName);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
  std::printf("Profile: %s (rig %s)\n", profile.source.c_str(), profile.name.c_str());
  std::printf("Surface: %s\n", profile.activeSurface.empty() ? "none (the profile's own numbers)"
                                                              : profile.activeSurface.c_str());
  std::string unavailable = Heron::Robot::assistedRefusal(profile);
  std::printf("SwC: Low %s, middle ASSISTED, High %s\n",
              Heron::Robot::toString(Heron::Robot::modeFromSwitch(Heron::Remote::Switch3::Low, manualIsHigh)),
              Heron::Robot::toString(Heron::Robot::modeFromSwitch(Heron::Remote::Switch3::High, manualIsHigh)));

  // 2. The remote.
  Heron::Remote::RemoteControl remote(remoteConfig);
  if (!remote.start()) {
    std::fprintf(stderr, "ERROR: remote: %s\n", remote.getHealth().lastError.c_str());
    return 1;
  }

  // 3. The OTOS, still, then the pose tracker.
  std::unique_ptr<Heron::Odometry::Odometry> odometry;
  std::unique_ptr<Heron::Pose::PoseTracker> pose;
  if (unavailable.empty()) {
    std::printf("Starting the OTOS: keep the robot still...\n");
    odometry = std::make_unique<Heron::Odometry::Odometry>(Heron::Robot::toOtosConfig(*profile.odometry));
    if (!odometry->start()) {
      unavailable = "OTOS: " + odometry->getHealth().lastError;
    } else {
      pose = std::make_unique<Heron::Pose::PoseTracker>(*odometry, Heron::Robot::toPoseConfig(profile));
      if (!pose->start()) {
        unavailable = "the pose tracker did not start";
      }
    }
  }

  // 4. The LiDAR.
  std::unique_ptr<Heron::Lidar::Lidar> lidar;
  if (unavailable.empty()) {
    std::printf("Starting the LiDAR...\n");
    lidar = std::make_unique<Heron::Lidar::Lidar>(Heron::Robot::toLidarConfig(*profile.lidar));
    if (!lidar->start()) {
      unavailable = "LiDAR: " + lidar->getHealth().lastError;
    }
  }

  std::unique_ptr<Heron::Limiter::Limiter> limiter;
  std::optional<Heron::Motion::MotionTable> motion;
  if (unavailable.empty()) {
    limiter = std::make_unique<Heron::Limiter::Limiter>(Heron::Robot::toLimiterConfig(profile));
    motion = Heron::Robot::toMotionTable(profile);  // checked by assistedRefusal()
    std::printf("Assisted: ready, %s\n",
                motion ? "through the surface's motion table" : "with the min_speed model (no table)");
  } else {
    std::printf("Assisted: UNAVAILABLE (%s). Manual works; Assisted stops.\n", unavailable.c_str());
  }

  // 5. The motors.
  std::unique_ptr<Heron::Motor::Motors> motors;
  if (!dryRun) {
    motors = std::make_unique<Heron::Motor::Motors>(Heron::Motor::toMotorsConfig(profile));
    std::printf("Starting the motors (resets the Arduino)...\n");
    if (!motors->start()) {
      std::fprintf(stderr, "ERROR: motors: %s\n", motors->getHealth().lastError.c_str());
      return 1;
    }
  }
  std::printf("Running. Check the mode, then SwA down to arm and throttle to the bottom. "
              "Ctrl-C to end.\n\n");

  const Heron::Drive::StickConfig sticks;
  Heron::Drive::ArmGate gate;
  const Heron::Profile::GeometryProfile geometry =
      profile.geometry.value_or(Heron::Profile::GeometryProfile{});
  Mode shownMode = Mode::Manual;
  Why shownWhy = Why::Disarmed;
  bool first = true;
  auto next = steady_clock::now();
  auto nextLine = next;
  std::uint64_t cycles = 0;
  std::uint64_t driving = 0;
  int status = 0;

  try {
    while (s_stopRequested == 0) {
      if (motors && motors->getHealth().state == Heron::Motor::MotorsState::Failed) {
        std::fprintf(stderr, "ERROR: motors failed: %s\n", motors->getHealth().lastError.c_str());
        status = 1;
        break;
      }
      const auto now = steady_clock::now();
      const Heron::Remote::RemoteSnapshot r = remote.getLatest(now);
      const Heron::Remote::Switch3 swc =
          Heron::Remote::decodeSwitch3(r.frame.channels[Heron::Robot::kModeChannel]);
      const Mode mode = Heron::Robot::modeFromSwitch(swc, manualIsHigh);
      const double throttle = Heron::Drive::readThrottle(r.frame, sticks);
      const bool armed = gate.update(r.isOk(), throttle);
      const Heron::Drive::BodyVelocity stick = Heron::Drive::sticksToVelocity(r.frame, sticks);

      // Assisted's inputs, read only in Assisted: Manual reads nothing new.
      std::shared_ptr<const Heron::Scan> scan;
      Heron::Pose::PoseHistory history;
      Heron::Robot::AssistedInput assisted;
      const bool ready = limiter != nullptr;
      if (mode == Mode::Assisted && ready) {
        scan = lidar->getLatest();
        history = pose->copyHistory().first;
        const Heron::Pose::PoseSnapshot p = pose->getLatest();
        assisted = Heron::Robot::AssistedInput{
            limiter.get(), geometry, motion ? &*motion : nullptr,
            Heron::Limiter::LimitInput{{}, scan.get(), &history, p.fresh, p.newest.velocity, now}};
      }
      const Heron::Robot::Decision d =
          Heron::Robot::decide(mode, armed, stick, ready ? &assisted : nullptr);

      if (armed && (d.why == Why::Manual || d.why == Why::Limited)) {
        ++driving;
        if (motors) {
          (void)motors->drive(d.wheels);
        }
      } else if (motors) {
        (void)motors->stopMotors();
      }
      ++cycles;

      if (first || mode != shownMode || d.why != shownWhy || now >= nextLine) {
        if (first || mode != shownMode) {
          std::printf("-- mode %s (SwC %s)\n", Heron::Robot::toString(mode), Heron::Remote::toString(swc));
        }
        if (first || d.why != shownWhy) {
          std::printf("-- %s%s%s\n", Heron::Robot::toString(d.why),
                      d.why == Why::AssistedUnavailable ? ": " : "",
                      d.why == Why::AssistedUnavailable ? unavailable.c_str() : "");
        }
        first = false;
        shownMode = mode;
        shownWhy = d.why;
        nextLine = now + microseconds(1000000 / linesPerSecond);

        std::string limitText;
        if (d.limited) {
          char text[96];
          const auto &l = *d.limited;
          if (l.nearest) {
            std::snprintf(text, sizeof(text), "  limiter %s, nearest %+.2f %+.2f m",
                          Heron::Limiter::toString(l.reason), l.nearest->x, l.nearest->y);
          } else {
            std::snprintf(text, sizeof(text), "  limiter %s", Heron::Limiter::toString(l.reason));
          }
          limitText = text;
          if (scan) {
            limitText += "  scan " + std::to_string(duration_cast<milliseconds>(now - scan->tEnd).count()) + " ms";
          }
        }
        // The measured speed, robot frame, read for this line only: it never
        // reaches decide(), so Manual still reads nothing new. Against
        // "out" x max_speed it shows whether the robot does what it is told.
        std::string speedText;
        if (pose) {
          const Heron::Pose::PoseSnapshot p = pose->getLatest();
          char text[128];
          std::snprintf(text, sizeof(text), "  speed %+5.2f %+5.2f m/s %+5.2f rad/s  heading %+6.1f deg%s%s%s",
                        p.newest.velocity.vx, p.newest.velocity.vy, p.newest.velocity.wz,
                        p.newest.pose.heading / Heron::Geometry::kDegToRad,
                        p.fresh ? "" : " (not fresh)", p.trackingWarning ? " TRACKING" : "",
                        p.tiltWarning ? " TILT" : "");
          speedText = text;
        }
        std::string motorText = "dry run";
        if (motors) {
          const Heron::Motor::MotorsSnapshot m = motors->getLatest(now);
          motorText = std::string(m.isOk() ? "motors OK" : "motors NOT OK") + "  resets " +
                      std::to_string(motors->getHealth().resets);
        }
        std::printf("%-8s %-8s throttle %4.2f  stick %+5.2f %+5.2f %+5.2f  out %+5.2f %+5.2f %+5.2f  "
                    "wheels %+5.2f %+5.2f %+5.2f %+5.2f%s%s  %s\n",
                    Heron::Robot::toString(mode), armed ? "ARMED" : r.isOk() ? "WAIT" : "DISARMED",
                    throttle, stick.vx, stick.vy, stick.wz, d.body.vx, d.body.vy, d.body.wz,
                    d.wheels[0], d.wheels[1], d.wheels[2], d.wheels[3], speedText.c_str(),
                    limitText.c_str(), motorText.c_str());
      }

      next += kPeriod;
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    status = 1;
  }

  if (motors) {
    motors->stop();  // sends Stop, closes the port
  }
  if (lidar) {
    lidar->stop();
  }
  if (pose) {
    pose->stop();
  }
  if (odometry) {
    odometry->stop();
  }
  remote.stop();
  std::printf("\nStopped. %" PRIu64 " cycles, %" PRIu64 " driving.\n", cycles, driving);
  return status;
}
