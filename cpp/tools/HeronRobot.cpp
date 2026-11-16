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
//             along a wall and backing away still work. With the turn
//             stick centred, heading hold keeps the heading while it
//             moves (P14). Told to move while the OTOS sees nothing for
//             half a second (lifted, or stuck), it stops (NOT_MOVING)
//             until the stick is centred. Once the limiter cuts, the
//             command only falls until it has been clear for 0.1 s
//             (CutHold), so a noisy speed cannot pulse the wheels.
//   Navigate  hold the right stick in a direction and Heron keeps to that
//             line, sliding round what is in the way and coming back to
//             it (CourseKeeper, the map, the planner; design §8). The
//             plan's command goes through everything Assisted has: the
//             limiter, heading hold, NOT_MOVING, CutHold. Letting go stops
//             at once; the left stick turns as in Manual. BLOCKED on the
//             status line: the limiter has let none of the plan through
//             for over 0.5 s.
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
//   the OTOS's TRACKING and TILT warnings when set, while heading hold
//   holds the heading error it is correcting ("hold", degrees, held minus
//   now), and in Assisted and Navigate the limiter's reason, the nearest
//   point and the scan's age. In Navigate also the plan: its state
//   (FOLLOWING, AVOIDING, BLOCKED, STOPPED) and reason, the clearance its
//   path keeps, and BLOCKED when the limiter holds it back.
//   "out" x max_speed is the speed asked for; "speed" is what it did.
//
// Start-up
// --------
//   1. The profile: Assisted needs [geometry], [lidar], [odometry] and
//      [navigation], and the LiDAR's blind circle inside the stop outline.
//   2. The remote.
//   3. The OTOS, first among the sensors: its gyro calibrates at start, so
//      keep the robot still (design §9). Then the pose tracker.
//   4. The LiDAR, then the map (Perception) and the planner, each a thread.
//   5. The motors (resets the Arduino), unless --dry-run.
// Each sensor's start is tried up to three times, 2 s apart (both fail now
// and then and pass on the next start). One that still fails is reported and leaves Assisted and Navigate
// unavailable: Manual still works, and the other two stop rather than
// falling back to Manual.
// An OTOS whose pose stays not fresh for 2 s (stuck after a lift), with the
// sticks centred, is restarted in the background; "-- OTOS restart" lines
// say so. A LiDAR that stops while running leaves its scans going stale, and the
// limiter stops Assisted (Stale, design §7.4).
//
// --dry-run starts everything but the motors and prints what it would
// send: for checking the sticks, SwC and the limiter with nothing turning.
// "Commanded but not moving" is off in it, since nothing moves.
// Ctrl-C ends it; the motors are released on the way out.

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "heron/drive/Drive.h"
#include "heron/lidar/Lidar.h"
#include "heron/limiter/Limiter.h"
#include "heron/motor/Motors.h"
#include "heron/odometry/Odometry.h"
#include "heron/perception/Perception.h"
#include "heron/planner/LocalPlanner.h"
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

// Starting a sensor is tried this many times, this far apart. Both fail now
// and then and pass on the next start (2026-11-16, in their workstreams'
// Inboxes): the OTOS's scalars read back wrong shortly after the Pi powers
// on; the LiDAR refuses its scan mode. The wait gives the LiDAR's motor,
// stopped by the failed start, time to spin up again (a guess, unchecked).
constexpr int kStartAttempts = 3;
constexpr std::chrono::seconds kStartRetryWait(2);

/** Makes a sensor and starts it, up to kStartAttempts times, a fresh
 *  instance each time (restarting one is not yet proven for the LiDAR:
 *  RESUME-lidar.md §9), waiting between; says so on each failure.
 *  @return the running sensor; on failure the last one, stopped, whose
 *          getHealth().lastError says why */
template <typename Sensor, typename Make>
std::unique_ptr<Sensor> startWithRetries(Make make, const char *name, bool &started) {
  std::unique_ptr<Sensor> sensor;
  started = false;
  for (int attempt = 1; attempt <= kStartAttempts && s_stopRequested == 0; ++attempt) {
    sensor = make();
    if (sensor->start()) {
      started = true;
      return sensor;
    }
    std::printf("  %s did not start (attempt %d of %d): %s\n", name, attempt, kStartAttempts,
                sensor->getHealth().lastError.c_str());
    sensor->stop();
    if (attempt < kStartAttempts) {
      std::this_thread::sleep_for(kStartRetryWait);
    }
  }
  return sensor;
}

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: heron_robot [--dry-run] [--manual-is-high] [--lines N] [--profile FILE]\n"
               "                   [--surface NAME] [--remote DEVICE] [--test-otos-restart]\n"
               "  --dry-run         everything but the motors; print the wheels\n"
               "  --manual-is-high  SwC High is Manual (default: Low is Manual)\n"
               "  --lines N         status lines a second, 1 to 50 (default 1)\n"
               "  --profile FILE    the rig profile (default this Pi's)\n"
               "  --surface NAME    the floor (default the profile's [rig] surface)\n"
               "  --remote DEVICE   the iBUS receiver's port (default /dev/serial0)\n"
               "  --test-otos-restart  restart the OTOS once, 5 s in, as a stuck one is\n");
  std::exit(status);
}

}  // namespace

int main(int argc, char **argv) {
  using namespace std::chrono;
  using Heron::Robot::Mode;
  using Heron::Robot::Why;

  bool dryRun = false;
  bool manualIsHigh = false;
  bool testOtosRestart = false;
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
    } else if (arg == "--test-otos-restart") {
      testOtosRestart = true;
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
    bool started = false;
    odometry = startWithRetries<Heron::Odometry::Odometry>(
        [&profile] {
          return std::make_unique<Heron::Odometry::Odometry>(Heron::Robot::toOtosConfig(*profile.odometry));
        },
        "the OTOS", started);
    if (!started) {
      unavailable = "OTOS: " + (odometry ? odometry->getHealth().lastError : std::string("stopped"));
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
    bool started = false;
    lidar = startWithRetries<Heron::Lidar::Lidar>(
        [&profile] { return std::make_unique<Heron::Lidar::Lidar>(Heron::Robot::toLidarConfig(*profile.lidar)); },
        "the LiDAR", started);
    if (!started) {
      unavailable = "LiDAR: " + (lidar ? lidar->getHealth().lastError : std::string("stopped"));
    }
  }

  std::unique_ptr<Heron::Limiter::Limiter> limiter;
  std::shared_ptr<const Heron::Motion::MotionTable> motion;
  std::unique_ptr<Heron::Perception::Perception> perception;
  std::unique_ptr<Heron::Planner::LocalPlanner> planner;
  if (unavailable.empty()) {
    limiter = std::make_unique<Heron::Limiter::Limiter>(Heron::Robot::toLimiterConfig(profile));
    if (auto table = Heron::Robot::toMotionTable(profile)) {  // checked by assistedRefusal()
      motion = std::make_shared<const Heron::Motion::MotionTable>(std::move(*table));
    }
    std::printf("Assisted: ready, %s\n",
                motion ? "through the surface's motion table" : "with the min_speed model (no table)");
    // Navigate's map and planner, each its own thread (design §5).
    perception = std::make_unique<Heron::Perception::Perception>(*lidar, *pose,
                                                                 Heron::Robot::toPerceptionConfig(*profile.lidar));
    Heron::Planner::LocalPlannerConfig plannerConfig;
    plannerConfig.planner = Heron::Robot::toPlannerConfig(profile, motion);
    planner = std::make_unique<Heron::Planner::LocalPlanner>(*perception, *pose, plannerConfig);
    (void)perception->start();
    (void)planner->start();
    std::printf("Navigate: ready\n");
  } else {
    std::printf("Assisted and Navigate: UNAVAILABLE (%s). Manual works; the others stop.\n",
                unavailable.c_str());
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
  Heron::Robot::HeadingHold hold;
  Heron::Robot::MotionWatch watch;
  Heron::Robot::CutHold cutHold;
  Heron::Planner::CourseKeeper keeper;
  Heron::Robot::BlockedWatch blockedWatch;
  Heron::Robot::OtosRecovery otosRecovery;
  Heron::Robot::LidarStill lidarStill;  // for OtosRecovery: still by the LiDAR
  std::future<bool> otosRestart;  // valid while a restart runs, off the loop
  bool coursing = false;                // a course was last given to the planner
  Mode lastMode = Mode::Manual;         // last cycle's, to start afresh on a change
  Heron::Drive::BodyVelocity lastBody;  // what the wheels were told last cycle
  Mode shownMode = Mode::Manual;
  Why shownWhy = Why::Disarmed;
  bool first = true;
  auto next = steady_clock::now();
  // --test-otos-restart: once, through the same path as a stuck OTOS.
  std::optional<steady_clock::time_point> forcedRestartAt;
  if (testOtosRestart) {
    forcedRestartAt = next + seconds(5);
  }
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

      // Assisted's and Navigate's inputs, read only there: Manual reads
      // nothing new.
      std::shared_ptr<const Heron::Scan> scan;
      Heron::Pose::PoseHistory history;
      Heron::Robot::AssistedInput assisted;
      const bool ready = limiter != nullptr;
      const bool sensing = (mode == Mode::Assisted || mode == Mode::Navigate) && ready;
      // Navigate: the plan's command stands in for the stick from here on.
      Heron::Drive::BodyVelocity drive = stick;
      std::shared_ptr<const Heron::Planner::Plan> plan;
      std::optional<Heron::Planner::Course> course;
      if (mode != lastMode || !armed) {
        hold.reset();  // a mode's heading, ceiling and course are its own
        watch.reset();
        cutHold.reset();
        keeper.reset();
        blockedWatch.reset();
      }
      lastMode = mode;
      if (sensing) {
        scan = lidar->getLatest();
        history = pose->copyHistory().first;
        const Heron::Pose::PoseSnapshot p = pose->getLatest();
        if (mode == Mode::Navigate && armed) {
          course = keeper.update(Heron::Robot::toTwist(stick, geometry), p.newest.pose, p.epoch);
          if (course && hold.holding()) {  // last cycle's: the turn the limiter is told (Planner.h)
            course->holdHeadingRad = hold.targetRad();
          }
          planner->setCourse(course);
          coursing = true;
          plan = planner->getLatest();
          drive = Heron::Robot::navigateCommand(course, plan.get(), stick, geometry, motion != nullptr, now);
        }
        assisted = Heron::Robot::AssistedInput{
            limiter.get(), geometry, motion.get(),
            Heron::Limiter::LimitInput{{}, scan.get(), &history, p.fresh, p.newest.velocity, now}, std::nullopt, false,
            std::nullopt};
        // Heading hold (P14): with the turn stick centred and a translation
        // asked for, its rate stands in for the stick's turn.
        const bool translating = drive.vx != 0.0 || drive.vy != 0.0;
        const double stickTurn = Heron::Robot::toTwist(Heron::Drive::BodyVelocity{0.0, 0.0, drive.wz}, geometry).wz;
        const double drift = motion && translating ? motion->predict(drive.vx, drive.vy).wz : 0.0;
        const double rate = hold.update(stickTurn, translating, p.newest.pose.heading, drift, p.epoch);
        if (armed && hold.holding()) {
          assisted.holdRadps = rate;
        }
        // Commanded but not moving: last cycle's command, from rest,
        // against what the OTOS reads now (design §19.4 step 4). Off in a
        // dry run, where nothing is driven, so nothing moves.
        const Heron::Geometry::Twist expected = motion && !dryRun
                                                    ? motion->predictFromRest(lastBody.vx, lastBody.vy)
                                                    : Heron::Geometry::Twist{};
        assisted.notMoving = watch.update(std::hypot(expected.vx, expected.vy),
                                          std::hypot(p.newest.velocity.vx, p.newest.velocity.vy),
                                          translating, now);
        // While the limiter cuts, the command only falls (CutHold).
        assisted.cutCeiling = cutHold.ceiling(drive);
      }
      // The OTOS stuck after a lift (OtosRecovery): restarted off the loop,
      // so the heartbeat never pauses; Manual drives on meanwhile, and
      // Assisted and Navigate stay stopped until the pose is fresh again.
      if (pose) {
        if (otosRestart.valid() &&
            otosRestart.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          const bool ok = otosRestart.get();
          std::printf("-- OTOS restart %s%s\n", ok ? "done" : "FAILED: ",
                      ok ? "" : odometry->getHealth().lastError.c_str());
        }
        const Heron::Pose::PoseSnapshot p = pose->getLatest();
        Heron::Robot::OtosRecovery::Input in;
        in.poseFresh = p.fresh;
        in.tilt = p.tiltWarning;
        in.turnRadps = p.newest.velocity.wz;
        in.sticksCentred = stick.vx == 0.0 && stick.vy == 0.0 && stick.wz == 0.0;
        // No LiDAR: centred sticks stand in for still, as before.
        in.still = lidar ? lidarStill.update(lidar->getLatest()) : true;
        const bool forced = forcedRestartAt && now >= *forcedRestartAt;
        if ((otosRecovery.update(in, now) || forced) && !otosRestart.valid()) {
          forcedRestartAt.reset();
          std::printf("-- OTOS: %s, restarting it (keep the robot still)\n",
                      forced ? "--test-otos-restart" : otosRecovery.why());
          Heron::Odometry::Odometry *o = odometry.get();
          otosRestart = std::async(std::launch::async, [o] {
            o->stop();
            return o->start();
          });
        }
      }
      if (coursing && !(mode == Mode::Navigate && armed && sensing)) {
        planner->setCourse(std::nullopt);  // the planner idles
        coursing = false;
      }
      if (!armed || !sensing) {
        hold.reset();
        watch.reset();
        cutHold.reset();
      }
      const Heron::Robot::Decision d =
          Heron::Robot::decide(mode, armed, drive, ready ? &assisted : nullptr);
      cutHold.update(drive, d);
      // BLOCKED is the limiter's doing only: stopped for any other reason
      // (NOT_MOVING, the pose) the status already says why.
      const bool blocked =
          mode == Mode::Navigate &&
          blockedWatch.update(d.why == Why::Limited && (drive.vx != 0.0 || drive.vy != 0.0),
                              d.body.vx != 0.0 || d.body.vy != 0.0, now);

      lastBody = armed && d.why == Why::Limited ? d.body : Heron::Drive::BodyVelocity{};
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
        if (mode == Mode::Navigate && plan) {
          char text[160];
          std::snprintf(text, sizeof(text), "  plan %s%s%s clear %.2f m  %lld ms old%s  moved %.2f m/s%s",
                        Heron::Planner::toString(plan->state),
                        plan->reason == Heron::Planner::Reason::None ? "" : " ",
                        plan->reason == Heron::Planner::Reason::None ? "" : Heron::Planner::toString(plan->reason),
                        plan->clearanceM, static_cast<long long>(duration_cast<milliseconds>(now - plan->time).count()),
                        course && plan->courseSeq != course->seq ? " (older course)" : "", plan->movedMps,
                        plan->nearlyStill ? " STILL" : "");
          limitText += text;
        }
        if (blocked) {
          limitText += "  BLOCKED";
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
          if (lidar && !lidarStill.still()) {  // OtosRecovery waits for still
            speedText += "  view moving";
          }
          if (p.epoch > 1) {  // the OTOS restarted: a new odom frame (I8)
            speedText += "  epoch " + std::to_string(p.epoch);
          }
          if (hold.holding()) {
            std::snprintf(text, sizeof(text), "  hold %+5.1f deg",
                          Heron::Geometry::wrapAngle(hold.targetRad() - p.newest.pose.heading) /
                              Heron::Geometry::kDegToRad);
            speedText += text;
          }
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
  if (otosRestart.valid()) {
    (void)otosRestart.get();  // the restart owns the OTOS until it ends
  }
  if (planner) {
    const Heron::Planner::PlannerHealth h = planner->getHealth();
    planner->stop();
    std::printf("Planner: %" PRIu64 " plans, %" PRIu64 " blocked, longest %.1f ms\n", h.plans, h.blocked,
                static_cast<double>(duration_cast<microseconds>(h.longestCycle).count()) / 1000.0);
  }
  if (perception) {
    const Heron::Perception::PerceptionHealth h = perception->getHealth();
    perception->stop();
    std::printf("Map: %" PRIu64 " scans added, %" PRIu64 " skipped, %" PRIu64 " resets\n", h.scansAdded,
                h.scansSkipped, h.mapResets);
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
