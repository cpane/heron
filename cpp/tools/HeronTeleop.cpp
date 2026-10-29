// File        : HeronTeleop.cpp
// Author      : Chris Pane
// Description : heron_teleop: drive Heron with the remote control. The
//               remote, the mixing and the motors, in one 50 Hz loop.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The first program that drives the robot: in effect the first, deliberately
// small, version of the robot core. Every 20 ms it reads the remote
// (RemoteControl), turns the sticks into four wheel values (heron_drive) and
// sends them (Motors). It adds nothing of its own beyond the loop and a
// status line: the rules are in the libraries.
//
//     scripts/deploy_cpp.sh <user>@<pi> heron_teleop
//     scripts/deploy_cpp.sh <user>@<pi> heron_teleop --dry-run
//
// Driving
// -------
//   1. Transmitter on. SwA (CH7) down arms; up disarms.
//   2. Once armed, bring the throttle (left stick up-down) to the bottom:
//      until then nothing moves, so arming with the throttle up cannot lurch
//      the robot.
//   3. The throttle sets the speed limit. Right stick: forward and back,
//      strafe left and right. Left stick left and right: turn.
//   Disarming, switching the transmitter off (the receiver fails SwA to
//   disarmed) or losing the receiver's link stops the motors at once, and
//   arming again needs the throttle at the bottom again.
//
// --dry-run reads the remote and prints what it would send, without opening
// the motors: for checking the sticks with nothing turning.
//
// Ctrl-C ends it; the motors are released on the way out.
//
// At boot
// -------
// target/systemd/heron-teleop@.service runs it at every boot, for driving
// with no laptop; scripts/teleop_autostart.sh turns that on and off. Its
// output then goes to the journal (journalctl -u heron-teleop@<user> -f).

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include "heron/drive/Drive.h"
#include "heron/motor/Motors.h"
#include "heron/profile/RigProfile.h"
#include "heron/remote/RemoteControl.h"

namespace {

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

constexpr std::chrono::milliseconds kPeriod(20);  // 50 Hz, the protocol's rate

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: heron_teleop [--dry-run] [--profile FILE] [--remote DEVICE]\n"
               "  --dry-run        read the remote, print the wheels, drive nothing\n"
               "  --profile FILE   the rig profile (default this Pi's)\n"
               "  --remote DEVICE  the iBUS receiver's port (default /dev/serial0)\n");
  std::exit(status);
}

// What the loop is doing, for the status line.
enum class Mode : std::uint8_t { Disarmed, WaitThrottle, Driving };

const char *toText(Mode mode) {
  switch (mode) {
    case Mode::Disarmed:
      return "DISARMED";
    case Mode::WaitThrottle:
      return "ARMED: bring the throttle down";
    case Mode::Driving:
      return "DRIVING";
  }
  return "?";
}

}  // namespace

int main(int argc, char **argv) {
  using namespace std::chrono;
  using Heron::Drive::ArmGate;
  using Heron::Drive::StickConfig;

  bool dryRun = false;
  std::string profilePath;
  Heron::Remote::ReceiverConfig remoteConfig;  // /dev/serial0, stale after 100 ms
  // SwA on CH7: 1000 up (disarmed) and the transmitter's failsafe value, so
  // disarming and radio loss read the same (docs/remote-ibus-protocol.md §5).
  remoteConfig.failsafe = Heron::Remote::FailsafeConfig{6, 1000, 50};

  for (int i = 1; i < argc; ++i) {  // int: argc's type
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage(0);
    } else if (arg == "--dry-run") {
      dryRun = true;
    } else if ((arg == "--profile" || arg == "--remote") && i + 1 < argc) {
      (arg == "--profile" ? profilePath : remoteConfig.serial.device) = argv[++i];
    } else {
      std::fprintf(stderr, "ERROR: unexpected argument '%s'\n", arg.c_str());
      usage(2);
    }
  }
  // Line by line even when stdout is not a terminal, so the journal shows
  // the status as it happens when this runs as a service.
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" heron_teleop%s\n", dryRun ? " (dry run: no motors)" : "");
  std::printf("========================================\n\n");

  Heron::Remote::RemoteControl remote(remoteConfig);
  if (!remote.start()) {
    std::fprintf(stderr, "ERROR: remote: %s\n", remote.getHealth().lastError.c_str());
    return 1;
  }

  std::unique_ptr<Heron::Motor::Motors> motors;
  if (!dryRun) {
    try {
      const auto profile = Heron::Profile::loadRigProfile(
          profilePath.empty() ? Heron::Profile::findRigProfile()
                              : std::filesystem::path(profilePath));
      std::printf("Profile: %s (rig %s)\n", profile.source.c_str(), profile.name.c_str());
      motors = std::make_unique<Heron::Motor::Motors>(Heron::Motor::toMotorsConfig(profile));
    } catch (const std::exception &e) {
      std::fprintf(stderr, "ERROR: %s\n", e.what());
      return 1;
    }
    std::printf("Starting the motors (resets the Arduino)...\n");
    if (!motors->start()) {
      std::fprintf(stderr, "ERROR: motors: %s\n", motors->getHealth().lastError.c_str());
      return 1;
    }
  }
  std::printf("Running. SwA down to arm, then throttle to the bottom. Ctrl-C to end.\n\n");

  const StickConfig sticks;
  ArmGate gate;
  Mode shown = Mode::Disarmed;
  auto next = steady_clock::now();
  auto nextLine = next;
  std::uint64_t cycles = 0;
  std::uint64_t driving = 0;

  try {
    while (s_stopRequested == 0) {
      const auto now = steady_clock::now();
      const Heron::Remote::RemoteSnapshot r = remote.getLatest(now);
      const double throttle = Heron::Drive::readThrottle(r.frame, sticks);
      const bool allowed = gate.update(r.isOk(), throttle);
      const Mode mode = allowed ? Mode::Driving : r.isOk() ? Mode::WaitThrottle : Mode::Disarmed;

      Heron::Drive::BodyVelocity v{};
      Heron::Drive::WheelValues wheels{};
      if (allowed) {
        v = Heron::Drive::sticksToVelocity(r.frame, sticks);
        wheels = Heron::Drive::mix(v);
        ++driving;
        if (motors) {
          (void)motors->drive(wheels);
        }
      } else if (motors) {
        (void)motors->stopMotors();
      }
      ++cycles;

      if (mode != shown || now >= nextLine) {
        if (mode != shown) {
          std::printf("-- %s\n", toText(mode));
          shown = mode;
        }
        nextLine = now + seconds(1);
        std::string motorText = "dry run";
        if (motors) {
          const Heron::Motor::MotorsSnapshot m = motors->getLatest(now);
          motorText = std::string(m.isOk() ? "motors OK" : "motors NOT OK") + "  resets " +
                      std::to_string(motors->getHealth().resets);
        }
        std::printf("%-8s throttle %4.2f  vx %+5.2f vy %+5.2f wz %+5.2f  "
                    "wheels %+5.2f %+5.2f %+5.2f %+5.2f  %s\n",
                    mode == Mode::Driving ? "DRIVING" : mode == Mode::WaitThrottle ? "ARMED" : "DISARMED",
                    throttle, v.vx, v.vy, v.wz, wheels[0], wheels[1], wheels[2], wheels[3],
                    motorText.c_str());
      }

      next += kPeriod;
      std::this_thread::sleep_until(next);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
  }

  if (motors) {
    motors->stop();  // sends Stop, closes the port
  }
  remote.stop();
  std::printf("\nStopped. %" PRIu64 " cycles, %" PRIu64 " driving.\n", cycles, driving);
  return 0;
}
