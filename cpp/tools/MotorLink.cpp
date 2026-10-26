// File        : MotorLink.cpp
// Author      : Chris Pane
// Description : Bench tool for the Arduino motor link: sends commands, prints
//               what the firmware reports, and runs the bench checks.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The tool for talking to the motor firmware by hand, before the robot's
// own motor library (heron_motor, step 4) exists, and for debugging the link
// after. It speaks the real protocol through the same shared code as the
// firmware (protocol/motor/), so what it shows is what the robot will see.
//
//     scripts/deploy_cpp.sh <user>@<pi> motor_link info
//     scripts/deploy_cpp.sh <user>@<pi> motor_link drive 100 0 0 0 --seconds 2
//     scripts/deploy_cpp.sh <user>@<pi> motor_link sweep 1 --step 20
//     build-host/bin/motor_link watch --device /dev/cu.usbserial-0001   (an Uno on a Mac)
//
// Commands, each one of the bench checks in docs/RESUME-motor.md:
//
//   info       Boot: how long after opening the port it arrives (the port
//              resets the Uno), then again on an InfoRequest
//   watch      print every Status and Boot; sends nothing
//   drive      send Drive at 50 Hz for --seconds, printing Status, then Stop
//   watchdog   Drive for 1 s (at 0, or the wheels given), then go silent:
//              when does the firmware stop?
//   noise      Drive while injecting junk and corrupted frames: do the
//              counters see them, and does the link carry on?
//   sweep      one motor, stepped from low to full, forward then backward,
//              holding each value: where does it start to turn? That is
//              the platform's dead band (firmware/motor/src/PlatformConfig.h)
//
// Every command except watch first waits for the Boot that opening the port
// causes, so its first frames are not lost while the Uno restarts.
//
// How it is built
// ---------------
// MotorPort moves bytes; FrameParser turns them into frames; the decode*()
// functions in Protocol.h turn frames into messages. Link below ties the
// three together with a clock, and each command is a function that sends on
// a schedule and prints what Link receives.
//
// A bench tool, so it prints. From step 3 the firmware drives the motors:
// drive and sweep turn wheels, so the robot must be on the bench, wheels
// off the ground. Ctrl-C sends Stop.

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>
#include <system_error>

#include "heron/motor/FrameParser.h"
#include "heron/motor/MotorPort.h"
#include "heron/motor/Protocol.h"

namespace {

using namespace Heron::Motor;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

volatile std::sig_atomic_t s_stopRequested = 0;

void onSignal(int) {
  s_stopRequested = 1;
}

constexpr milliseconds kDrivePeriod(20);       // 50 Hz, as the robot will send
constexpr milliseconds kBootWait(3000);        // the Uno's restart after opening
constexpr milliseconds kReplyWait(1000);

// ---------------------------------------------------------------------------
// Options

struct Options {
  std::string command;
  MotorPortConfig port;
  double seconds{3.0};
  bool secondsGiven{false};
  std::int16_t wheels[kMotorCount]{};
  std::size_t sweepMotor{0};      // 1..4; 0 until given
  std::int16_t sweepStep{50};
};

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: motor_link <command> [options]\n"
               "\n"
               "Commands:\n"
               "  info                 when Boot arrives after opening, then Boot on request\n"
               "  watch                print every Status and Boot (until Ctrl-C or --seconds)\n"
               "  drive M1 M2 M3 M4    Drive at 50 Hz for --seconds (default 3), then Stop.\n"
               "                       Wheels -1000..1000: front-left, front-right,\n"
               "                       back-right, back-left; positive drives forward\n"
               "  watchdog [M1..M4]    Drive 1 s (default all 0), then stop sending:\n"
               "                       when does it stop?\n"
               "  noise                Drive while injecting junk and corrupted frames\n"
               "  sweep MOTOR          one motor (1..4) from --step to 1000, then backward,\n"
               "                       holding each value --seconds (default 1)\n"
               "\n"
               "Options:\n"
               "  --device PATH        serial device (default /dev/arduino)\n"
               "  --seconds S          how long drive and watch run; sweep: per value\n"
               "  --step N             sweep: the value step, 1..1000 (default 50)\n"
               "  -h, --help           this text\n");
  std::exit(status);
}

std::int16_t toWheel(const std::string &text) {
  try {
    std::size_t used = 0;
    const long value = std::stol(text, &used);
    if (used == text.size() && value >= kWheelMin && value <= kWheelMax) {
      return static_cast<std::int16_t>(value);
    }
  } catch (const std::exception &) {
  }
  std::fprintf(stderr, "ERROR: a wheel value must be a whole number -1000..1000, got '%s'\n",
               text.c_str());
  usage(2);
}

Options parse(int argc, char **argv) {
  Options options;
  std::size_t wheelsGiven = 0;
  for (int i = 1; i < argc; ++i) {  // int: matches argc
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      usage(0);
    } else if (arg == "--device" || arg == "--seconds" || arg == "--step") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "ERROR: %s needs a value\n", arg.c_str());
        usage(2);
      }
      const std::string value = argv[++i];
      if (arg == "--device") {
        options.port.device = value;
      } else if (arg == "--step") {
        options.sweepStep = toWheel(value);
        if (options.sweepStep < 1) {
          std::fprintf(stderr, "ERROR: --step must be 1..1000\n");
          usage(2);
        }
      } else {
        try {
          options.seconds = std::stod(value);
        } catch (const std::exception &) {
          options.seconds = -1.0;
        }
        options.secondsGiven = true;
        if (options.seconds <= 0.0) {
          std::fprintf(stderr, "ERROR: --seconds must be a positive number\n");
          usage(2);
        }
      }
    } else if (options.command.empty() && !arg.empty() && arg[0] != '-') {
      options.command = arg;
    } else if ((options.command == "drive" || options.command == "watchdog") &&
               wheelsGiven < kMotorCount) {
      options.wheels[wheelsGiven++] = toWheel(arg);  // negative values start with '-'
    } else if (options.command == "sweep" && options.sweepMotor == 0) {
      const std::int16_t motor = toWheel(arg);
      if (motor < 1 || motor > static_cast<std::int16_t>(kMotorCount)) {
        std::fprintf(stderr, "ERROR: sweep needs a motor 1..4, got '%s'\n", arg.c_str());
        usage(2);
      }
      options.sweepMotor = static_cast<std::size_t>(motor);
    } else {
      std::fprintf(stderr, "ERROR: unexpected argument '%s'\n", arg.c_str());
      usage(2);
    }
  }
  if (options.command.empty()) {
    usage(2);
  }
  if ((options.command == "drive" && wheelsGiven != kMotorCount) ||
      (options.command == "watchdog" && wheelsGiven != 0 && wheelsGiven != kMotorCount)) {
    std::fprintf(stderr, "ERROR: %s needs four wheel values\n", options.command.c_str());
    usage(2);
  }
  if (options.command == "sweep" && options.sweepMotor == 0) {
    std::fprintf(stderr, "ERROR: sweep needs a motor, 1..4\n");
    usage(2);
  }
  if (options.command == "sweep" && !options.secondsGiven) {
    options.seconds = 1.0;
  }
  return options;
}

// ---------------------------------------------------------------------------
// Printing

const char *toText(MotorState state) {
  switch (state) {
    case MotorState::Stopped:
      return "stopped";
    case MotorState::Running:
      return "running";
  }
  return "?";
}

const char *toText(StopReason reason) {
  switch (reason) {
    case StopReason::None:
      return "-";
    case StopReason::Boot:
      return "boot";
    case StopReason::StopMessage:
      return "stop";
    case StopReason::Watchdog:
      return "watchdog";
  }
  return "?";
}

const char *toText(Platform platform) {
  switch (platform) {
    case Platform::Unknown:
      return "unknown";
    case Platform::Bench:
      return "bench";
    case Platform::Mini:
      return "mini";
    case Platform::FullSize:
      return "full-size";
  }
  return "?";
}

void printStatus(std::int64_t atMs, const Status &s) {
  std::printf("%7lld ms  seq %3u  %-7s %-8s  target %5d %5d %5d %5d  applied %5d %5d %5d %5d"
              "  wd %u  frames %u  crc %u  ver %u  disc %u  loop %u us\n",
              static_cast<long long>(atMs), s.lastSequence, toText(s.state),
              toText(s.stopReason), s.target[0], s.target[1], s.target[2], s.target[3],
              s.applied[0], s.applied[1], s.applied[2], s.applied[3], s.watchdogStops,
              s.framesReceived, s.checksumErrors, s.versionMismatches, s.bytesDiscarded,
              s.longestLoopMicros);
}

void printBoot(std::int64_t atMs, const BootInfo &b) {
  std::string reset;
  if ((b.resetFlags & kResetPowerOn) != 0) {
    reset += " power-on";
  }
  if ((b.resetFlags & kResetExternal) != 0) {
    reset += " external";
  }
  if ((b.resetFlags & kResetBrownOut) != 0) {
    reset += " brown-out";
  }
  if ((b.resetFlags & kResetWatchdog) != 0) {
    reset += " watchdog";
  }
  if (reset.empty()) {
    reset = " none reported";
  }
  std::printf("%7lld ms  BOOT  firmware %u.%u.%u  protocol %u  platform %s  reset:%s"
              "  uptime %lu ms  free RAM %u bytes\n",
              static_cast<long long>(atMs), b.firmwareMajor, b.firmwareMinor, b.firmwarePatch,
              b.protocolVersion, toText(b.platform), reset.c_str(),
              static_cast<unsigned long>(b.uptimeMillis), b.freeRamBytes);
}

// ---------------------------------------------------------------------------
// The link: port, parser and clock

class Link {
  public:
    explicit Link(const MotorPortConfig &config) : m_port(config), m_opened(Clock::now()) {}

    /** @return Milliseconds since the port was opened */
    [[nodiscard]] std::int64_t sinceOpen() const {
      return std::chrono::duration_cast<milliseconds>(Clock::now() - m_opened).count();
    }

    /**
     * Reads for up to timeout, decoding whatever arrives; prints it unless
     * quiet. The newest Status and Boot are kept.
     */
    void pump(milliseconds timeout, bool quiet = false) {
      std::uint8_t buf[256];
      const std::size_t n = m_port.read(buf, sizeof(buf), timeout);
      m_parser.feed(buf, n, [this, quiet](const Frame &frame) {
        Status status;
        BootInfo boot;
        if (decodeStatus(frame, status)) {
          m_status = status;
          ++m_statusCount;
          if (!quiet) {
            printStatus(sinceOpen(), status);
          }
        } else if (decodeBoot(frame, boot)) {
          m_boot = boot;
          m_bootAtMs = sinceOpen();
          if (!quiet) {
            printBoot(m_bootAtMs, boot);
          }
        } else {
          std::printf("%7lld ms  unexpected frame: type 0x%02X, %zu payload bytes\n",
                      static_cast<long long>(sinceOpen()), static_cast<unsigned>(frame.type),
                      frame.payloadLen);
        }
      });
    }

    /** Pumps until a Boot arrives or timeout passes. @return whether one did */
    bool waitForBoot(milliseconds timeout) {
      m_boot.reset();
      const auto until = Clock::now() + timeout;
      while (!m_boot && Clock::now() < until && s_stopRequested == 0) {
        pump(milliseconds(20));
      }
      return m_boot.has_value();
    }

    void sendDrive(const std::int16_t (&wheels)[kMotorCount]) {
      DriveCommand cmd;
      cmd.sequence = m_sequence++;
      for (std::size_t i = 0; i < kMotorCount; ++i) {
        cmd.wheels[i] = wheels[i];
      }
      std::uint8_t frame[kMaxFrameLen];
      m_port.write(frame, encodeDrive(cmd, frame, sizeof(frame)));
    }

    void sendStop() {
      std::uint8_t frame[kMaxFrameLen];
      m_port.write(frame, encodeStop(StopCommand{m_sequence++}, frame, sizeof(frame)));
    }

    void sendInfoRequest() {
      std::uint8_t frame[kMaxFrameLen];
      m_port.write(frame, encodeInfoRequest(InfoRequest{m_sequence++}, frame, sizeof(frame)));
    }

    void sendRaw(const std::uint8_t *bytes, std::size_t length) { m_port.write(bytes, length); }

    [[nodiscard]] std::uint8_t nextSequence() const { return m_sequence; }
    [[nodiscard]] const std::optional<Status> &lastStatus() const { return m_status; }
    [[nodiscard]] const std::optional<BootInfo> &lastBoot() const { return m_boot; }
    [[nodiscard]] std::int64_t bootAtMs() const { return m_bootAtMs; }
    [[nodiscard]] std::uint64_t statusCount() const { return m_statusCount; }

  private:
    MotorPort m_port;
    FrameParser m_parser;
    Clock::time_point m_opened;
    std::uint8_t m_sequence{0};
    std::optional<Status> m_status;
    std::optional<BootInfo> m_boot;
    std::int64_t m_bootAtMs{0};
    std::uint64_t m_statusCount{0};
};

/** Opening the port restarts the Uno; wait for it, and say how long it took. */
void waitForStartup(Link &link) {
  std::printf("Opened the port; waiting for the Boot it causes...\n");
  if (link.waitForBoot(kBootWait)) {
    std::printf("Boot arrived %lld ms after opening.\n\n", static_cast<long long>(link.bootAtMs()));
  } else {
    std::printf("No Boot within %lld ms: the board did not restart, or is not running the\n"
                "firmware. Carrying on.\n\n",
                static_cast<long long>(kBootWait.count()));
  }
}

/**
 * Sends Drive every 20 ms for duration, pumping between sends; stops early
 * on Ctrl-C. optional perTick runs before each send.
 */
template <typename PerTick>
void driveFor(Link &link, const std::int16_t (&wheels)[kMotorCount], Clock::duration duration,
              bool quiet, PerTick &&perTick) {
  const auto start = Clock::now();
  auto next = start;
  while (Clock::now() - start < duration && s_stopRequested == 0) {
    perTick();
    link.sendDrive(wheels);
    next += kDrivePeriod;
    while (Clock::now() < next && s_stopRequested == 0) {
      const auto left = std::chrono::duration_cast<milliseconds>(next - Clock::now());
      link.pump(left > milliseconds(0) ? left : milliseconds(0), quiet);
    }
  }
}

/** Sends Stop and prints the Status that follows, so the run ends stopped. */
void stopAndReport(Link &link) {
  link.sendStop();
  const auto until = Clock::now() + milliseconds(300);
  while (Clock::now() < until) {
    link.pump(milliseconds(20));
  }
}

// ---------------------------------------------------------------------------
// Commands

int runInfo(Link &link) {
  waitForStartup(link);
  std::printf("Sending InfoRequest (sequence %u)...\n", link.nextSequence());
  link.sendInfoRequest();
  if (!link.waitForBoot(kReplyWait)) {
    std::printf("No Boot within %lld ms of the request.\n",
                static_cast<long long>(kReplyWait.count()));
    return 1;
  }
  return 0;
}

int runWatch(Link &link, const Options &options) {
  std::printf("Watching (Ctrl-C to stop). Nothing is sent.\n");
  const auto until = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                        std::chrono::duration<double>(options.seconds));
  while (s_stopRequested == 0 && (!options.secondsGiven || Clock::now() < until)) {
    link.pump(milliseconds(100));
  }
  return 0;
}

int runDrive(Link &link, const Options &options) {
  waitForStartup(link);
  std::printf("Drive %d %d %d %d at 50 Hz for %.1f s, then Stop.\n", options.wheels[0],
              options.wheels[1], options.wheels[2], options.wheels[3], options.seconds);
  driveFor(link, options.wheels,
           std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(options.seconds)),
           false, [] {});
  std::printf("Stop.\n");
  stopAndReport(link);
  return 0;
}

int runWatchdog(Link &link, const Options &options) {
  waitForStartup(link);
  std::printf("Drive %d %d %d %d for 1 s...\n", options.wheels[0], options.wheels[1],
              options.wheels[2], options.wheels[3]);
  driveFor(link, options.wheels, std::chrono::seconds(1), false, [] {});
  const std::int64_t lastDriveMs = link.sinceOpen();
  std::printf("Silent from %lld ms. The firmware should stop about 250 ms later; Status\n"
              "comes every 100 ms, so it shows within 100 ms after that.\n",
              static_cast<long long>(lastDriveMs));
  const auto until = Clock::now() + std::chrono::seconds(1);
  std::optional<std::int64_t> seenAtMs;
  while (Clock::now() < until && s_stopRequested == 0) {
    link.pump(milliseconds(20));
    const auto &s = link.lastStatus();
    if (!seenAtMs && s && s->stopReason == StopReason::Watchdog) {
      seenAtMs = link.sinceOpen();
    }
  }
  if (!seenAtMs) {
    std::printf("\nFAIL: no watchdog stop reported within 1 s of going silent.\n");
    return 1;
  }
  std::printf("\nWatchdog stop first reported %lld ms after the last Drive (expect 250-350).\n",
              static_cast<long long>(*seenAtMs - lastDriveMs));
  return 0;
}

int runNoise(Link &link) {
  waitForStartup(link);
  // No 0xC5 among the junk, so each injection costs exactly what is counted
  // below; a corrupted frame can still hide a false start, hence "at least".
  const std::uint8_t junk[] = {0x00, 0x11, 0x22, 0x33, 0x44};
  const std::int16_t zero[kMotorCount] = {0, 0, 0, 0};

  link.pump(milliseconds(150), true);  // a fresh Status to start from
  const std::optional<Status> before = link.lastStatus();
  if (!before) {
    std::printf("FAIL: no Status before starting.\n");
    return 1;
  }

  std::printf("Drive 0 0 0 0 for 2 s; every 200 ms inject 5 junk bytes and one corrupted\n"
              "Drive frame (one bit flipped)...\n");
  std::uint32_t injections = 0;
  std::uint32_t tick = 0;
  driveFor(link, zero, std::chrono::seconds(2), true, [&] {
    if (tick++ % 10 != 0) {
      return;
    }
    DriveCommand cmd;
    cmd.sequence = 0x55;
    std::uint8_t frame[kMaxFrameLen];
    const std::size_t length = encodeDrive(cmd, frame, sizeof(frame));
    frame[kPayloadOffset + 1] ^= 0x04;
    link.sendRaw(junk, sizeof(junk));
    link.sendRaw(frame, length);
    ++injections;
  });
  link.pump(milliseconds(150), true);
  const Status after = *link.lastStatus();
  printStatus(link.sinceOpen(), after);

  const auto delta = [](std::uint16_t a, std::uint16_t b) {
    return static_cast<std::uint16_t>(b - a);  // counters wrap
  };
  const std::uint32_t crc = delta(before->checksumErrors, after.checksumErrors);
  const std::uint32_t discarded = delta(before->bytesDiscarded, after.bytesDiscarded);
  const std::uint32_t frames = delta(before->framesReceived, after.framesReceived);
  std::printf("\nInjected %u times.\n", injections);
  std::printf("  checksum errors  +%u   (expect at least %u)\n", crc, injections);
  std::printf("  bytes discarded  +%u   (expect at least %u)\n", discarded, injections * 20);
  std::printf("  frames received  +%u   (the good Drives, about 100)\n", frames);
  const bool ok = crc >= injections && discarded >= injections * 20 &&
                  after.state == MotorState::Running && after.watchdogStops == before->watchdogStops;
  std::printf("%s\n", ok ? "PASS: errors counted, the link kept running."
                         : "FAIL: see the counts above.");
  stopAndReport(link);
  return ok ? 0 : 1;
}

int runSweep(Link &link, const Options &options) {
  waitForStartup(link);
  const std::size_t index = options.sweepMotor - 1;
  const auto hold = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(options.seconds));
  std::printf("Motor %zu from %d to 1000, then backward, %.1f s per value. Note the value\n"
              "where the wheel starts to turn. While the firmware's dead band is off\n"
              "(kMinDuty 0), the duty is about value x 255 / 1000. Ctrl-C to end.\n\n",
              options.sweepMotor, options.sweepStep, options.seconds);
  for (const int direction : {1, -1}) {
    for (int value = options.sweepStep; value <= kWheelMax && s_stopRequested == 0;
         value += options.sweepStep) {
      std::int16_t wheels[kMotorCount] = {0, 0, 0, 0};
      wheels[index] = static_cast<std::int16_t>(direction * value);
      driveFor(link, wheels, hold, true, [] {});
      const auto &s = link.lastStatus();
      std::printf("  value %5d  duty about %3d  applied %5d  %s\n", wheels[index],
                  (value * 255 + 500) / 1000, s ? s->applied[index] : 0,
                  s ? toText(s->state) : "no Status");
    }
    if (s_stopRequested != 0) {
      break;
    }
    // Stop between the two directions, so the reversal is not part of it.
    std::printf("\n");
    link.sendStop();
    const auto until = Clock::now() + milliseconds(500);
    while (Clock::now() < until) {
      link.pump(milliseconds(20), true);
    }
  }
  std::printf("Stop.\n");
  stopAndReport(link);
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  const Options options = parse(argc, argv);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    Link link(options.port);
    int result = 2;
    if (options.command == "info") {
      result = runInfo(link);
    } else if (options.command == "watch") {
      result = runWatch(link, options);
    } else if (options.command == "drive") {
      result = runDrive(link, options);
    } else if (options.command == "watchdog") {
      result = runWatchdog(link, options);
    } else if (options.command == "noise") {
      result = runNoise(link);
    } else if (options.command == "sweep") {
      result = runSweep(link, options);
    } else {
      std::fprintf(stderr, "ERROR: unknown command '%s'\n", options.command.c_str());
      usage(2);
    }
    if (s_stopRequested != 0 && options.command != "watch" && options.command != "info") {
      // Interrupted mid-run: leave the motors stopped, not to the watchdog.
      link.sendStop();
    }
    return result;
  } catch (const std::system_error &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    if (e.code() == std::errc::no_such_file_or_directory) {
      std::fprintf(stderr,
                   "  %s does not exist. On the Pi the Arduino must be in USB port 1-1.2\n"
                   "  for the udev rule to name it (target/udev/99-heron-arduino.rules).\n",
                   options.port.device.c_str());
    } else if (e.code() == std::errc::permission_denied) {
      std::fprintf(stderr,
                   "  Permission denied: add this user to the dialout group, then log in\n"
                   "  again:   sudo usermod -aG dialout \"$USER\"\n");
    }
    return 1;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
