// File        : MotorsTest.cpp
// Author      : Chris Pane
// Description : Motors end to end against a fake Arduino on a pseudo-terminal:
//               start-up and its failures, driving, Status, stale, a reset
//               mid-run, a lost Configure, stop.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The fake board runs the firmware's own logic: the same MotorController and
// FrameParser the Arduino compiles (firmware/motor/lib/, protocol/motor/),
// fed the bytes Motors writes, and answering with Boot and Status as
// main.cpp does. So it refuses Drive until configured, checks Configure,
// and reports the checksum exactly as the board does; only the pins and the
// serial port are missing. Its knobs simulate what the real board does to
// the Pi: a reset (Boot again, calibration lost), going quiet, old
// firmware, another platform, a dropped Configure.
//
// The pty stands in for USB serial: Motors opens the slave side, the fake
// reads and writes the master. The test also holds the slave open itself,
// in raw mode, so the pty survives Motors closing and reopening it and
// never echoes. A real board resets when the port opens; the fake cannot
// see that, so each test tells it when to send its Boot (bootIn()), after
// start() has opened the port. Timing margins are several times the 20 ms
// read timeout, so a loaded host does not make the test flaky.
//
// Same style as the other tests: no framework.

#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "MotorController.h"
#include "heron/motor/FrameParser.h"
#include "heron/motor/Motors.h"
#include "heron/motor/Protocol.h"

namespace {

using namespace Heron::Motor;
using std::chrono::milliseconds;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool contains(const std::string &text, const std::string &part) {
  return text.find(part) != std::string::npos;
}

void sleepMs(std::int64_t ms) {
  std::this_thread::sleep_for(milliseconds(ms));
}

// As the robot's loop does: one drive() every 20 ms. A single Drive would
// be stopped by the board's 250 ms watchdog.
void driveFor(Motors &motors, const WheelValues &wheels, std::int64_t ms) {
  for (std::int64_t t = 0; t < ms; t += 20) {
    (void)motors.drive(wheels);
    sleepMs(20);
  }
}

/** A fake Arduino running the firmware's logic on the master side of a pty. */
class FakeBoard {
  public:
    FakeBoard() {
      m_master = posix_openpt(O_RDWR | O_NOCTTY);
      if (m_master < 0 || grantpt(m_master) != 0 || unlockpt(m_master) != 0) {
        std::perror("posix_openpt");
        std::exit(2);
      }
      const char *name = ptsname(m_master);
      if (name == nullptr) {
        std::perror("ptsname");
        std::exit(2);
      }
      m_device = name;
      // Held open, raw, for the life of the test (see the file header).
      m_slave = ::open(m_device.c_str(), O_RDWR | O_NOCTTY);
      termios tio{};
      if (m_slave < 0 || tcgetattr(m_slave, &tio) != 0) {
        std::perror("open slave");
        std::exit(2);
      }
      cfmakeraw(&tio);
      tcsetattr(m_slave, TCSANOW, &tio);
      ::fcntl(m_master, F_SETFL, ::fcntl(m_master, F_GETFL) | O_NONBLOCK);
      m_thread = std::jthread([this](std::stop_token stop) { run(stop); });
    }

    ~FakeBoard() {
      m_thread.request_stop();
      m_thread.join();
      ::close(m_slave);
      ::close(m_master);
    }

    FakeBoard(const FakeBoard &) = delete;
    FakeBoard &operator=(const FakeBoard &) = delete;

    [[nodiscard]] const std::string &getDevice() const { return m_device; }

    /** Powers up delay from now: a fresh, unconfigured controller, then Boot.
     *  Until then it is in its bootloader: bytes are discarded, nothing sent. */
    void bootIn(milliseconds delay) {
      const std::lock_guard<std::mutex> lock(m_mutex);
      m_controller = MotorController{};
      m_parser = FrameParser{};
      m_booted = false;
      m_bootAt = Clock::now() + delay;
    }

    void setQuiet(bool quiet) { m_quiet = quiet; }
    void setProtocol(std::uint8_t version) { m_protocol = version; }
    void setPlatform(Platform platform) { m_platform = platform; }
    void dropConfigures(std::int32_t count) { m_dropConfigures = count; }

    [[nodiscard]] std::int32_t getDrives() const { return m_drives; }
    [[nodiscard]] std::int32_t getStops() const { return m_stops; }
    [[nodiscard]] std::int32_t getConfigures() const { return m_configures; }

    [[nodiscard]] DriveCommand getLastDrive() const {
      const std::lock_guard<std::mutex> lock(m_mutex);
      return m_lastDrive;
    }

    /** @return the checksum the board would report: 0 while unconfigured */
    [[nodiscard]] std::uint16_t getChecksum() const {
      const std::lock_guard<std::mutex> lock(m_mutex);
      return m_controller.isConfigured() ? configChecksum(m_controller.getConfig()) : 0;
    }

  private:
    // What main.cpp does in loop(), once a millisecond.
    void run(const std::stop_token &stop) {
      const TimePoint start = Clock::now();
      while (!stop.stop_requested()) {
        const TimePoint now = Clock::now();
        const auto nowMs = static_cast<std::uint32_t>(
            std::chrono::duration_cast<milliseconds>(now - start).count());
        std::uint8_t buf[256];
        const ssize_t n = ::read(m_master, buf, sizeof(buf));
        step(buf, n > 0 ? static_cast<std::size_t>(n) : 0, now, nowMs);
        // Outside the lock: std::mutex is not fair, and a loop that slept
        // holding it starved the test's own calls for seconds on the Pi.
        std::this_thread::sleep_for(milliseconds(1));
      }
    }

    // One pass of the board's loop, on bytes just read.
    void step(const std::uint8_t *buf, std::size_t n, TimePoint now, std::uint32_t nowMs) {
      const std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_booted) {
        if (m_bootAt && now >= *m_bootAt) {
          m_booted = true;
          m_bootAt.reset();
          sendBoot(nowMs);
        }
      } else {
        for (std::size_t i = 0; i < n; ++i) {
          m_parser.feed(buf[i], [this, nowMs](const Frame &frame) { onFrame(frame, nowMs); });
        }
        m_controller.update(nowMs);
        if (m_controller.takeBootRequest()) {
          sendBoot(nowMs);
        }
        if (m_controller.isStatusDue(nowMs) && !m_quiet) {
          const Status status = m_controller.makeStatus(m_parser.getStats(), 0, 0);
          std::uint8_t out[kMaxFrameLen];
          write(out, encodeStatus(status, out, sizeof(out)));
        }
      }
    }

    void onFrame(const Frame &frame, std::uint32_t nowMs) {
      DriveCommand drive;
      MotorConfig config;
      if (decodeDrive(frame, drive)) {
        ++m_drives;
        m_lastDrive = drive;
      } else if (frame.type == MessageType::Stop) {
        ++m_stops;
      } else if (decodeConfigure(frame, config)) {
        ++m_configures;
        if (m_dropConfigures > 0) {
          --m_dropConfigures;
          return;  // lost on the wire
        }
      }
      m_controller.onFrame(frame, nowMs);
    }

    void sendBoot(std::uint32_t nowMs) {
      BootInfo boot;
      boot.firmwareMajor = 0;
      boot.firmwareMinor = 3;
      boot.protocolVersion = m_protocol;
      boot.platform = m_platform;
      boot.uptimeMillis = nowMs;
      std::uint8_t out[kMaxFrameLen];
      write(out, encodeBoot(boot, out, sizeof(out)));
    }

    // Every frame goes out with the board's protocol version: for old
    // firmware, the version byte is changed and the CRC redone.
    void write(std::uint8_t *frame, std::size_t length) {
      if (m_protocol != kProtocolVersion) {
        frame[kVersionOffset] = m_protocol;
        putU16(frame + length - kChecksumLen, crc16(frame, length - kChecksumLen));
      }
      (void)!::write(m_master, frame, length);
    }

    int m_master{-1};
    int m_slave{-1};
    std::string m_device;

    std::atomic<bool> m_quiet{false};
    std::atomic<std::uint8_t> m_protocol{kProtocolVersion};
    std::atomic<Platform> m_platform{Platform::UnoL293d};
    std::atomic<std::int32_t> m_dropConfigures{0};
    std::atomic<std::int32_t> m_drives{0};
    std::atomic<std::int32_t> m_stops{0};
    std::atomic<std::int32_t> m_configures{0};

    mutable std::mutex m_mutex;
    MotorController m_controller;
    FrameParser m_parser;
    bool m_booted{false};
    std::optional<TimePoint> m_bootAt;
    DriveCommand m_lastDrive{};

    std::jthread m_thread;  // last: joined before the members it uses
};

// The bench rig's profile, without the ramp, so applied follows target.
Heron::Profile::MotorProfile benchMotor() {
  Heron::Profile::MotorProfile motor;
  motor.terminal = {1, 2, 4, 3};
  motor.sign = {1, 1, 1, -1};
  motor.rampPerMs = 0;
  motor.minDutyForward = {76, 81, 87, 92};
  motor.minDutyBackward = {71, 76, 76, 102};
  return motor;
}

MotorsConfig configFor(const FakeBoard &board) {
  MotorsConfig config;
  config.device = board.getDevice();
  config.platform = "uno-l293d";
  config.motor = benchMotor();
  config.startTimeout = milliseconds(1000);
  return config;
}

constexpr milliseconds kBootDelay(100);  // after start() has opened the port

}  // namespace

int main() {
  std::printf("Converting the profile\n");

  {
    const MotorConfig c = toMotorConfig(benchMotor());
    check("terminals copied", c.terminal[0] == 1 && c.terminal[2] == 4 && c.terminal[3] == 3);
    check("sign -1 sets the motor's reversed bit", c.reversed == 0x08);
    check("dead band copied, both directions",
          c.minDutyForward[3] == 92 && c.minDutyBackward[3] == 102 && c.rampPerMs == 0);
    check("platform names", toPlatform("uno-l293d") == Platform::UnoL293d &&
                                !toPlatform("uno").has_value() &&
                                std::string(toString(Platform::UnoL293d)) == "uno-l293d");
  }

  {
    Heron::Profile::RigProfile profile;
    profile.name = "bench";
    profile.platform = "uno-l293d";
    profile.motor = benchMotor();
    const MotorsConfig c = toMotorsConfig(profile);
    check("toMotorsConfig: default device, platform and [motor] from the profile",
          c.device == "/dev/arduino" && c.platform == "uno-l293d" && c.motor.terminal[2] == 4);
  }

  std::printf("Start-up\n");

  {
    FakeBoard board;
    Motors motors(configFor(board));
    check("drive() before start(): false, nothing sent",
          !motors.drive({0.5, 0.5, 0.5, 0.5}) && board.getDrives() == 0);

    board.bootIn(kBootDelay);
    const bool started = motors.start();
    const MotorsHealth h = motors.getHealth();
    check("start(): true, running", started && h.state == MotorsState::Running);
    if (!started) {
      std::printf("        lastError: %s\n", h.lastError.c_str());
    }
    check("start(): one Boot, one Configure", h.boots == 1 && h.configuresSent == 1 &&
                                                   h.resets == 0);
    check("start(): Boot recorded", h.boot.firmwareMinor == 3 &&
                                        h.boot.platform == Platform::UnoL293d);
    check("the board holds our calibration",
          board.getChecksum() == configChecksum(toMotorConfig(benchMotor())));
    const MotorsSnapshot s = motors.getLatest();
    check("snapshot: fresh, configured, OK", s.fresh && s.configured && s.isOk());
    check("snapshot: stopped, reason boot (configured, no Drive yet)",
          s.status.state == MotorState::Stopped && s.status.stopReason == StopReason::Boot);

    std::printf("Driving\n");

    check("drive(): sent", motors.drive({0.5, -0.25, 1.5, -2.0}));
    sleepMs(50);
    const DriveCommand d = board.getLastDrive();
    check("scaled by 1000 and clamped", d.wheels[0] == 500 && d.wheels[1] == -250 &&
                                            d.wheels[2] == 1000 && d.wheels[3] == -1000);
    driveFor(motors, {0.5, -0.25, 1.5, -2.0}, 300);
    const MotorsSnapshot running = motors.getLatest();
    check("Status shows it running, applied as sent",
          running.isOk() && running.status.state == MotorState::Running &&
              running.status.applied[0] == 500 && running.status.applied[1] == -250);

    (void)motors.drive({0.0004, 0.0005, -0.0005, 0.9996});
    sleepMs(50);
    const DriveCommand r = board.getLastDrive();
    check("rounded to the nearest unit, half away from zero",
          r.wheels[0] == 0 && r.wheels[1] == 1 && r.wheels[2] == -1 && r.wheels[3] == 1000);

    const std::int32_t stopsBefore = board.getStops();
    const std::int32_t drivesBefore = board.getDrives();
    check("drive() with NaN: false",
          !motors.drive({0.1, std::numeric_limits<double>::quiet_NaN(), 0.1, 0.1}));
    check("drive() with infinity: false",
          !motors.drive({std::numeric_limits<double>::infinity(), 0.1, 0.1, 0.1}));
    sleepMs(50);
    check("a non-finite value sends Stop instead, never a Drive",
          board.getStops() == stopsBefore + 2 && board.getDrives() == drivesBefore);

    (void)motors.drive({0.3, 0.3, 0.3, 0.3});
    check("stopMotors(): sent", motors.stopMotors());
    sleepMs(200);
    check("Status: stopped, reason Stop message",
          motors.getLatest().status.stopReason == StopReason::StopMessage);

    std::printf("Stale\n");

    board.setQuiet(true);
    sleepMs(450);
    const MotorsSnapshot quiet = motors.getLatest();
    check("no Status for 450 ms: not fresh, not OK", !quiet.fresh && !quiet.isOk());
    check("counted as a stale event", motors.getHealth().staleEvents == 1);
    board.setQuiet(false);
    sleepMs(200);
    check("Status again: OK", motors.getLatest().isOk());

    std::printf("A reset mid-run\n");

    (void)motors.drive({0.4, 0.4, 0.4, 0.4});
    board.bootIn(milliseconds(50));  // the board resets: calibration lost
    sleepMs(430);
    MotorsHealth afterReset = motors.getHealth();
    check("the reset is counted", afterReset.resets == 1 && afterReset.boots == 2);
    check("configured again at once", afterReset.configuresSent == 2 &&
                                          board.getChecksum() ==
                                              configChecksum(toMotorConfig(benchMotor())));
    check("OK again", motors.getLatest().isOk());
    check("still running: drive() sends", motors.drive({0.1, 0.1, 0.1, 0.1}));

    std::printf("A lost Configure\n");

    board.dropConfigures(1);
    board.bootIn(milliseconds(50));
    sleepMs(300);  // the resend is due 500 ms after the lost one
    check("the first Configure after the reset is lost: not configured",
          !motors.getLatest().configured && board.getChecksum() == 0);
    sleepMs(800);
    afterReset = motors.getHealth();
    check("sent again within the 500 ms interval, and taken",
          afterReset.configuresSent >= 4 && motors.getLatest().isOk() &&
              board.getChecksum() != 0);

    std::printf("Stopping\n");

    const std::int32_t stops = board.getStops();
    motors.stop();
    sleepMs(50);
    check("stop(): sends Stop", board.getStops() == stops + 1);
    check("stop(): state STOPPED", motors.getHealth().state == MotorsState::Stopped);
    check("after stop(): drive() and stopMotors() send nothing",
          !motors.drive({0.2, 0.2, 0.2, 0.2}) && !motors.stopMotors());
    sleepMs(450);
    check("after stop(): the snapshot goes stale", !motors.getLatest().fresh);

    board.bootIn(kBootDelay);
    check("restart: start() again works, counters from zero",
          motors.start() && motors.getHealth().boots == 1 && motors.getHealth().resets == 0);
  }

  std::printf("Start-up failures\n");

  {
    FakeBoard board;
    MotorsConfig config = configFor(board);
    config.startTimeout = milliseconds(300);
    Motors motors(config);
    const bool started = motors.start();  // no bootIn(): the board never boots
    const MotorsHealth h = motors.getHealth();
    check("no Boot: start() fails", !started && h.state == MotorsState::Failed);
    check("no Boot: the message says what to check",
          contains(h.lastError, "no Boot from") && contains(h.lastError, "firmware flashed"));
    check("no Boot: drive() sends nothing", !motors.drive({0.1, 0.1, 0.1, 0.1}));
  }

  {
    FakeBoard board;
    board.setProtocol(1);
    Motors motors(configFor(board));
    board.bootIn(kBootDelay);
    const bool started = motors.start();
    const std::string error = motors.getHealth().lastError;
    check("old firmware (protocol 1): start() fails", !started);
    check("old firmware: the message says reflash",
          contains(error, "another protocol version") && contains(error, "flash_firmware.sh"));
  }

  {
    FakeBoard board;
    board.setPlatform(Platform::Unknown);
    Motors motors(configFor(board));
    board.bootIn(kBootDelay);
    const bool started = motors.start();
    const std::string error = motors.getHealth().lastError;
    check("another platform: start() fails, naming both",
          !started && contains(error, "'unknown'") && contains(error, "'uno-l293d'"));
  }

  {
    FakeBoard board;
    board.dropConfigures(100);  // the board never takes a Configure
    Motors motors(configFor(board));
    board.bootIn(kBootDelay);
    const bool started = motors.start();
    check("Configure never confirmed: start() fails",
          !started && contains(motors.getHealth().lastError, "did not confirm"));
  }

  {
    FakeBoard board;
    MotorsConfig config = configFor(board);
    config.motor.terminal = {1, 1, 3, 4};
    Motors motors(config);
    check("an invalid [motor] section: start() fails before opening the port",
          !motors.start() && contains(motors.getHealth().lastError, "valid Configure"));
    config = configFor(board);
    config.platform = "uno";
    Motors other(config);
    check("an unknown platform: start() fails",
          !other.start() && contains(other.getHealth().lastError, "unknown platform 'uno'"));
  }

  {
    MotorsConfig config;
    config.device = "/nonexistent/arduino";
    config.platform = "uno-l293d";
    config.motor = benchMotor();
    Motors motors(config);
    check("a missing device: start() fails, naming it",
          !motors.start() && contains(motors.getHealth().lastError, "/nonexistent/arduino"));
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
