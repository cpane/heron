// File        : OtosMove.cpp
// Author      : Chris Pane
// Description : Probe O3. Push the OTOS a measured distance; compare what it
//               reports with the tape measure.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> otos_move
//     scripts/deploy_cpp.sh <user>@<pi> otos_move /dev/i2c-1
//
// Answers docs/odometry-otos-reference.md §10 questions 4-6 a first time:
// does the optical tracking work on our floor at our height, and how far off
// is the distance with the default scalars (1.0)?
//
// Interactive, one trial at a time:
//
//   1. Start-up as probe O2 (IMU calibration, reset): the sled must be
//      still.
//   2. Each trial: Enter resets tracking (pose to zero); push the sled along
//      the tape measure and stop; Enter ends the trial. The probe prints the
//      reported straight-line distance, x, y, heading change, path length
//      and the status bits seen, then asks for the true distance in mm.
//      Given one, it prints the error and the linear scalar that would
//      correct it (reference §5: scalar = true / measured).
//   3. 'q' and Enter at the prompt quits, with a table of all trials.
//
// While a trial runs, one line updates in place five times a second.
//
// The start-up and the per-trial reset are the only writes. No scalars are
// set: this measures the uncorrected sensor.

#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

const char *kDefaultDevice = "/dev/i2c-1";
constexpr std::uint16_t kAddress = 0x17;
constexpr std::uint8_t kProductId = 0x5F;

constexpr std::uint8_t kRegProductId = 0x00;
constexpr std::uint8_t kRegImuCalibration = 0x06;
constexpr std::uint8_t kRegReset = 0x07;
constexpr std::uint8_t kRegStatus = 0x1F;

constexpr std::uint8_t kCalibrationSamples = 255;
constexpr std::uint8_t kResetTracking = 0x01;

// Register scalings, reference §3.
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerLsb = 10.0 / 32768.0;
constexpr double kRadiansPerLsb = kPi / 32768.0;

using Clock = std::chrono::steady_clock;

// Signed 16-bit little-endian, reference §3.
std::int16_t readInt16(const std::uint8_t *bytes) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(bytes[0]) |
                                   (static_cast<std::uint16_t>(bytes[1]) << 8));
}

// Register write and read in one combined transaction (repeated start).
// Returns 0, or the errno of the failure.
std::int32_t readRegisters(int fd, std::uint8_t reg, std::uint8_t *out, std::size_t count) {
  i2c_msg messages[2];
  messages[0].addr = kAddress;
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &reg;
  messages[1].addr = kAddress;
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<std::uint16_t>(count);
  messages[1].buf = out;

  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = messages;
  transfer.nmsgs = 2;
  if (::ioctl(fd, I2C_RDWR, &transfer) < 0) {
    return errno;
  }
  return 0;
}

// One register, one byte: the index and the value in a single write.
std::int32_t writeRegister(int fd, std::uint8_t reg, std::uint8_t value) {
  std::uint8_t bytes[2] = {reg, value};
  i2c_msg message;
  message.addr = kAddress;
  message.flags = 0;
  message.len = sizeof bytes;
  message.buf = bytes;

  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = &message;
  transfer.nmsgs = 1;
  if (::ioctl(fd, I2C_RDWR, &transfer) < 0) {
    return errno;
  }
  return 0;
}

// Write a self-clearing register and poll until it reads 0. Returns the
// time taken in milliseconds, or a negative value on failure or timeout.
double writeAndWait(int fd, std::uint8_t reg, std::uint8_t value, const char *what) {
  constexpr std::chrono::milliseconds kPoll(10);
  constexpr std::chrono::seconds kTimeout(5);

  const auto begin = Clock::now();
  const std::int32_t err = writeRegister(fd, reg, value);
  if (err != 0) {
    std::fprintf(stderr, "ERROR: %s: write to 0x%02X failed: %s\n", what, reg,
                 std::strerror(err));
    return -1.0;
  }
  std::uint8_t remaining = value;
  while (remaining != 0) {
    if (Clock::now() - begin > kTimeout) {
      std::fprintf(stderr, "ERROR: %s: 0x%02X still reads %u after %lld s\n", what, reg,
                   remaining, static_cast<long long>(kTimeout.count()));
      return -1.0;
    }
    std::this_thread::sleep_for(kPoll);
    const std::int32_t readErr = readRegisters(fd, reg, &remaining, 1);
    if (readErr != 0) {
      std::fprintf(stderr, "ERROR: %s: read of 0x%02X failed: %s\n", what, reg,
                   std::strerror(readErr));
      return -1.0;
    }
  }
  return std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
}

struct Pose {
  std::uint8_t status;
  double x;
  double y;
  double h;
};

bool readPose(int fd, Pose &out) {
  std::uint8_t snap[7] = {};
  if (readRegisters(fd, kRegStatus, snap, sizeof snap) != 0) {
    return false;
  }
  out.status = snap[0];
  out.x = readInt16(&snap[1]) * kMetresPerLsb;
  out.y = readInt16(&snap[3]) * kMetresPerLsb;
  out.h = readInt16(&snap[5]) * kRadiansPerLsb;
  return true;
}

// A line typed at the terminal, without its newline. False at end of input.
bool readLine(std::string &line) {
  line.clear();
  char c = 0;
  while (::read(STDIN_FILENO, &c, 1) == 1) {
    if (c == '\n') {
      return true;
    }
    if (c != '\r') {
      line += c;
    }
  }
  return false;
}

// True once a line has been typed, without blocking longer than the timeout.
bool lineReady(std::chrono::milliseconds timeout) {
  pollfd p{STDIN_FILENO, POLLIN, 0};
  return ::poll(&p, 1, static_cast<int>(timeout.count())) > 0;
}

struct Trial {
  double reportedMm;
  double trueMm;  // 0 when not entered
  double xMm;
  double yMm;
  double headingDeg;
  double pathMm;
  std::uint8_t statusSeen;
  std::uint64_t failedReads;
};

// One trial: reset, then track until Enter. False if the sensor stops
// answering.
bool runTrial(int fd, Trial &trial) {
  if (writeAndWait(fd, kRegReset, kResetTracking, "reset") < 0.0) {
    return false;
  }
  std::printf("  Tracking. Push the sled, stop, then press Enter.\n");
  std::fflush(stdout);

  Pose start{};
  if (!readPose(fd, start)) {
    std::fprintf(stderr, "ERROR: read after reset failed\n");
    return false;
  }
  Pose last = start;
  double path = 0.0;
  std::uint8_t statusSeen = start.status;
  std::uint64_t failed = 0;
  auto nextShow = Clock::now();

  while (!lineReady(std::chrono::milliseconds(20))) {
    Pose p{};
    if (!readPose(fd, p)) {
      ++failed;
      continue;
    }
    path += std::hypot(p.x - last.x, p.y - last.y);
    statusSeen = static_cast<std::uint8_t>(statusSeen | p.status);
    last = p;
    if (Clock::now() >= nextShow) {
      nextShow += std::chrono::milliseconds(200);
      std::printf("\r  dist %7.1f mm  x %+8.1f  y %+8.1f mm  heading %+7.2f deg  status 0x%02X ",
                  std::hypot(p.x - start.x, p.y - start.y) * 1000.0, p.x * 1000.0,
                  p.y * 1000.0, (p.h - start.h) * 180.0 / kPi, p.status);
      std::fflush(stdout);
    }
  }
  std::string discard;
  readLine(discard);

  trial.xMm = (last.x - start.x) * 1000.0;
  trial.yMm = (last.y - start.y) * 1000.0;
  trial.reportedMm = std::hypot(trial.xMm, trial.yMm);
  trial.headingDeg = (last.h - start.h) * 180.0 / kPi;
  trial.pathMm = path * 1000.0;
  trial.statusSeen = statusSeen;
  trial.failedReads = failed;
  trial.trueMm = 0.0;
  return true;
}

void printTable(const std::vector<Trial> &trials) {
  std::printf("\n  #  reported mm   true mm   error %%  scalar   x mm     y mm     heading  path mm"
              "  status  failed\n");
  for (std::size_t i = 0; i < trials.size(); ++i) {
    const Trial &t = trials[i];
    std::printf("%3zu  %10.1f", i + 1, t.reportedMm);
    if (t.trueMm > 0.0) {
      std::printf("  %8.1f  %+7.2f  %6.3f", t.trueMm,
                  (t.reportedMm - t.trueMm) / t.trueMm * 100.0, t.trueMm / t.reportedMm);
    } else {
      std::printf("  %8s  %7s  %6s", "-", "-", "-");
    }
    std::printf("  %+7.1f  %+7.1f  %+7.2f  %7.1f   0x%02X  %6" PRIu64 "\n", t.xMm, t.yMm,
                t.headingDeg, t.pathMm, t.statusSeen, t.failedReads);
  }
}

}  // namespace

int main(int argc, char **argv) {
  const char *device = (argc > 1) ? argv[1] : kDefaultDevice;

  std::printf("========================================\n");
  std::printf(" Probe O3 -- OTOS, pushed a measured distance\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s, address 0x%02X\n", device, kAddress);
  std::printf("  scalars         defaults (1.0): measuring the uncorrected sensor\n\n");

  const int fd = ::open(device, O_RDWR);
  if (fd < 0) {
    std::fprintf(stderr, "ERROR: cannot open %s: %s\n", device, std::strerror(errno));
    return 1;
  }

  std::uint8_t id = 0;
  const std::int32_t idErr = readRegisters(fd, kRegProductId, &id, 1);
  if (idErr != 0 || id != kProductId) {
    std::fprintf(stderr, "ERROR: no OTOS at 0x%02X (%s, ID 0x%02X)\n", kAddress,
                 idErr != 0 ? std::strerror(idErr) : "read OK", id);
    ::close(fd);
    return 1;
  }

  std::printf("  Calibrating the IMU: keep the sled still...\n");
  std::fflush(stdout);
  const double calMs = writeAndWait(fd, kRegImuCalibration, kCalibrationSamples, "calibration");
  if (calMs < 0.0) {
    ::close(fd);
    return 1;
  }
  std::printf("  calibration     %u samples, %.0f ms\n", kCalibrationSamples, calMs);

  std::vector<Trial> trials;
  std::string line;
  for (;;) {
    std::printf("\nTrial %zu: put the sled at the start mark, then press Enter"
                " ('q' Enter to finish).\n",
                trials.size() + 1);
    std::fflush(stdout);
    if (!readLine(line) || line == "q") {
      break;
    }

    Trial t{};
    if (!runTrial(fd, t)) {
      break;
    }
    std::printf("\n\n  reported        %.1f mm (x %+.1f, y %+.1f), heading %+.2f deg\n",
                t.reportedMm, t.xMm, t.yMm, t.headingDeg);
    std::printf("  path length     %.1f mm\n", t.pathMm);
    std::printf("  status bits     0x%02X (bit 0 tilt, 1 optical tracking, 6 optical chip,"
                " 7 IMU)\n",
                t.statusSeen);
    if (t.failedReads > 0) {
      std::printf("  failed reads    %" PRIu64 "\n", t.failedReads);
    }

    std::printf("\n  True distance in mm (Enter to skip): ");
    std::fflush(stdout);
    if (!readLine(line)) {
      trials.push_back(t);
      break;
    }
    const double trueMm = std::atof(line.c_str());
    if (trueMm > 0.0 && t.reportedMm > 0.0) {
      t.trueMm = trueMm;
      std::printf("  error           %+.2f %%  (linear scalar %.3f)\n",
                  (t.reportedMm - trueMm) / trueMm * 100.0, trueMm / t.reportedMm);
    }
    trials.push_back(t);
  }
  ::close(fd);

  if (trials.empty()) {
    std::printf("\nNo trials.\n");
    return 0;
  }
  printTable(trials);
  return 0;
}
