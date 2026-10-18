// File        : OtosStart.cpp
// Author      : Chris Pane
// Description : Probe O2. The OTOS start-up sequence, then what a still
//               sensor reports: status, heading drift, raw accelerometer.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> otos_start
//     scripts/deploy_cpp.sh <user>@<pi> otos_start 60
//     scripts/deploy_cpp.sh <user>@<pi> otos_start 60 /dev/i2c-1
//
// Probe O1 (otos_bus) showed the bus is clean, and that without this
// sequence a still board reports a tilt warning and a heading drifting by
// degrees a minute. This does what docs/odometry-otos-reference.md §4 says
// SparkFun's libraries do at start-up -- written from that description, not
// from their code:
//
//   1. IMU calibration: write 0x06 = 255, poll until it reads 0. The board
//      must be still throughout (about 0.6 s, reference §4).
//   2. Reset tracking: write 0x07 = 1, poll until it reads 0. Pose to zero.
//
// No scalars and no offset: both stay at their power-up defaults.
//
// Then it watches for the given number of seconds, printing once a second
// the status, the pose and the raw accelerometer words (0x4A-0x4F). Which
// accelerometer axis carries gravity, and with what sign, shows how the
// board sees its own orientation, which bears on the tilt warning. The raw
// words are printed unscaled: the full-scale range the firmware sets is not
// known (reference §3), and the axes compare without it.
//
// Keep the board still and in its working orientation, sensor side down.

#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
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
constexpr std::uint8_t kRegRawImu = 0x44;  // gyro x, y, z then accel x, y, z

constexpr std::uint8_t kCalibrationSamples = 255;
constexpr std::uint8_t kResetTracking = 0x01;

// Register scalings, reference §3.
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerLsb = 10.0 / 32768.0;
constexpr double kRadiansPerLsb = kPi / 32768.0;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

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

struct Reading {
  std::uint8_t status;
  double x;
  double y;
  double h;
  std::int16_t accel[3];
};

// Status and pose from 0x1F, raw accelerometer from 0x4A. Two transactions,
// so the two halves are not one snapshot; for a still board that is fine.
bool readOnce(int fd, Reading &out) {
  std::uint8_t snap[7] = {};
  std::uint8_t imu[12] = {};
  if (readRegisters(fd, kRegStatus, snap, sizeof snap) != 0 ||
      readRegisters(fd, kRegRawImu, imu, sizeof imu) != 0) {
    return false;
  }
  out.status = snap[0];
  out.x = readInt16(&snap[1]) * kMetresPerLsb;
  out.y = readInt16(&snap[3]) * kMetresPerLsb;
  out.h = readInt16(&snap[5]) * kRadiansPerLsb;
  for (std::size_t i = 0; i < 3; ++i) {
    out.accel[i] = readInt16(&imu[6 + 2 * i]);
  }
  return true;
}

void printReading(const char *label, const Reading &r) {
  std::printf("%-7s status 0x%02X  pose %+8.4f %+8.4f m %+9.3f deg  accel x %+6d y %+6d z %+6d\n",
              label, r.status, r.x, r.y, r.h * 180.0 / kPi, r.accel[0], r.accel[1], r.accel[2]);
}

}  // namespace

int main(int argc, char **argv) {
  const std::int64_t seconds = (argc > 1) ? std::atoll(argv[1]) : 30;
  const char *device = (argc > 2) ? argv[2] : kDefaultDevice;
  if (seconds <= 0) {
    std::fprintf(stderr, "ERROR: seconds must be positive, got %s\n", argv[1]);
    return 1;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" Probe O2 -- OTOS start-up, then still\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s, address 0x%02X\n", device, kAddress);
  std::printf("  watch           %" PRId64 " s\n\n", seconds);

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

  Reading r{};
  if (!readOnce(fd, r)) {
    std::fprintf(stderr, "ERROR: first read failed\n");
    ::close(fd);
    return 1;
  }
  printReading("before", r);

  // -- start-up ---------------------------------------------------------------

  std::printf("\n  Calibrating the IMU: keep the board still...\n");
  std::fflush(stdout);
  const double calMs = writeAndWait(fd, kRegImuCalibration, kCalibrationSamples, "calibration");
  if (calMs < 0.0) {
    ::close(fd);
    return 1;
  }
  std::printf("  calibration     %u samples, %.0f ms\n", kCalibrationSamples, calMs);

  const double resetMs = writeAndWait(fd, kRegReset, kResetTracking, "reset");
  if (resetMs < 0.0) {
    ::close(fd);
    return 1;
  }
  std::printf("  reset           %.0f ms\n\n", resetMs);

  if (!readOnce(fd, r)) {
    std::fprintf(stderr, "ERROR: read after start-up failed\n");
    ::close(fd);
    return 1;
  }
  printReading("after", r);
  std::printf("\n");

  // -- watch ------------------------------------------------------------------

  const Reading start = r;
  std::uint8_t statusSeen = 0;
  std::uint64_t readings = 0;
  std::uint64_t failures = 0;
  double accelSum[3] = {};

  const auto begin = Clock::now();
  auto nextReport = begin + std::chrono::seconds(1);
  while (s_stopRequested == 0 && Clock::now() - begin < std::chrono::seconds(seconds)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!readOnce(fd, r)) {
      ++failures;
      continue;
    }
    ++readings;
    statusSeen = static_cast<std::uint8_t>(statusSeen | r.status);
    for (std::size_t i = 0; i < 3; ++i) {
      accelSum[i] += r.accel[i];
    }
    const auto now = Clock::now();
    if (now >= nextReport) {
      nextReport += std::chrono::seconds(1);
      char label[16];
      std::snprintf(label, sizeof label, "%3.0f s",
                    std::chrono::duration<double>(now - begin).count());
      printReading(label, r);
      std::fflush(stdout);
    }
  }
  const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
  ::close(fd);

  // -- summary ----------------------------------------------------------------

  std::printf("\n%s after %.1f s: %" PRIu64 " readings, %" PRIu64 " failed\n\n",
              s_stopRequested != 0 ? "Stopped by signal" : "Finished", elapsed, readings,
              failures);
  if (readings == 0) {
    return 1;
  }
  const double n = static_cast<double>(readings);
  const double driftDeg = (r.h - start.h) * 180.0 / kPi;
  std::printf("  status bits     0x%02X seen (bit 0 tilt, 1 optical tracking, 6 optical chip,"
              " 7 IMU)\n",
              statusSeen);
  std::printf("  pose change     %+.4f %+.4f m\n", r.x - start.x, r.y - start.y);
  std::printf("  heading drift   %+.3f deg, %+.2f deg/min\n", driftDeg,
              driftDeg * 60.0 / elapsed);
  std::printf("  mean accel raw  x %+.0f  y %+.0f  z %+.0f\n", accelSum[0] / n, accelSum[1] / n,
              accelSum[2] / n);
  return failures == 0 ? 0 : 1;
}
