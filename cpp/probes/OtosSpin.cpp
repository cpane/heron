// File        : OtosSpin.cpp
// Author      : Chris Pane
// Description : Probe O4. Spin the OTOS a counted number of turns; compare
//               the heading it reports with the true angle.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> otos_spin
//     scripts/deploy_cpp.sh <user>@<pi> otos_spin /dev/i2c-1
//
// Answers the angular half of docs/odometry-otos-reference.md §10 question
// 5: how far off is the heading with the default angular scalar (1.0)?
// Reference §5: spin about 10 full turns; scalar = true angle / reported
// angle. Heading is gyro only, so the floor does not matter.
//
// Interactive, one trial at a time, shaped like probe O3 (otos_move):
//
//   1. Start-up as probe O2 (IMU calibration, reset): the sled must be
//      still.
//   2. Each trial: line an edge of the sled up with a mark; Enter resets
//      tracking; spin the sled in place a whole number of turns and line it
//      up with the mark again; Enter ends the trial. The probe prints the
//      total angle turned, then asks for the true number of turns. Given
//      one, it prints the error and the angular scalar that would correct
//      it.
//   3. 'q' and Enter at the prompt quits, with a table of all trials.
//
// The heading register wraps at +-180 deg, so the probe unwraps it: it adds
// up the change between reads, taken every 20 ms. That holds as long as the
// sled turns less than 180 deg between two reads, 25 turns a second.
//
// The true turns are entered unsigned; the sign is taken from the reported
// angle, so either direction of spin works.
//
// The x, y columns show how far the sensor moved during the spin: the
// board's position follows the sensor, not the centre of the spin, unless an
// offset is written (reference §6). Not written here.
//
// The start-up and the per-trial reset are the only writes. The scalars are
// read back, not set: this measures the uncorrected sensor.

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
constexpr std::uint8_t kRegLinearScalar = 0x04;  // then 0x05, angular
constexpr std::uint8_t kRegImuCalibration = 0x06;
constexpr std::uint8_t kRegReset = 0x07;
constexpr std::uint8_t kRegStatus = 0x1F;

constexpr std::uint8_t kCalibrationSamples = 255;
constexpr std::uint8_t kResetTracking = 0x01;

// Register scalings, reference §3.
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerLsb = 10.0 / 32768.0;
constexpr double kRadiansPerLsb = kPi / 32768.0;
constexpr double kScalarPerLsb = 0.001;

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

// The change from one heading to the next, taken the short way round.
double wrappedDelta(double from, double to) {
  double d = to - from;
  if (d > kPi) {
    d -= 2.0 * kPi;
  } else if (d < -kPi) {
    d += 2.0 * kPi;
  }
  return d;
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
  double reportedDeg;  // unwrapped, signed
  double trueDeg;      // signed like reportedDeg; 0 when not entered
  double finalDeg;     // the register's own reading at the end, wrapped
  double xMm;
  double yMm;
  double seconds;
  double maxRateDps;   // fastest turn seen between two reads
  std::uint8_t statusSeen;
  std::uint64_t failedReads;
};

// One trial: reset, then track until Enter. False if the sensor stops
// answering.
bool runTrial(int fd, Trial &trial) {
  if (writeAndWait(fd, kRegReset, kResetTracking, "reset") < 0.0) {
    return false;
  }
  std::printf("  Tracking. Spin the sled, line it up with the mark, then press Enter.\n");
  std::fflush(stdout);

  Pose start{};
  if (!readPose(fd, start)) {
    std::fprintf(stderr, "ERROR: read after reset failed\n");
    return false;
  }
  Pose last = start;
  auto lastAt = Clock::now();
  const auto begin = lastAt;
  double turned = 0.0;
  double maxRate = 0.0;
  std::uint8_t statusSeen = start.status;
  std::uint64_t failed = 0;
  auto nextShow = begin;

  while (!lineReady(std::chrono::milliseconds(20))) {
    Pose p{};
    if (!readPose(fd, p)) {
      ++failed;
      continue;
    }
    const auto now = Clock::now();
    const double delta = wrappedDelta(last.h, p.h);
    const double dt = std::chrono::duration<double>(now - lastAt).count();
    turned += delta;
    if (dt > 0.0) {
      maxRate = std::fmax(maxRate, std::fabs(delta) / dt);
    }
    statusSeen = static_cast<std::uint8_t>(statusSeen | p.status);
    last = p;
    lastAt = now;
    if (now >= nextShow) {
      nextShow += std::chrono::milliseconds(200);
      std::printf("\r  turned %+9.2f deg (%+6.2f turns)  heading %+7.2f deg  status 0x%02X ",
                  turned * 180.0 / kPi, turned / (2.0 * kPi), p.h * 180.0 / kPi, p.status);
      std::fflush(stdout);
    }
  }
  std::string discard;
  readLine(discard);

  trial.reportedDeg = turned * 180.0 / kPi;
  trial.finalDeg = last.h * 180.0 / kPi;
  trial.xMm = (last.x - start.x) * 1000.0;
  trial.yMm = (last.y - start.y) * 1000.0;
  trial.seconds = std::chrono::duration<double>(lastAt - begin).count();
  trial.maxRateDps = maxRate * 180.0 / kPi;
  trial.statusSeen = statusSeen;
  trial.failedReads = failed;
  trial.trueDeg = 0.0;
  return true;
}

void printTable(const std::vector<Trial> &trials) {
  std::printf("\n  #  reported deg  true deg  error %%  scalar  final h     x mm     y mm"
              "   time s  max deg/s  status  failed\n");
  for (std::size_t i = 0; i < trials.size(); ++i) {
    const Trial &t = trials[i];
    std::printf("%3zu  %+11.2f", i + 1, t.reportedDeg);
    if (t.trueDeg != 0.0) {
      std::printf("  %+8.1f  %+7.2f  %6.3f", t.trueDeg,
                  (t.reportedDeg - t.trueDeg) / t.trueDeg * 100.0, t.trueDeg / t.reportedDeg);
    } else {
      std::printf("  %8s  %7s  %6s", "-", "-", "-");
    }
    std::printf("  %+7.2f  %+7.1f  %+7.1f  %7.1f  %9.0f   0x%02X  %6" PRIu64 "\n", t.finalDeg,
                t.xMm, t.yMm, t.seconds, t.maxRateDps, t.statusSeen, t.failedReads);
  }
}

}  // namespace

int main(int argc, char **argv) {
  const char *device = (argc > 1) ? argv[1] : kDefaultDevice;

  std::printf("========================================\n");
  std::printf(" Probe O4 -- OTOS, spun a counted number of turns\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s, address 0x%02X\n", device, kAddress);

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

  // Read back, not set: shows the defaults are in force (reference §3, §4).
  std::uint8_t scalars[2] = {};
  if (readRegisters(fd, kRegLinearScalar, scalars, sizeof scalars) != 0) {
    std::fprintf(stderr, "ERROR: read of the scalars failed\n");
    ::close(fd);
    return 1;
  }
  const auto linearRaw = static_cast<std::int8_t>(scalars[0]);
  const auto angularRaw = static_cast<std::int8_t>(scalars[1]);
  std::printf("  scalars         linear %.3f (raw %d), angular %.3f (raw %d): not set\n\n",
              1.0 + linearRaw * kScalarPerLsb, linearRaw, 1.0 + angularRaw * kScalarPerLsb,
              angularRaw);

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
    std::printf("\nTrial %zu: line the sled up with the mark, then press Enter"
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
    std::printf("\n\n  reported        %+.2f deg (%+.3f turns) in %.1f s, fastest %.0f deg/s\n",
                t.reportedDeg, t.reportedDeg / 360.0, t.seconds, t.maxRateDps);
    std::printf("  final heading   %+.2f deg as the register reads it\n", t.finalDeg);
    std::printf("  moved           x %+.1f, y %+.1f mm\n", t.xMm, t.yMm);
    std::printf("  status bits     0x%02X (bit 0 tilt, 1 optical tracking, 6 optical chip,"
                " 7 IMU)\n",
                t.statusSeen);
    if (t.failedReads > 0) {
      std::printf("  failed reads    %" PRIu64 "\n", t.failedReads);
    }

    std::printf("\n  True whole turns (Enter to skip): ");
    std::fflush(stdout);
    if (!readLine(line)) {
      trials.push_back(t);
      break;
    }
    const double turns = std::atof(line.c_str());
    if (turns > 0.0 && t.reportedDeg != 0.0) {
      t.trueDeg = std::copysign(turns * 360.0, t.reportedDeg);
      std::printf("  error           %+.2f %%  (angular scalar %.3f)\n",
                  (t.reportedDeg - t.trueDeg) / t.trueDeg * 100.0, t.trueDeg / t.reportedDeg);
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
