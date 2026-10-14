// File        : OtosBus.cpp
// Author      : Chris Pane
// Description : Probe O1. Can the Pi read the OTOS reliably over I2C?
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> otos_bus
//     scripts/deploy_cpp.sh <user>@<pi> otos_bus 5000
//     scripts/deploy_cpp.sh <user>@<pi> otos_bus 5000 /dev/i2c-1
//
// Answers docs/odometry-otos-reference.md §10 question 1. The board stretches
// the I2C clock and the Pi 3's controller is reported to mishandle that
// (reference §3), so before any library is designed: does a 36-byte burst
// from 0x20 come back in one combined transaction, every time?
//
// Each iteration does two transactions, each a register write and a read
// joined by a repeated start (I2C_RDWR), so nothing can come between them:
//
//   1. 0x00, 3 bytes: product ID, hardware and firmware version. The ID is
//      a known constant, 0x5F, so a wrong value is a corrupted read -- the
//      only check here that does not depend on the sensor's state.
//   2. 0x1F, 37 bytes: status, then the 36-byte block of pose, velocity
//      and acceleration from 0x20. One byte earlier than SparkFun's 36 from
//      0x20, so each snapshot carries its own status.
//
// Corruption that keeps the ID intact would show as jumps in the pose. With
// the sensor still, the pose barely moves; a jump over the thresholds below
// between consecutive reads is counted. Moving the sensor by hand makes jumps
// legitimate, so keep it still for a clean count.
//
// Deliberately writes nothing to the sensor: no calibration, no reset, no
// scalars. Only register indexes go on the bus. The pose is whatever the
// board has integrated since power-up.

#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
constexpr std::uint8_t kRegStatus = 0x1F;
constexpr std::size_t kSnapshotBytes = 37;  // status + 36 from 0x20

// Register scalings, reference §3.
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetresPerLsb = 10.0 / 32768.0;
constexpr double kRadiansPerLsb = kPi / 32768.0;

// A jump between consecutive reads of a still sensor. Reads are a few
// milliseconds apart, so anything near these is not motion on a bench.
constexpr double kJumpMetres = 0.01;
constexpr double kJumpRadians = 0.05;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

using Clock = std::chrono::steady_clock;

struct Pose {
  double x;
  double y;
  double h;
};

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

// The few errno values a failed I2C transfer gives, counted apart: which one
// it is says something about the failure (no ACK vs timeout vs bus fault).
struct ErrorCounts {
  std::uint64_t total = 0;
  std::uint64_t noAck = 0;    // ENXIO, EREMOTEIO
  std::uint64_t timeout = 0;  // ETIMEDOUT
  std::uint64_t io = 0;       // EIO
  std::uint64_t other = 0;
  std::int32_t lastErrno = 0;

  void add(std::int32_t err) {
    ++total;
    lastErrno = err;
    if (err == ENXIO || err == EREMOTEIO) {
      ++noAck;
    } else if (err == ETIMEDOUT) {
      ++timeout;
    } else if (err == EIO) {
      ++io;
    } else {
      ++other;
    }
  }
};

void printErrors(const char *label, const ErrorCounts &e) {
  std::printf("  %-16s %" PRIu64 " failed", label, e.total);
  if (e.total > 0) {
    std::printf(" (no ACK %" PRIu64 ", timeout %" PRIu64 ", EIO %" PRIu64 ", other %" PRIu64
                "; last: %s)",
                e.noAck, e.timeout, e.io, e.other, std::strerror(e.lastErrno));
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char **argv) {
  const std::int64_t reads = (argc > 1) ? std::atoll(argv[1]) : 2000;
  const char *device = (argc > 2) ? argv[2] : kDefaultDevice;
  if (reads <= 0) {
    std::fprintf(stderr, "ERROR: read count must be positive, got %s\n", argv[1]);
    return 1;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" Probe O1 -- OTOS over I2C\n");
  std::printf("========================================\n\n");
  std::printf("  device          %s, address 0x%02X\n", device, kAddress);
  std::printf("  reads           %" PRId64 "\n\n", reads);

  const int fd = ::open(device, O_RDWR);
  if (fd < 0) {
    std::fprintf(stderr,
                 "ERROR: cannot open %s: %s\n"
                 "  Check provisioning has run and the Pi was rebooted (i2c_arm,\n"
                 "  i2c-dev), and that you are in the 'i2c' group.\n",
                 device, std::strerror(errno));
    return 1;
  }

  // I2C_RDWR needs the adapter to do plain I2C, not just SMBus. The Pi's
  // does; check rather than fail obscurely later.
  unsigned long funcs = 0;
  if (::ioctl(fd, I2C_FUNCS, &funcs) < 0 || (funcs & I2C_FUNC_I2C) == 0) {
    std::fprintf(stderr, "ERROR: %s does not support combined I2C transfers\n", device);
    ::close(fd);
    return 1;
  }

  // -- identity ---------------------------------------------------------------

  std::uint8_t id[3] = {};
  const std::int32_t idErr = readRegisters(fd, kRegProductId, id, sizeof id);
  if (idErr != 0) {
    std::fprintf(stderr,
                 "ERROR: no answer from 0x%02X: %s\n"
                 "  i2cdetect should show 17. If it does and this fails, the\n"
                 "  combined transfer itself is the problem.\n",
                 kAddress, std::strerror(idErr));
    ::close(fd);
    return 1;
  }
  std::printf("  product ID      0x%02X %s\n", id[0],
              id[0] == kProductId ? "(OTOS)" : "(EXPECTED 0x5F)");
  std::printf("  hardware        %u.%u\n", id[1] >> 4, id[1] & 0x0F);
  std::printf("  firmware        %u.%u\n\n", id[2] >> 4, id[2] & 0x0F);

  // -- the loop ---------------------------------------------------------------

  ErrorCounts idErrors;
  ErrorCounts snapshotErrors;
  std::uint64_t badIds = 0;
  std::uint64_t jumps = 0;
  std::uint64_t snapshots = 0;
  std::uint64_t statusWarnings = 0;
  std::uint8_t statusSeen = 0;  // OR of every status byte
  double minMs = 1e9;
  double maxMs = 0.0;
  double sumMs = 0.0;

  Pose first{};
  Pose last{};
  bool havePose = false;

  const auto begin = Clock::now();
  auto nextReport = begin + std::chrono::seconds(1);
  std::int64_t done = 0;
  for (; done < reads && s_stopRequested == 0; ++done) {
    std::uint8_t check[1] = {};
    const std::int32_t checkErr = readRegisters(fd, kRegProductId, check, sizeof check);
    if (checkErr != 0) {
      idErrors.add(checkErr);
    } else if (check[0] != kProductId) {
      ++badIds;
    }

    std::uint8_t snap[kSnapshotBytes] = {};
    const auto t0 = Clock::now();
    const std::int32_t snapErr = readRegisters(fd, kRegStatus, snap, sizeof snap);
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    if (snapErr != 0) {
      snapshotErrors.add(snapErr);
    } else {
      ++snapshots;
      minMs = (ms < minMs) ? ms : minMs;
      maxMs = (ms > maxMs) ? ms : maxMs;
      sumMs += ms;
      statusSeen = static_cast<std::uint8_t>(statusSeen | snap[0]);
      if (snap[0] != 0) {
        ++statusWarnings;
      }
      const Pose p{readInt16(&snap[1]) * kMetresPerLsb, readInt16(&snap[3]) * kMetresPerLsb,
                   readInt16(&snap[5]) * kRadiansPerLsb};
      if (havePose) {
        if (std::fabs(p.x - last.x) > kJumpMetres || std::fabs(p.y - last.y) > kJumpMetres ||
            std::fabs(p.h - last.h) > kJumpRadians) {
          ++jumps;
        }
      } else {
        first = p;
        havePose = true;
      }
      last = p;
    }

    const auto now = Clock::now();
    if (now >= nextReport) {
      nextReport += std::chrono::seconds(1);
      std::printf("%3.0f s  %6" PRId64 " reads  errors %" PRIu64 "  bad ID %" PRIu64
                  "  jumps %" PRIu64 "  status 0x%02X  pose %+8.4f %+8.4f m %+8.3f deg\n",
                  std::chrono::duration<double>(now - begin).count(), done + 1,
                  idErrors.total + snapshotErrors.total, badIds, jumps, snap[0], last.x, last.y,
                  last.h * 180.0 / kPi);
      std::fflush(stdout);
    }
  }
  const double seconds = std::chrono::duration<double>(Clock::now() - begin).count();
  ::close(fd);

  // -- summary ----------------------------------------------------------------

  std::printf("\n%s after %" PRId64 " iterations in %.1f s (%.0f per second)\n\n",
              s_stopRequested != 0 ? "Stopped by signal" : "Finished", done, seconds,
              static_cast<double>(done) / seconds);
  printErrors("ID reads", idErrors);
  printErrors("snapshot reads", snapshotErrors);
  std::printf("  bad product ID   %" PRIu64 "\n", badIds);
  std::printf("  pose jumps       %" PRIu64 " (over %.0f mm or %.1f deg between reads)\n", jumps,
              kJumpMetres * 1000.0, kJumpRadians * 180.0 / kPi);
  if (snapshots > 0) {
    std::printf("  snapshot time    min %.2f  avg %.2f  max %.2f ms\n", minMs,
                sumMs / static_cast<double>(snapshots), maxMs);
    std::printf("  status           non-zero in %" PRIu64 " of %" PRIu64
                " snapshots; bits seen 0x%02X\n",
                statusWarnings, snapshots, statusSeen);
    std::printf("                   (bit 0 tilt, 1 optical tracking, 6 optical chip, 7 IMU)\n");
    std::printf("  pose drift       %+.4f %+.4f m %+.3f deg over the run\n", last.x - first.x,
                last.y - first.y, (last.h - first.h) * 180.0 / kPi);
  }

  const bool clean =
      idErrors.total == 0 && snapshotErrors.total == 0 && badIds == 0 && jumps == 0;
  std::printf("\n  %s\n", clean ? "Clean: every read succeeded and made sense."
                                : "NOT clean: see the counts above.");
  return clean ? 0 : 1;
}
