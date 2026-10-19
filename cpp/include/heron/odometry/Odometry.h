// File        : Odometry.h
// Author      : Chris Pane
// Description : The odometry sensor as the robot sees it: start it, and read
//               the newest pose from any thread.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Shaped like Heron::Lidar::Lidar and Heron::Remote::RemoteControl on
// purpose: one owned reader thread, latest-wins publication, a health
// snapshot. Design and intended use: docs/odometry-cpp-design.md.
//
// start() blocks for about 0.65 s and the sensor must be still and level
// throughout: it calibrates the IMU (docs/odometry-otos-reference.md §4).
// So the robot starts odometry with its motors off.

#ifndef HERON_ODOMETRY_ODOMETRY_H
#define HERON_ODOMETRY_ODOMETRY_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "heron/odometry/OtosConfig.h"
#include "heron/odometry/Planar.h"

namespace Heron::Odometry {

class I2cBus;  // private to the library; named here only for the test seam

/**
 * Lifecycle state.
 */
enum class OdometryState : std::uint8_t {
  Stopped,  // constructed, or stopped cleanly
  Running,  // the reader thread is polling the sensor
  Failed    // start() failed, or the reader stopped on an error; see lastError
};

/**
 * The newest reading, as of the moment it was asked for.
 */
struct OdometrySnapshot {
  /** Running, and the newest reading is no older than staleAfter; judged at
   *  the getLatest() call, so a dead reader reads as not fresh. */
  bool fresh{false};
  /** Readings so far, the one start() took included; a gap between two
   *  calls says how many the caller did not see. 0 before the first. */
  std::uint64_t seq{0};
  /** Time from the newest reading to the call; zero before the first. */
  Duration age{};
  /** Register 0x1F as read: bit 0 tilt, 1 optical tracking, 6 optical chip
   *  error, 7 IMU error. */
  std::uint8_t status{0};
  Planar pose{};              // m, m, rad; heading wrapped to [-pi, pi)
  Planar velocity{};          // m/s, m/s, rad/s
  Planar acceleration{};      // m/s^2, m/s^2, rad/s^2
  /** The board's filter statistic for the pose, not a measured error. */
  Planar poseStdDev{};
  /** Heading unwrapped since start(), rad: counts whole turns. */
  double totalHeading{0.0};

  /** @return true if the board reports more than about 11.5 deg of tilt */
  [[nodiscard]] bool tiltWarning() const noexcept { return (status & 0x01) != 0; }
  /** @return true if optical tracking is unreliable: x and y may be off,
   *  the heading (gyro) is not affected */
  [[nodiscard]] bool trackingWarning() const noexcept { return (status & 0x02) != 0; }
  /** @return true if the optical chip or the IMU reports an error */
  [[nodiscard]] bool sensorError() const noexcept { return (status & 0xC0) != 0; }
  /** @return true when fresh with no sensor error: the one test for "may I
   *  use this reading". The warnings do not fail it; the tilt bit was set
   *  through accurate runs (docs/odometry-cpp-design.md §2.2). */
  [[nodiscard]] bool isOk() const noexcept { return fresh && !sensorError(); }
};

/**
 * Counters and state, for diagnostics and the robot's health reporting.
 */
struct OdometryHealth {
  OdometryState state{OdometryState::Stopped};
  std::uint8_t hardwareMajor{0};      // 1.1 on ours
  std::uint8_t hardwareMinor{0};
  std::uint8_t firmwareMajor{0};      // 1.0 on ours
  std::uint8_t firmwareMinor{0};
  Duration calibrationTime{};         // the last start(); about 0.62 s
  std::uint64_t readings{0};
  std::uint64_t failedReads{0};
  std::uint64_t tiltSnapshots{0};
  std::uint64_t trackingWarnings{0};
  std::uint8_t statusSeen{0};         // every status bit seen, ORed
  std::string lastError;
};

/**
 * The OTOS odometry sensor, read by a thread of its own.
 *
 * start() runs the sensor's start-up sequence and launches a reader thread,
 * which reads a snapshot every pollPeriod and publishes the newest.
 * Consumers call getLatest() at whatever rate suits them; they never wait,
 * and freshness is judged at each call.
 *
 * Thread Safety: getLatest() and getHealth() may be called from any thread at
 * any rate and never block for long. start() and stop() are for one owning
 * thread.
 *
 * Usage:
 * @code
 * Heron::Odometry::Odometry odometry(Heron::Odometry::OtosConfig{});
 * if (!odometry.start()) { report(odometry.getHealth().lastError); }  // still!
 * const Heron::Odometry::OdometrySnapshot o = odometry.getLatest();
 * if (o.isOk()) { useHeading(o.totalHeading); }
 * @endcode
 */
class Odometry {
  public:
    /** Opens the bus start() will use. Tests pass a fake; robot code never
     *  needs this. May throw; start() reports what it throws. */
    using BusFactory = std::function<std::unique_ptr<I2cBus>()>;

    /**
     * @param config Bus, scalars, offset and timing; checked by start(),
     *               which reports a bad one through lastError
     */
    explicit Odometry(const OtosConfig &config);

    /**
     * @param config As above; config.device is not used
     * @param openBus Called by each start() for the bus to use
     */
    Odometry(const OtosConfig &config, BusFactory openBus);

    /**
     * Stops the reader and closes the bus.
     */
    ~Odometry();

    Odometry(const Odometry &) = delete;
    Odometry &operator=(const Odometry &) = delete;

    /**
     * Opens the bus, runs the start-up sequence and starts the reader thread.
     * Blocks for about 0.65 s; the sensor must be still and level. Restarting
     * a stopped or failed instance is allowed, and recalibrates; the counters
     * and totalHeading start again from zero.
     *
     * @return true when running; false with getHealth().lastError set
     */
    bool start();

    /**
     * Stops the reader thread and closes the bus. Returns within about one
     * poll period and a read. Safe to call more than once, and on an instance
     * that never started.
     */
    void stop();

    /** @return The newest reading, with freshness judged now */
    [[nodiscard]] OdometrySnapshot getLatest() const;

    /**
     * @param now The time to judge freshness at, steady_clock; for callers
     *            that already hold a timestamp for this control cycle
     * @return The newest reading, with freshness judged at now
     */
    [[nodiscard]] OdometrySnapshot getLatest(TimePoint now) const;

    /** @return A snapshot of the counters and state */
    [[nodiscard]] OdometryHealth getHealth() const;

  private:
    struct Impl;  // the thread, the bus and the lock live here
    std::unique_ptr<Impl> m_impl;
};

/** @return "STOPPED", "RUNNING" or "FAILED" */
[[nodiscard]] const char *toString(OdometryState state) noexcept;

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_ODOMETRY_H
