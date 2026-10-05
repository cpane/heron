// File        : Lidar.h
// Author      : Chris Pane
// Description : The LiDAR adapter's public interface: start it, and read the
//               latest Scan from any thread.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Together with Scan.h this is the boundary: robot code includes these two
// headers and never sees an SDK type. Everything SDK-specific lives behind the
// pimpl in cpp/src/lidar/Lidar.cpp. Design: docs/lidar-cpp-design.md, §5.

#ifndef HERON_LIDAR_LIDAR_H
#define HERON_LIDAR_LIDAR_H

#include <cstdint>
#include <memory>
#include <string>

#include "heron/lidar/Scan.h"

namespace Heron::Lidar {

/**
 * Settings fixed for the life of a Lidar.
 */
struct LidarConfig {
  std::string port{"/dev/rplidar"};  // never ttyUSB0: enumeration order moves
  std::uint32_t baud{115200};
  /** 1 (Express) by default: its decode is proven against Python. 3 is the
   *  device's own default, selectable once its ultra decode is checked. */
  std::int32_t scanMode{1};
  /** Robot-frame bearing of the sensor's front; 0 until measured. */
  float mountingOffsetRad{0.0f};
  /** Revolutions ignored after start while the motor comes up to speed. */
  std::int32_t discardRevolutions{3};
  /** Testing only: read this .rpraw (recorded by lidar_record) instead of the
   *  port. The adapter makes the same SDK calls as the recorded session, so
   *  the recording replays in lockstep. Empty means the real sensor. */
  std::string replayFile;
  double replaySpeed{1.0};
};

/**
 * Lifecycle state.
 */
enum class LidarState : std::uint8_t {
  Stopped,  // constructed, or stopped cleanly
  Running,  // the reader thread is publishing scans
  Failed    // start() failed, or the reader stopped on an error; see lastError
};

/**
 * Counters and state, for diagnostics and the robot's health reporting.
 *
 * A first version: the robot-wide health shape is to be designed with the
 * second sensor (docs/lidar-cpp-design.md, §10).
 */
struct LidarHealth {
  LidarState state{LidarState::Stopped};
  std::uint64_t scansPublished{0};
  /** Revolutions not published because the sample count showed two merged
   *  after an overrun. They still consume a seq number, so consumers see
   *  the gap. */
  std::uint64_t scansDroppedMerged{0};
  /** Revolutions not published because they were drained from a backlog
   *  after the reader stalled: the SDK stamps them on arrival, so they would
   *  claim to be fresh while holding data up to the stall's length old. */
  std::uint64_t scansDroppedBacklog{0};
  /** Revolutions published with samples missing (droppedEstimate > 0). */
  std::uint64_t scansWithLoss{0};
  /** Grabs that waited a full timeout with no revolution arriving. */
  std::uint64_t grabTimeouts{0};
  std::string lastError;
};

/**
 * What the device reported at start(): its identity and the scan mode in use.
 *
 * Captured from calls start() makes anyway, so asking costs no SDK traffic
 * and works the same against a replay.
 */
struct LidarInfo {
  std::uint8_t model{0};
  std::string firmware;     // "major.minor", e.g. "1.29"
  std::uint8_t hardware{0};
  /** The 16-byte serial as hex, in the SDK's byte order. Which order the
   *  housing label uses is not settled (docs/cpp-build.md). */
  std::string serialHex;
  std::int32_t scanModeId{-1};
  std::string scanModeName;
  float usPerSample{0.0f};
  float maxDistanceM{0.0f};
  std::uint8_t answerType{0};  // 0x81 legacy, 0x82 express, 0x84 ultra
};

/**
 * The LiDAR, as the robot sees it.
 *
 * start() connects, checks the device answers, starts the scan and launches a
 * reader thread. That thread turns each revolution into a Scan and publishes
 * it, latest-wins: a consumer that is slower than ~7.3 Hz misses scans rather
 * than delaying them, and Scan::seq shows how many.
 *
 * Two kinds of revolution are never published, because their times or
 * geometry cannot be trusted (decided 2026-10-04): two revolutions merged
 * after an overrun, and revolutions drained in a burst after the reader
 * stalled. Both still advance seq, and both are counted in LidarHealth. After
 * a stall a consumer therefore sees no new scan for a second or two, and its
 * freshness check (docs/lidar-cpp-design.md, §7.1) stops the robot.
 *
 * Thread Safety: getLatest() and getHealth() may be called from any thread at
 * any rate and never block for long. start() and stop() are for one owning
 * thread.
 *
 * Usage:
 * @code
 * Heron::Lidar::Lidar lidar(Heron::Lidar::LidarConfig{});
 * if (!lidar.start()) { report(lidar.getHealth().lastError); }
 * std::shared_ptr<const Heron::Scan> scan = lidar.getLatest();
 * @endcode
 */
class Lidar {
  public:
    explicit Lidar(const LidarConfig &config);

    /**
     * Stops the reader, the scan and the motor. Always.
     */
    ~Lidar();

    Lidar(const Lidar &) = delete;
    Lidar &operator=(const Lidar &) = delete;

    /**
     * Connects, verifies the device answers, starts the scan and the reader.
     *
     * @return true when running; false with getHealth().lastError set
     */
    bool start();

    /**
     * Stops the reader thread, the scan and the motor, and disconnects.
     * Safe to call more than once, and on a Lidar that never started.
     */
    void stop();

    /**
     * @return The newest published revolution, or nullptr before the first.
     *         The Scan is immutable and stays valid while the pointer is held.
     */
    [[nodiscard]] std::shared_ptr<const Scan> getLatest() const;

    /**
     * @return A snapshot of the counters and state
     */
    [[nodiscard]] LidarHealth getHealth() const;

    /**
     * @return What the device reported at the last successful start();
     *         default values before the first
     */
    [[nodiscard]] LidarInfo getInfo() const;

  private:
    struct Impl;  // SDK types live here and nowhere else
    std::unique_ptr<Impl> m_impl;
};

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_LIDAR_H
