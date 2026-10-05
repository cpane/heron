// File        : ReplayChannel.h
// Author      : Chris Pane
// Description : Plays a .rpraw recorded by TapChannel back to the SDK, in
//               lockstep.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Below the boundary: includes SDK headers. Only the adapter, probes and
// tests may use it. The overridden methods keep the SDK's own names and
// signatures, which are the vendor's interface and outside this standard.

#ifndef HERON_LIDAR_REPLAY_CHANNEL_H
#define HERON_LIDAR_REPLAY_CHANNEL_H

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <sl_lidar_driver.h>

#include "lidar/Rpraw.h"

namespace Heron::Lidar {

/**
 * An SDK channel that serves a recorded session back, in lockstep.
 *
 * The SDK is not a passive reader. connect() sends GET_DEVICE_INFO, and
 * startScan()/startScanExpress() send a run of GET_LIDAR_CONF queries before
 * the scan command, each waiting for its answer. So a replay cannot just pour
 * recorded bytes in: each answer has to arrive after the question.
 *
 * Lockstep is how. A received chunk is released only once every write that
 * preceded it in the recording has been matched by an identical write from
 * the SDK. The writes are checked byte for byte, so a replay session that
 * asks the device something the recording did not ask is caught at once and
 * reported, rather than fed the wrong answer.
 *
 * Pacing: released chunks are served at the recorded rate (speed 1.0 = real
 * time), measured from the most recent matched write. Real time is the
 * default, not an indulgence: the SDK publishes scans latest-wins from a
 * double buffer, so serving a stream faster than the consumer grabs it
 * silently drops whole revolutions.
 *
 * Reports CHANNEL_TYPE_TCP, deliberately. For a device without PWM motor
 * control the SDK downcasts any CHANNEL_TYPE_SERIALPORT channel to
 * ISerialPortChannel unchecked (sl_lidar_driver.cpp, setMotorSpeed()), and
 * motor control means nothing to a recording. The only other branch on the
 * type is baud-rate detection, which a replay never runs.
 *
 * Thread Safety: thread-safe. The SDK calls write() from the caller's thread
 * and waitForDataExt()/read() from its receive thread.
 *
 * Usage:
 * @code
 * Rpraw raw = readRpraw(path);
 * ReplayChannel channel(std::move(raw.chunks));
 * driver->connect(&channel);
 * // ... the same SDK calls as the recorded session ...
 * if (!channel.getDivergence().empty()) { ... }
 * @endcode
 */
class ReplayChannel final : public sl::IChannel {
  public:
    /**
     * @param chunks The recorded session, in file order
     * @param speed Scales the recorded timing: 1.0 is real time, 2.0 twice as
     *        fast. 0 serves everything as soon as it is released -- fine for
     *        the command dialogue, lossy for a stream (see above).
     */
    explicit ReplayChannel(std::vector<Chunk> chunks, double speed = 1.0);

    bool open() override;
    void close() override;
    void flush() override {}
    bool waitForData(std::size_t size, sl_u32 timeoutInMs, std::size_t *actualReady) override;
    sl_result waitForDataExt(std::size_t &sizeHint, sl_u32 timeoutInMs) override;
    int write(const void *data, std::size_t size) override;
    int read(void *buffer, std::size_t size) override;
    void clearReadCache() override {}
    int getChannelType() override { return sl::CHANNEL_TYPE_TCP; }

    /**
     * @return Empty while every SDK write has matched the recording.
     *         Otherwise, what was expected and what arrived. Once set,
     *         nothing more is served.
     */
    [[nodiscard]] std::string getDivergence() const;

    /**
     * @return true once every recorded write has matched and every recorded
     *         byte has been served
     */
    [[nodiscard]] bool isFinished() const;

    /**
     * @return How many recorded writes the SDK has matched so far
     */
    [[nodiscard]] std::size_t getWritesMatched() const;

    /**
     * @return How many writes the recording holds
     */
    [[nodiscard]] std::size_t getWritesRecorded() const;

  private:
    using SteadyClock = std::chrono::steady_clock;

    // All *Locked() methods expect m_mutex to be held.
    [[nodiscard]] bool isRxReadyLocked(SteadyClock::time_point now) const;
    [[nodiscard]] SteadyClock::time_point computeReleaseTimeLocked(std::size_t index) const;
    void skipToNextRxLocked();

    std::vector<Chunk> m_chunks;
    const double m_speed;

    // m_anchor[i]: the index of the last tx chunk before chunk i, or kNoAnchor.
    std::vector<std::size_t> m_anchor;
    // When each tx chunk was matched in this replay; only valid once matched.
    std::vector<SteadyClock::time_point> m_matchedAt;
    SteadyClock::time_point m_openedAt{};

    std::size_t m_nextTx{0};    // the next tx chunk awaiting a match
    std::size_t m_rx{0};        // the next rx chunk to serve
    std::size_t m_rxOffset{0};  // bytes of m_chunks[m_rx] already served
    std::size_t m_matched{0};
    std::size_t m_recordedWrites{0};
    bool m_closed{false};
    std::string m_divergence;

    mutable std::mutex m_mutex;
    std::condition_variable m_changed;
};

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_REPLAY_CHANNEL_H
