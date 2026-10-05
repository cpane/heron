// File        : ReplayChannel.cpp
// Author      : Chris Pane
// Description : The lockstep gate, pacing and divergence checks of the replay.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <utility>

#include "ReplayChannel.h"

namespace Heron::Lidar {

namespace {

constexpr std::size_t kNoAnchor = std::numeric_limits<std::size_t>::max();

std::string toHex(const std::uint8_t *data, std::size_t size) {
  std::string out;
  out.reserve(size * 2);
  for (std::size_t i = 0; i < size; ++i) {
    char byte[4];
    std::snprintf(byte, sizeof byte, "%02x", data[i]);
    out += byte;
  }
  return out;
}

}  // namespace

ReplayChannel::ReplayChannel(std::vector<Chunk> chunks, double speed)
    : m_chunks(std::move(chunks)), m_speed(speed), m_anchor(m_chunks.size(), kNoAnchor),
      m_matchedAt(m_chunks.size()) {
  std::size_t lastTx = kNoAnchor;
  for (std::size_t i = 0; i < m_chunks.size(); ++i) {
    m_anchor[i] = lastTx;
    if (m_chunks[i].direction == Direction::Tx) {
      lastTx = i;
      ++m_recordedWrites;
    }
  }

  // Position both cursors on their first chunk of the right kind. No lock:
  // nothing else can see the object until the constructor returns.
  while (m_nextTx < m_chunks.size() && m_chunks[m_nextTx].direction != Direction::Tx) {
    ++m_nextTx;
  }
  skipToNextRxLocked();
}

bool ReplayChannel::open() {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_openedAt = SteadyClock::now();
  m_closed = false;
  return true;
}

void ReplayChannel::close() {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_closed = true;
  }
  m_changed.notify_all();
}

void ReplayChannel::skipToNextRxLocked() {
  while (m_rx < m_chunks.size() && m_chunks[m_rx].direction != Direction::Rx) {
    ++m_rx;
  }
  m_rxOffset = 0;
}

ReplayChannel::SteadyClock::time_point
ReplayChannel::computeReleaseTimeLocked(std::size_t index) const {
  if (m_speed <= 0.0) {
    return SteadyClock::time_point::min();
  }

  // Measure from the write this chunk followed in the recording, so time the
  // SDK spends differently between writes does not accumulate.
  const std::size_t anchor = m_anchor[index];
  const SteadyClock::time_point baseWall =
      (anchor == kNoAnchor) ? m_openedAt : m_matchedAt[anchor];
  const std::uint64_t baseT =
      (anchor == kNoAnchor) ? m_chunks.front().tMonoNs : m_chunks[anchor].tMonoNs;
  const std::uint64_t t = m_chunks[index].tMonoNs;
  const double deltaNs = (t > baseT) ? static_cast<double>(t - baseT) / m_speed : 0.0;
  return baseWall + std::chrono::duration_cast<SteadyClock::duration>(
                        std::chrono::duration<double, std::nano>(deltaNs));
}

bool ReplayChannel::isRxReadyLocked(SteadyClock::time_point now) const {
  if (m_closed || !m_divergence.empty()) {
    return false;
  }
  if (m_rx >= m_chunks.size()) {
    return false;
  }
  // Lockstep: not before every write recorded ahead of it has been matched.
  if (!(m_rx < m_nextTx)) {
    return false;
  }
  return now >= computeReleaseTimeLocked(m_rx);
}

sl_result ReplayChannel::waitForDataExt(std::size_t &sizeHint, sl_u32 timeoutInMs) {
  std::unique_lock<std::mutex> lock(m_mutex);
  sizeHint = 0;
  const SteadyClock::time_point deadline =
      SteadyClock::now() + std::chrono::milliseconds(timeoutInMs);

  for (;;) {
    const SteadyClock::time_point now = SteadyClock::now();
    if (isRxReadyLocked(now)) {
      sizeHint = m_chunks[m_rx].data.size() - m_rxOffset;
      return SL_RESULT_OK;
    }
    if (m_closed || now >= deadline) {
      return SL_RESULT_OPERATION_TIMEOUT;
    }

    // Wake for whichever comes first: the deadline, the next chunk's release
    // time, or a write that unblocks the lockstep gate.
    SteadyClock::time_point wake = deadline;
    if (m_divergence.empty() && m_rx < m_chunks.size() && m_rx < m_nextTx) {
      wake = std::min(wake, computeReleaseTimeLocked(m_rx));
    }
    m_changed.wait_until(lock, wake);
  }
}

bool ReplayChannel::waitForData(std::size_t size, sl_u32 timeoutInMs, std::size_t *actualReady) {
  std::size_t hint = 0;
  const bool ok = SL_IS_OK(waitForDataExt(hint, timeoutInMs)) && hint >= size;
  if (actualReady != nullptr) {
    *actualReady = hint;
  }
  return ok;
}

int ReplayChannel::read(void *buffer, std::size_t size) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!isRxReadyLocked(SteadyClock::now())) {
    return 0;
  }

  const std::vector<std::uint8_t> &data = m_chunks[m_rx].data;
  const std::size_t n = std::min(size, data.size() - m_rxOffset);
  std::memcpy(buffer, data.data() + m_rxOffset, n);
  m_rxOffset += n;
  if (m_rxOffset >= data.size()) {
    ++m_rx;
    skipToNextRxLocked();
  }
  return static_cast<int>(n);  // int: the SDK's IChannel::read() signature
}

int ReplayChannel::write(const void *data, std::size_t size) {
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const std::uint8_t *bytes = static_cast<const std::uint8_t *>(data);

    if (!m_divergence.empty()) {
      // Already off the recording; keep the SDK quiet rather than let a
      // failed write cascade into an error path that does not exist live.
    } else if (m_nextTx >= m_chunks.size()) {
      m_divergence = "write #" + std::to_string(m_matched + 1) +
                     " is beyond the recording, which has " +
                     std::to_string(m_recordedWrites) + " writes. Got " + toHex(bytes, size);
    } else {
      const std::vector<std::uint8_t> &expected = m_chunks[m_nextTx].data;
      if (expected.size() != size || std::memcmp(expected.data(), bytes, size) != 0) {
        m_divergence = "write #" + std::to_string(m_matched + 1) +
                       " differs from the recording. Expected " +
                       toHex(expected.data(), expected.size()) + ", got " + toHex(bytes, size);
      } else {
        m_matchedAt[m_nextTx] = SteadyClock::now();
        ++m_matched;
        ++m_nextTx;
        while (m_nextTx < m_chunks.size() && m_chunks[m_nextTx].direction != Direction::Tx) {
          ++m_nextTx;
        }
      }
    }
  }
  m_changed.notify_all();
  return static_cast<int>(size);  // int: the SDK's IChannel::write() signature
}

std::string ReplayChannel::getDivergence() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_divergence;
}

bool ReplayChannel::isFinished() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_divergence.empty() && m_nextTx >= m_chunks.size() && m_rx >= m_chunks.size();
}

std::size_t ReplayChannel::getWritesMatched() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_matched;
}

std::size_t ReplayChannel::getWritesRecorded() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_recordedWrites;
}

}  // namespace Heron::Lidar
