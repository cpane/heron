// File        : IBusParser.cpp
// Author      : Chris Pane
// Description : FlySky iBUS frame parser: header search, checksum, channel
//               decode and statistics.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/remote/IBusParser.h"

#include <algorithm>

namespace Heron::Remote {

namespace {

std::uint16_t le16(std::uint8_t lo, std::uint8_t hi) {
  return static_cast<std::uint16_t>(lo | (hi << 8));
}

}  // namespace

std::uint16_t IBusParser::checksum(std::span<const std::uint8_t> frame) noexcept {
  std::uint16_t sum = 0xFFFF;
  const std::size_t n = std::min(frame.size(), kIBusChecksumOffset);
  for (std::size_t i = 0; i < n; ++i) {
    sum = static_cast<std::uint16_t>(sum - frame[i]);
  }
  return sum;
}

std::size_t IBusParser::feed(std::span<const std::uint8_t> bytes, TimePoint stamp,
                             const FrameHandler &onFrame) {
  m_stats.bytesReceived += bytes.size();
  std::size_t decoded = 0;
  for (const std::uint8_t byte : bytes) {
    m_buf[m_len++] = byte;
    // Re-examine the front after every byte. Dropping one byte can expose a
    // header further in, so this loops until the front is a plausible frame
    // start that is still filling, or a complete frame has been taken.
    while (m_len > 0) {
      if (m_buf[0] != kIBusHeader0 || (m_len >= 2 && m_buf[1] != kIBusHeader1)) {
        dropFront();
        continue;
      }
      if (m_len < kIBusFrameLen) {
        break;
      }
      const std::uint16_t expected = checksum(m_buf);
      const std::uint16_t received =
          le16(m_buf[kIBusChecksumOffset], m_buf[kIBusChecksumOffset + 1]);
      if (expected != received) {
        ++m_stats.checksumErrors;
        dropFront();
        continue;
      }
      decodeFrame(stamp, onFrame);
      ++decoded;
      m_len = 0;
    }
  }
  return decoded;
}

void IBusParser::dropFront() noexcept {
  if (!m_hunting) {
    m_hunting = true;
    ++m_stats.resyncs;
  }
  ++m_stats.discardedBytes;
  std::copy(m_buf.begin() + 1, m_buf.begin() + static_cast<std::ptrdiff_t>(m_len),
            m_buf.begin());
  --m_len;
}

void IBusParser::decodeFrame(TimePoint stamp, const FrameHandler &onFrame) {
  m_hunting = false;
  IBusFrame frame;
  frame.bytes = m_buf;
  frame.stamp = stamp;
  for (std::size_t i = 0; i < kIBusChannelCount; ++i) {
    frame.channels[i] = le16(m_buf[2 + 2 * i], m_buf[3 + 2 * i]);
  }

  if (m_stats.framesDecoded == 0) {
    m_stats.firstFrame = stamp;
  } else {
    const Duration interval = stamp - m_stats.lastFrame;
    m_stats.minInterval = std::min(m_stats.minInterval, interval);
    m_stats.maxInterval = std::max(m_stats.maxInterval, interval);
    m_stats.totalInterval += interval;
  }
  m_stats.lastFrame = stamp;
  ++m_stats.framesDecoded;

  if (onFrame) {
    onFrame(frame);
  }
}

}  // namespace Heron::Remote
