// File        : TapChannel.cpp
// Author      : Chris Pane
// Description : Forwarding to the real serial channel, recording as it goes.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include <cstdint>

#include "TapChannel.h"

namespace Heron::Lidar {

TapChannel::TapChannel(sl::ISerialPortChannel &inner, RprawWriter &writer)
    : m_inner(inner), m_writer(writer) {}

bool TapChannel::open() {
  return m_inner.open();
}

void TapChannel::close() {
  m_inner.close();
}

void TapChannel::flush() {
  m_inner.flush();
}

bool TapChannel::waitForData(std::size_t size, sl_u32 timeoutInMs, std::size_t *actualReady) {
  return m_inner.waitForData(size, timeoutInMs, actualReady);
}

sl_result TapChannel::waitForDataExt(std::size_t &sizeHint, sl_u32 timeoutInMs) {
  return m_inner.waitForDataExt(sizeHint, timeoutInMs);
}

int TapChannel::write(const void *data, std::size_t size) {
  // Stamp before the call: the time the SDK decided to send, which is what
  // replay pacing anchors on.
  const std::uint64_t t = nowMonoNs();
  const int sent = m_inner.write(data, size);
  if (sent > 0) {
    m_writer.writeChunk(Direction::Tx, t, data, static_cast<std::size_t>(sent));
  }
  return sent;
}

int TapChannel::read(void *buffer, std::size_t size) {
  const int got = m_inner.read(buffer, size);
  if (got > 0) {
    m_writer.writeChunk(Direction::Rx, nowMonoNs(), buffer, static_cast<std::size_t>(got));
  }
  return got;
}

void TapChannel::clearReadCache() {
  m_inner.clearReadCache();
}

int TapChannel::getChannelType() {
  return m_inner.getChannelType();
}

void TapChannel::setDTR(bool dtr) {
  m_inner.setDTR(dtr);
}

}  // namespace Heron::Lidar
