// File        : IBusReceiver.h
// Author      : Chris Pane
// Description : The iBUS receiver, synchronous: serial port, frame parser and
//               stale-link rule, driven by the caller one read at a time.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// For code that wants every frame: ibus_reader's dump, record and stats, and
// RemoteControl's reader thread. Robot code wants the newest state from a
// thread of its own instead, which is RemoteControl (docs/remote-cpp-design.md).

#ifndef HERON_REMOTE_IBUS_RECEIVER_H
#define HERON_REMOTE_IBUS_RECEIVER_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

#include "heron/remote/IBusParser.h"
#include "heron/remote/IBusPort.h"
#include "heron/remote/LinkMonitor.h"

namespace Heron::Remote {

/**
 * Settings fixed for the life of a receiver.
 */
struct ReceiverConfig {
  SerialConfig serial{};
  Duration staleAfter{kDefaultStaleAfter};
};

/**
 * Port, parser and link rule, as one object.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 *
 * Usage:
 * @code
 * IBusReceiver rx(ReceiverConfig{});
 * for (;;) {
 *   rx.pump(std::chrono::milliseconds(20), [](const IBusFrame &f) { use(f); });
 *   if (!rx.getLink().isOk(Clock::now())) { stop(); }
 * }
 * @endcode
 */
class IBusReceiver {
  public:
    /**
     * Opens the port.
     *
     * @param config Port settings and the stale-link time
     * @throws std::invalid_argument for bad settings
     * @throws std::system_error if the port cannot be opened (see IBusPort)
     */
    explicit IBusReceiver(const ReceiverConfig &config)
        : m_port(config.serial), m_link(config.staleAfter) {}

    /**
     * Waits up to timeout for bytes, decodes them, and updates the link rule.
     *
     * @param timeout How long to wait; never busy-waits
     * @param onFrame Called once per valid frame, in order; may be empty
     * @return Frames decoded by this call
     * @throws std::system_error if the device fails or goes away
     */
    std::size_t pump(std::chrono::milliseconds timeout,
                     const IBusParser::FrameHandler &onFrame = {}) {
      const std::size_t n = m_port.read(m_buf, timeout);
      if (n > 0) {
        m_sawBytes = true;
      }
      return m_parser.feed(std::span(m_buf.data(), n), Clock::now(), [&](const IBusFrame &f) {
        m_link.onFrame(f.stamp);
        if (onFrame) {
          onFrame(f);
        }
      });
    }

    [[nodiscard]] const IBusStats &getStats() const noexcept { return m_parser.getStats(); }
    void resetStats() noexcept { m_parser.resetStats(); }
    [[nodiscard]] const LinkMonitor &getLink() const noexcept { return m_link; }
    /** @return true once any byte has arrived, valid or not: tells "silent
     *  line" from "wrong data" when no frame has */
    [[nodiscard]] bool sawBytes() const noexcept { return m_sawBytes; }
    [[nodiscard]] const SerialConfig &getConfig() const noexcept { return m_port.getConfig(); }

  private:
    IBusPort m_port;
    IBusParser m_parser;
    LinkMonitor m_link;
    std::array<std::uint8_t, 256> m_buf{};
    bool m_sawBytes{false};
};

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_IBUS_RECEIVER_H
