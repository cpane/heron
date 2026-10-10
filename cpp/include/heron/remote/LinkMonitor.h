// File        : LinkMonitor.h
// Author      : Chris Pane
// Description : The stale-link rule for the remote control: the link is lost
//               when no valid frame has arrived for a set time.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Header-only and clock-free: the caller passes "now", so the rule is testable
// without waiting, and every program applies it identically. Stale is judged
// on valid frames only; bytes that fail the checksum do not keep a link alive.
//
// Whether the receiver keeps sending frames after the transmitter is lost is
// a property of the receiver, recorded (once measured) in
// docs/remote-ibus-protocol.md §5. This rule alone is only sufficient if it
// stops sending; if it repeats values instead, a stale check cannot see it.

#ifndef HERON_REMOTE_LINK_MONITOR_H
#define HERON_REMOTE_LINK_MONITOR_H

#include <chrono>
#include <cstdint>
#include <stdexcept>

#include "heron/remote/IBusParser.h"

namespace Heron::Remote {

/** No valid frame for this long means the link is lost. About 14 frames. */
inline constexpr Duration kDefaultStaleAfter = std::chrono::milliseconds(100);

enum class LinkState : std::uint8_t {
  NoSignal,  // no valid frame has ever arrived
  Ok,        // the last valid frame is recent enough
  Stale      // there was one, but too long ago
};

/**
 * Tracks the time of the last valid frame and applies the stale rule.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 */
class LinkMonitor {
  public:
    /**
     * @param staleAfter No valid frame for longer than this means Stale
     * @throws std::invalid_argument if staleAfter is not positive
     */
    explicit LinkMonitor(Duration staleAfter = kDefaultStaleAfter) : m_staleAfter(staleAfter) {
      if (staleAfter <= Duration::zero()) {
        throw std::invalid_argument(
            "Heron::Remote::LinkMonitor::LinkMonitor() : staleAfter must be positive");
      }
    }

    /** @param stamp When a valid frame arrived */
    void onFrame(TimePoint stamp) noexcept {
      m_lastFrame = stamp;
      m_haveFrame = true;
    }

    /**
     * @param now The current time, steady_clock
     * @return The link state at now
     */
    [[nodiscard]] LinkState getState(TimePoint now) const noexcept {
      if (!m_haveFrame) {
        return LinkState::NoSignal;
      }
      return now - m_lastFrame > m_staleAfter ? LinkState::Stale : LinkState::Ok;
    }

    /** @return true only when the link state at now is Ok */
    [[nodiscard]] bool isOk(TimePoint now) const noexcept {
      return getState(now) == LinkState::Ok;
    }

    /** @return Time since the last valid frame; zero if there has been none */
    [[nodiscard]] Duration sinceLastFrame(TimePoint now) const noexcept {
      return m_haveFrame ? now - m_lastFrame : Duration::zero();
    }

    [[nodiscard]] bool hasFrame() const noexcept { return m_haveFrame; }
    [[nodiscard]] TimePoint getLastFrame() const noexcept { return m_lastFrame; }
    [[nodiscard]] Duration getStaleAfter() const noexcept { return m_staleAfter; }

  private:
    Duration m_staleAfter;
    TimePoint m_lastFrame{};
    bool m_haveFrame{false};
};

/** @return "NO SIGNAL", "OK" or "STALE" */
[[nodiscard]] inline const char *toString(LinkState state) noexcept {
  switch (state) {
    case LinkState::NoSignal:
      return "NO SIGNAL";
    case LinkState::Ok:
      return "OK";
    case LinkState::Stale:
      return "STALE";
  }
  return "?";
}

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_LINK_MONITOR_H
