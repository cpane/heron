// File        : RemoteControl.h
// Author      : Chris Pane
// Description : The remote control as the robot sees it: start it, and read
//               the newest channels and link state from any thread.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Shaped like Heron::Lidar::Lidar on purpose: one owned reader thread,
// latest-wins publication, a health snapshot. Design and intended use:
// docs/remote-cpp-design.md.
//
// What this does NOT decide yet: whether a frame is a failsafe frame. Every
// source says the FS-iA6B keeps sending on radio loss, so the stale rule
// alone may not see it (docs/remote-ibus-protocol.md §5, unverified). The
// snapshot carries the raw frame so that check can be added once measured.

#ifndef HERON_REMOTE_REMOTE_CONTROL_H
#define HERON_REMOTE_REMOTE_CONTROL_H

#include <cstdint>
#include <memory>
#include <string>

#include "heron/remote/IBusParser.h"
#include "heron/remote/IBusReceiver.h"
#include "heron/remote/LinkMonitor.h"

namespace Heron::Remote {

/**
 * Lifecycle state.
 */
enum class RemoteState : std::uint8_t {
  Stopped,  // constructed, or stopped cleanly
  Running,  // the reader thread is decoding frames
  Failed    // start() failed, or the reader stopped on an error; see lastError
};

/**
 * The newest state of the link, as of the moment it was asked for.
 */
struct RemoteSnapshot {
  /** OK, STALE or NO SIGNAL, judged at the time of the getLatest() call,
   *  not when the frame arrived: a reader that has died reads as stale. */
  LinkState link{LinkState::NoSignal};
  /** The newest valid frame; all zero before the first. */
  IBusFrame frame{};
  /** Frames decoded so far: 0 before the first, and a gap between two calls
   *  says how many frames the caller did not see. */
  std::uint64_t seq{0};
  /** Time from the newest frame to the call; zero before the first. */
  Duration age{};

  /** @return true only when the link is OK: the one test for "may I act on
   *  these channels" until failsafe detection exists */
  [[nodiscard]] bool isOk() const noexcept { return link == LinkState::Ok; }
};

/**
 * Counters and state, for diagnostics and the robot's health reporting.
 */
struct RemoteHealth {
  RemoteState state{RemoteState::Stopped};
  IBusStats stats{};
  /** Times the link went from OK to stale while running. */
  std::uint64_t staleEvents{0};
  std::string lastError;
};

/**
 * The remote control receiver, read by a thread of its own.
 *
 * start() opens the port and launches a reader thread, which decodes every
 * frame and publishes the newest. Consumers call getLatest() at whatever rate
 * suits them; they never wait for a frame, and the link state is evaluated at
 * each call, so a stalled or dead reader shows as STALE rather than as an old
 * frame that looks current.
 *
 * Thread Safety: getLatest() and getHealth() may be called from any thread at
 * any rate and never block for long. start() and stop() are for one owning
 * thread.
 *
 * Usage:
 * @code
 * Heron::Remote::RemoteControl remote(Heron::Remote::ReceiverConfig{});
 * if (!remote.start()) { report(remote.getHealth().lastError); }
 * const Heron::Remote::RemoteSnapshot s = remote.getLatest();
 * if (!s.isOk()) { stopMotors(); }
 * @endcode
 */
class RemoteControl {
  public:
    explicit RemoteControl(const ReceiverConfig &config);

    /**
     * Stops the reader and closes the port.
     */
    ~RemoteControl();

    RemoteControl(const RemoteControl &) = delete;
    RemoteControl &operator=(const RemoteControl &) = delete;

    /**
     * Opens the port and starts the reader thread. Restarting a stopped or
     * failed instance is allowed; the counters start again from zero.
     *
     * @return true when running; false with getHealth().lastError set
     */
    bool start();

    /**
     * Stops the reader thread and closes the port. Returns within about one
     * read timeout (20 ms). Safe to call more than once, and on an instance
     * that never started.
     */
    void stop();

    /** @return The newest state, with the link judged now */
    [[nodiscard]] RemoteSnapshot getLatest() const;

    /**
     * @param now The time to judge the link at, steady_clock; for callers
     *            that already hold a timestamp for this control cycle
     * @return The newest state, with the link judged at now
     */
    [[nodiscard]] RemoteSnapshot getLatest(TimePoint now) const;

    /** @return A snapshot of the counters and state */
    [[nodiscard]] RemoteHealth getHealth() const;

  private:
    struct Impl;  // the thread, the receiver and the lock live here
    std::unique_ptr<Impl> m_impl;
};

/** @return "STOPPED", "RUNNING" or "FAILED" */
[[nodiscard]] const char *toString(RemoteState state) noexcept;

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_REMOTE_CONTROL_H
