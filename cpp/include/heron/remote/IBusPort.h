// File        : IBusPort.h
// Author      : Chris Pane
// Description : Receive-only serial port for the iBUS receiver: termios
//               setup and poll()-based reads with a timeout.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Opened read-only, so nothing in this program can transmit on the line.
// Knows nothing about frames: bytes out of read() go to an IBusParser.
//
// Uses POSIX termios and poll, which the standard library does not provide.
// Errors are std::system_error, so a caller can tell a missing device
// (no_such_file_or_directory) from a permissions problem (permission_denied).

#ifndef HERON_REMOTE_IBUS_PORT_H
#define HERON_REMOTE_IBUS_PORT_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace Heron::Remote {

/**
 * Serial settings. Data bits are always 8 and parity always none.
 */
struct SerialConfig {
  std::string device{"/dev/serial0"};
  std::uint32_t baud{115200};
  /** 1 or 2. Sources disagree on 8N1 vs 8N2 for iBUS; a receiver set for
   *  one stop bit also accepts two, so 1 is the tolerant choice (see
   *  docs/remote-ibus-protocol.md §2). */
  std::uint8_t stopBits{1};
};

/**
 * An open serial port, closed on destruction.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 */
class IBusPort {
  public:
    /**
     * Opens and configures the port, and discards anything already queued.
     *
     * @param config Device and line settings
     * @throws std::invalid_argument for an unsupported baud rate or stop bits
     * @throws std::system_error if the device cannot be opened or configured;
     *         code() carries the errno
     */
    explicit IBusPort(const SerialConfig &config);
    ~IBusPort();

    IBusPort(const IBusPort &) = delete;
    IBusPort &operator=(const IBusPort &) = delete;

    /**
     * Waits up to timeout for bytes and reads what is available.
     *
     * @param buffer Where to put them
     * @param timeout How long to wait; never busy-waits
     * @return Bytes read; 0 on timeout or when interrupted by a signal
     * @throws std::system_error if the device fails or goes away
     */
    std::size_t read(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout);

    [[nodiscard]] const SerialConfig &getConfig() const noexcept { return m_config; }

  private:
    const SerialConfig m_config;
    int m_fd{-1};  // int: the POSIX descriptor type
};

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_IBUS_PORT_H
