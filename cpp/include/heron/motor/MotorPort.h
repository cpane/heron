// File        : MotorPort.h
// Author      : Chris Pane
// Description : The Pi's serial port to the Arduino motor controller: termios
//               setup, reads with a timeout, and writes.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The byte pipe between the Pi and the Arduino: /dev/arduino, a USB serial
// port, at 115200 baud, 8 data bits, no parity, one stop bit, raw (no echo,
// no line editing, no character translation). It knows nothing about
// frames. What goes in is built with Protocol.h's encode*() functions; what
// comes out goes to a FrameParser:
//
//   MotorPort port(MotorPortConfig{});            // opens /dev/arduino
//   FrameParser parser;
//
//   uint8_t frame[kMaxFrameLen];
//   port.write(frame, encodeDrive(cmd, frame, sizeof(frame)));
//
//   uint8_t buf[256];
//   const size_t n = port.read(buf, std::chrono::milliseconds(20));
//   parser.feed(buf, n, [&](const Frame &f) { ... });
//
// Opening the port resets the Arduino
// -----------------------------------
// Opening a USB serial port raises its DTR line, and on an Uno DTR is wired
// to the chip's reset, so the firmware restarts: the bootloader waits
// briefly for a new program, then the firmware sends Boot. Frames sent in
// that window are lost. On the bench Uno Boot arrives about 1.5 s after
// opening [ours, measured 2026-10-25], so a program must wait for Boot
// before driving.
//
// This is the Pi side only: it uses POSIX termios and poll, which the AVR
// does not have. It builds on Linux and macOS, so the same tool runs at the
// bench on either. Errors are std::system_error, so a caller can tell a
// missing device (no_such_file_or_directory) from a permissions problem
// (permission_denied: the user is not in the dialout group).

#ifndef HERON_MOTOR_MOTOR_PORT_H
#define HERON_MOTOR_MOTOR_PORT_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace Heron::Motor {

/** Where the Arduino is. The line settings are fixed by the protocol. */
struct MotorPortConfig {
  /** Never /dev/ttyUSB*: the Arduino and the LiDAR have identical USB
   *  bridges, so only the udev rule's port-based name is reliable. */
  std::string device{"/dev/arduino"};
};

/** The protocol's baud rate (docs/motor-protocol.md). */
inline constexpr std::uint32_t kMotorBaud = 115200;

/**
 * An open serial port to the motor controller, closed on destruction.
 *
 * Thread Safety: Not thread-safe. External synchronization required; one
 * thread reading while another writes is safe at the OS level, but this
 * class makes no promise about it.
 */
class MotorPort {
  public:
    /**
     * Opens and configures the port, and discards anything already queued.
     * Resets the Arduino; see the file header.
     *
     * @param config The device
     * @throws std::system_error if the device cannot be opened or configured;
     *         code() carries the errno
     */
    explicit MotorPort(const MotorPortConfig &config);
    ~MotorPort();

    MotorPort(const MotorPort &) = delete;
    MotorPort &operator=(const MotorPort &) = delete;

    /**
     * Waits up to timeout for bytes and reads what is available.
     *
     * @param buffer Where to put them
     * @param capacity Its size
     * @param timeout How long to wait; never busy-waits
     * @return Bytes read; 0 on timeout or when interrupted by a signal
     * @throws std::system_error if the device fails or goes away
     */
    std::size_t read(std::uint8_t *buffer, std::size_t capacity,
                     std::chrono::milliseconds timeout);

    /**
     * Writes all of bytes, waiting while the driver's buffer is full. A
     * frame is at most 66 bytes, so in practice this returns at once.
     *
     * @throws std::system_error if the device fails or goes away
     */
    void write(const std::uint8_t *bytes, std::size_t length);

    [[nodiscard]] const MotorPortConfig &getConfig() const noexcept { return m_config; }

  private:
    const MotorPortConfig m_config;
    int m_fd{-1};  // int: the POSIX descriptor type
};

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_MOTOR_PORT_H
