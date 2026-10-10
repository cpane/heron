// File        : IBusPort.cpp
// Author      : Chris Pane
// Description : Receive-only serial port for the iBUS receiver.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include "heron/remote/IBusPort.h"

#include <cerrno>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace Heron::Remote {

namespace {

speed_t toSpeed(std::uint32_t baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::invalid_argument("Heron::Remote::IBusPort::IBusPort() : unsupported baud rate " +
                                  std::to_string(baud));
  }
}

[[noreturn]] void throwErrno(const std::string &what) {
  throw std::system_error(errno, std::generic_category(), "Heron::Remote::IBusPort::" + what);
}

}  // namespace

IBusPort::IBusPort(const SerialConfig &config) : m_config(config) {
  const speed_t speed = toSpeed(config.baud);
  if (config.stopBits != 1 && config.stopBits != 2) {
    throw std::invalid_argument("Heron::Remote::IBusPort::IBusPort() : stop bits must be 1 or 2");
  }

  // O_RDONLY: this program never transmits on the receiver's line.
  m_fd = ::open(config.device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (m_fd < 0) {
    throwErrno("IBusPort() : cannot open " + config.device);
  }

  termios tio{};
  if (tcgetattr(m_fd, &tio) != 0) {
    const int saved = errno;
    ::close(m_fd);
    m_fd = -1;
    errno = saved;
    throwErrno("IBusPort() : tcgetattr failed on " + config.device);
  }
  cfmakeraw(&tio);  // no echo, no line editing, no translation
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;  // 8 data bits, no parity, ignore modem lines
  if (config.stopBits == 2) {
    tio.c_cflag |= CSTOPB;
  }
#ifdef CRTSCTS
  // cfmakeraw leaves hardware flow control as it found it; the receiver has
  // no RTS/CTS wiring, so a port left with it enabled would never deliver.
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
    const int saved = errno;
    ::close(m_fd);
    m_fd = -1;
    errno = saved;
    throwErrno("IBusPort() : tcsetattr failed on " + config.device);
  }
  tcflush(m_fd, TCIFLUSH);  // start from fresh bytes, not whatever queued before
}

IBusPort::~IBusPort() {
  if (m_fd >= 0) {
    ::close(m_fd);
  }
}

std::size_t IBusPort::read(std::span<std::uint8_t> buffer, std::chrono::milliseconds timeout) {
  pollfd pfd{m_fd, POLLIN, 0};
  const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;  // a signal: let the caller look at its stop flag
    }
    throwErrno("read() : poll failed on " + m_config.device);
  }
  if (ready == 0) {
    return 0;
  }
  if ((pfd.revents & POLLIN) == 0) {
    // POLLERR, POLLHUP or POLLNVAL with nothing to read: the device is gone.
    errno = EIO;
    throwErrno("read() : device error or hangup on " + m_config.device);
  }

  const ssize_t n = ::read(m_fd, buffer.data(), buffer.size());
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throwErrno("read() : read failed on " + m_config.device);
  }
  if (n == 0) {
    // poll() said readable and there was nothing: end of file, which is how
    // a vanished device (a hung-up pty, an unplugged USB adapter) shows up.
    // Returning 0 here would turn the caller's loop into a busy loop.
    errno = EIO;
    throwErrno("read() : device hung up: " + m_config.device);
  }
  return static_cast<std::size_t>(n);
}

}  // namespace Heron::Remote
