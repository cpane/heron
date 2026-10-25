// File        : MotorPort.cpp
// Author      : Chris Pane
// Description : The Pi's serial port to the Arduino motor controller.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Set up like IBusPort (src/remote/), but opened for reading and writing.
// The two are kept apart on purpose: the iBUS port must never be able to
// transmit, and this one must.

#include "heron/motor/MotorPort.h"

#include <cerrno>
#include <system_error>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace Heron::Motor {

namespace {

[[noreturn]] void throwErrno(const std::string &what) {
  throw std::system_error(errno, std::generic_category(), "Heron::Motor::MotorPort::" + what);
}

}  // namespace

MotorPort::MotorPort(const MotorPortConfig &config) : m_config(config) {
  m_fd = ::open(config.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (m_fd < 0) {
    throwErrno("MotorPort() : cannot open " + config.device);
  }

  termios tio{};
  if (tcgetattr(m_fd, &tio) != 0) {
    const int saved = errno;
    ::close(m_fd);
    m_fd = -1;
    errno = saved;
    throwErrno("MotorPort() : tcgetattr failed on " + config.device);
  }
  cfmakeraw(&tio);  // no echo, no line editing, no translation
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;  // 8N1, ignore modem status lines
#ifdef CRTSCTS
  // The Uno has no RTS/CTS; a port left with flow control on would stall.
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);
#endif
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
    const int saved = errno;
    ::close(m_fd);
    m_fd = -1;
    errno = saved;
    throwErrno("MotorPort() : tcsetattr failed on " + config.device);
  }
  tcflush(m_fd, TCIOFLUSH);  // start from fresh bytes, not whatever queued before
}

MotorPort::~MotorPort() {
  if (m_fd >= 0) {
    ::close(m_fd);
  }
}

std::size_t MotorPort::read(std::uint8_t *buffer, std::size_t capacity,
                            std::chrono::milliseconds timeout) {
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

  const ssize_t n = ::read(m_fd, buffer, capacity);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throwErrno("read() : read failed on " + m_config.device);
  }
  if (n == 0) {
    // Readable yet empty: end of file, which is how an unplugged USB adapter
    // shows up. Returning 0 would turn the caller's loop into a busy loop.
    errno = EIO;
    throwErrno("read() : device hung up: " + m_config.device);
  }
  return static_cast<std::size_t>(n);
}

void MotorPort::write(const std::uint8_t *bytes, std::size_t length) {
  std::size_t done = 0;
  while (done < length) {
    const ssize_t n = ::write(m_fd, bytes + done, length - done);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN) {
        // The port is non-blocking and the driver's buffer is full: wait
        // until it can take more rather than spinning.
        pollfd pfd{m_fd, POLLOUT, 0};
        if (::poll(&pfd, 1, 100) < 0 && errno != EINTR) {
          throwErrno("write() : poll failed on " + m_config.device);
        }
        continue;
      }
      throwErrno("write() : write failed on " + m_config.device);
    }
    done += static_cast<std::size_t>(n);
  }
}

}  // namespace Heron::Motor
