// File        : LinuxI2cBus.cpp
// Author      : Chris Pane
// Description : I2cBus over Linux i2c-dev; on other systems, a constructor
//               that says so.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The transactions are those the probes use (cpp/probes/OtosBus.cpp), which
// probe O1 soaked: a register write and a read joined by a repeated start
// in one I2C_RDWR call, and every write as a single message
// (docs/odometry-otos-reference.md §3). The address goes in each message, so
// no I2C_SLAVE call is needed.

#include "I2cBus.h"

#include <array>
#include <cerrno>
#include <system_error>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace Heron::Odometry {

#if defined(__linux__)

LinuxI2cBus::LinuxI2cBus(const std::string &device, std::uint16_t address) : m_address(address) {
  m_fd = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (m_fd < 0) {
    throw std::system_error(errno, std::generic_category(),
                            "Heron::Odometry::LinuxI2cBus::LinuxI2cBus() : cannot open " + device);
  }
}

LinuxI2cBus::~LinuxI2cBus() {
  if (m_fd >= 0) {
    ::close(m_fd);
  }
}

std::error_code LinuxI2cBus::read(std::uint8_t reg, std::span<std::uint8_t> out) {
  i2c_msg messages[2];
  messages[0].addr = m_address;
  messages[0].flags = 0;
  messages[0].len = 1;
  messages[0].buf = &reg;
  messages[1].addr = m_address;
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<std::uint16_t>(out.size());
  messages[1].buf = out.data();

  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = messages;
  transfer.nmsgs = 2;
  if (::ioctl(m_fd, I2C_RDWR, &transfer) < 0) {
    return {errno, std::generic_category()};
  }
  return {};
}

std::error_code LinuxI2cBus::write(std::uint8_t reg, std::span<const std::uint8_t> data) {
  if (data.size() > kMaxWriteLength) {
    return std::make_error_code(std::errc::message_size);
  }
  std::array<std::uint8_t, kMaxWriteLength + 1> bytes{};
  bytes[0] = reg;
  for (std::size_t i = 0; i < data.size(); ++i) {
    bytes[i + 1] = data[i];
  }
  i2c_msg message;
  message.addr = m_address;
  message.flags = 0;
  message.len = static_cast<std::uint16_t>(data.size() + 1);
  message.buf = bytes.data();

  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = &message;
  transfer.nmsgs = 1;
  if (::ioctl(m_fd, I2C_RDWR, &transfer) < 0) {
    return {errno, std::generic_category()};
  }
  return {};
}

#else  // not Linux

LinuxI2cBus::LinuxI2cBus(const std::string &device, std::uint16_t address) : m_address(address) {
  throw std::system_error(std::make_error_code(std::errc::function_not_supported),
                          "Heron::Odometry::LinuxI2cBus::LinuxI2cBus() : " + device +
                              " needs Linux i2c-dev; this is not Linux");
}

LinuxI2cBus::~LinuxI2cBus() {
  // Never reached with a live object, since the constructor throws; the
  // fields are named here only so the build stays free of warnings.
  static_cast<void>(m_fd);
  static_cast<void>(m_address);
}

std::error_code LinuxI2cBus::read(std::uint8_t, std::span<std::uint8_t>) {
  return std::make_error_code(std::errc::function_not_supported);
}

std::error_code LinuxI2cBus::write(std::uint8_t, std::span<const std::uint8_t>) {
  return std::make_error_code(std::errc::function_not_supported);
}

#endif

}  // namespace Heron::Odometry
