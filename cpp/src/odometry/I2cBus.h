// File        : I2cBus.h
// Author      : Chris Pane
// Description : The I2C bus as the odometry library uses it: register reads
//               and writes to one device, and the Linux i2c-dev version.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// An interface so that OtosDevice and Odometry can be tested on the host
// against a fake bus: there is no host equivalent of the pseudo-terminal the
// remote's tests use, and linux/i2c-dev.h does not exist on macOS
// (docs/odometry-cpp-design.md §1).
//
// Transfers return a std::error_code rather than throwing: the reader thread
// counts a failed read and carries on, so a failure is an ordinary result
// there, not an exception.

#ifndef HERON_ODOMETRY_I2C_BUS_H
#define HERON_ODOMETRY_I2C_BUS_H

#include <cstdint>
#include <span>
#include <string>
#include <system_error>

namespace Heron::Odometry {

/**
 * One device on an I2C bus, addressed by register.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 */
class I2cBus {
  public:
    virtual ~I2cBus() = default;

    /**
     * Writes the register index, then reads out.size() bytes, as one
     * combined transaction (repeated start), so nothing can come between.
     *
     * @param reg The first register
     * @param out Where the bytes go; its size is the read length
     * @return Empty on success; the error otherwise, with out unspecified
     */
    [[nodiscard]] virtual std::error_code read(std::uint8_t reg, std::span<std::uint8_t> out) = 0;

    /**
     * Writes the register index and data in one transaction.
     *
     * @param reg The first register
     * @param data The bytes, written to reg, reg + 1, ...
     * @return Empty on success; the error otherwise
     */
    [[nodiscard]] virtual std::error_code write(std::uint8_t reg,
                                                std::span<const std::uint8_t> data) = 0;
};

/**
 * I2cBus over Linux i2c-dev, with the I2C_RDWR ioctl.
 *
 * On any other system the constructor throws: the class exists everywhere so
 * that Odometry builds everywhere, and reports the failure through start().
 */
class LinuxI2cBus : public I2cBus {
  public:
    /** The longest write: register index plus this many data bytes. */
    static constexpr std::size_t kMaxWriteLength = 32;

    /**
     * Opens the bus device.
     *
     * @param device The bus, such as /dev/i2c-1
     * @param address The device's 7-bit address
     * @throws std::system_error if the device cannot be opened, or this is
     *         not Linux
     */
    LinuxI2cBus(const std::string &device, std::uint16_t address);

    ~LinuxI2cBus() override;

    LinuxI2cBus(const LinuxI2cBus &) = delete;
    LinuxI2cBus &operator=(const LinuxI2cBus &) = delete;

    [[nodiscard]] std::error_code read(std::uint8_t reg, std::span<std::uint8_t> out) override;

    /** @return message_size if data is longer than kMaxWriteLength */
    [[nodiscard]] std::error_code write(std::uint8_t reg,
                                        std::span<const std::uint8_t> data) override;

  private:
    int m_fd{-1};
    std::uint16_t m_address{0};
};

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_I2C_BUS_H
