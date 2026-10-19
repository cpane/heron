// File        : FakeOtosBus.h
// Author      : Chris Pane
// Description : An I2cBus that behaves like an OTOS, for tests: registers,
//               self-clearing calibration and reset, and injected failures.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Modelled on what the reference says the board does
// (docs/odometry-otos-reference.md §3, §4, §6): 0x06 counts down after a
// calibration write, 0x07 clears after a reset, the offset always reads back
// 0, and reads past 0x5F return 0. The counting is per poll rather than per
// millisecond, so tests run fast.
//
// Thread-safe for one reader thread: read() and write() take `mutex`, and a
// test that changes registers or switches while Odometry is running must
// hold it too.

#ifndef HERON_TESTS_FAKE_OTOS_BUS_H
#define HERON_TESTS_FAKE_OTOS_BUS_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <system_error>
#include <vector>

#include "I2cBus.h"

namespace Heron::Odometry {

class FakeOtosBus : public I2cBus {
  public:
    struct Write {
      std::uint8_t reg;
      std::vector<std::uint8_t> data;
    };

    FakeOtosBus() {
      m_regs[0x00] = 0x5F;
      m_regs[0x01] = 0x11;
      m_regs[0x02] = 0x10;
    }

    std::error_code read(std::uint8_t reg, std::span<std::uint8_t> out) override {
      const std::lock_guard<std::mutex> lock(mutex);
      ++reads;
      if (failReadsFrom >= 0 && reads > static_cast<std::size_t>(failReadsFrom)) {
        return std::make_error_code(std::errc::io_error);
      }
      if (failRegister >= 0 && reg == failRegister) {
        return std::make_error_code(std::errc::io_error);
      }
      // A poll of a self-clearing register moves it on.
      if (reg == 0x06 && m_regs[0x06] != 0 && !calibrationStuck) {
        m_regs[0x06] = m_regs[0x06] > calibrationStep ? m_regs[0x06] - calibrationStep : 0;
      }
      if (reg == 0x07) {
        m_regs[0x07] = 0;
      }
      for (std::size_t i = 0; i < out.size(); ++i) {
        const std::size_t at = reg + i;
        std::uint8_t value = at < m_regs.size() ? m_regs[at] : 0;
        if (at >= 0x10 && at <= 0x15) {
          value = 0;  // the offset never reads back (reference §6)
        }
        if (corruptScalars && (at == 0x04 || at == 0x05)) {
          value = static_cast<std::uint8_t>(value + 1);
        }
        out[i] = value;
      }
      return {};
    }

    std::error_code write(std::uint8_t reg, std::span<const std::uint8_t> data) override {
      const std::lock_guard<std::mutex> lock(mutex);
      writes.push_back(Write{reg, std::vector<std::uint8_t>(data.begin(), data.end())});
      if (failRegister >= 0 && reg == failRegister) {
        return std::make_error_code(std::errc::io_error);
      }
      for (std::size_t i = 0; i < data.size() && reg + i < m_regs.size(); ++i) {
        m_regs[reg + i] = data[i];
      }
      if (reg == 0x07 && !data.empty() && (data[0] & 0x01) != 0) {
        resets += 1;
      }
      return {};
    }

    /** Sets a register directly, as the board's own updates would. */
    void set(std::uint8_t reg, std::uint8_t value) { m_regs[reg] = value; }

    /** Puts a signed little-endian word at reg, reg + 1. */
    void setWord(std::uint8_t reg, std::int16_t value) {
      const auto u = static_cast<std::uint16_t>(value);
      m_regs[reg] = static_cast<std::uint8_t>(u & 0xFF);
      m_regs[reg + 1] = static_cast<std::uint8_t>(u >> 8);
    }

    // Behaviour switches and records, open to the tests; hold `mutex` to
    // touch them, or the registers, while a reader thread runs.
    std::mutex mutex;
    std::uint8_t calibrationStep{64};   // 255 clears in 4 polls
    bool calibrationStuck{false};
    bool corruptScalars{false};
    int failRegister{-1};               // every transfer touching it fails
    long failReadsFrom{-1};             // reads after this many fail; -1 never
    std::size_t reads{0};
    std::size_t resets{0};
    std::vector<Write> writes;

  private:
    std::array<std::uint8_t, 0x60> m_regs{};
};

}  // namespace Heron::Odometry

#endif  // HERON_TESTS_FAKE_OTOS_BUS_H
