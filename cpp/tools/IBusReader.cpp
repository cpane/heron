// File        : IBusReader.cpp
// Author      : Chris Pane
// Description : Minimal FlySky iBUS reader for the Pi's UART. A hardware
//               check for the remote control receiver, not a library.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> ibus_reader [/dev/serial0]
//
// Prints channels 1-6 ten times a second, or STALE when no valid frame has
// arrived for 100 ms. iBUS is 115200 8N1; each frame is 32 bytes: 0x20 0x40,
// fourteen little-endian 16-bit channels (~1000..2000), then a checksum of
// 0xFFFF minus the sum of the first 30 bytes.
//
// Uses POSIX termios and poll, which the standard library does not provide.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;

const char *kDefaultDevice = "/dev/serial0";
constexpr std::size_t kChannelCount = 14;
constexpr std::size_t kFrameLen = 32;
constexpr std::size_t kChecksumOffset = 30;

struct IBusFrame {
  std::array<uint16_t, kChannelCount> ch{};  // raw channel values, ~1000..2000
  Clock::time_point stamp{};                 // when this frame was decoded
};

class IBusReader {
  public:
    explicit IBusReader(std::string device) : m_device(std::move(device)) {}

    ~IBusReader() {
      if (m_fd >= 0) {
        ::close(m_fd);
      }
    }

    IBusReader(const IBusReader &) = delete;
    IBusReader &operator=(const IBusReader &) = delete;

    bool open() {
      m_fd = ::open(m_device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
      if (m_fd < 0) {
        std::perror("open");
        return false;
      }

      termios tio{};
      if (tcgetattr(m_fd, &tio) != 0) {
        std::perror("tcgetattr");
        return false;
      }
      cfmakeraw(&tio);  // no echo, no line editing, no translation
      cfsetispeed(&tio, B115200);
      cfsetospeed(&tio, B115200);
      tio.c_cflag &= ~static_cast<tcflag_t>(PARENB | CSTOPB | CSIZE);
      tio.c_cflag |= CS8 | CLOCAL | CREAD;  // 8N1, ignore modem lines
      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;
      if (tcsetattr(m_fd, TCSANOW, &tio) != 0) {
        std::perror("tcsetattr");
        return false;
      }
      tcflush(m_fd, TCIOFLUSH);
      return true;
    }

    // Waits up to timeoutMs for bytes. Returns true if at least one new valid
    // frame was decoded; 'out' then holds the most recent one.
    bool waitFrame(IBusFrame &out, int timeoutMs) {
      pollfd pfd{m_fd, POLLIN, 0};
      if (::poll(&pfd, 1, timeoutMs) <= 0 || (pfd.revents & POLLIN) == 0) {
        return false;
      }

      uint8_t tmp[128];
      ssize_t n = ::read(m_fd, tmp, sizeof tmp);
      if (n <= 0) {
        return false;
      }
      m_buf.insert(m_buf.end(), tmp, tmp + n);

      bool got = false;
      while (decodeOne(out)) {
        got = true;
      }
      return got;
    }

  private:
    static uint16_t le16(uint8_t lo, uint8_t hi) {
      return static_cast<uint16_t>(lo | (hi << 8));
    }

    // Tries to decode one frame from the front of the buffer, resyncing on junk.
    bool decodeOne(IBusFrame &out) {
      while (m_buf.size() >= kFrameLen) {
        if (m_buf[0] != 0x20 || m_buf[1] != 0x40) {
          m_buf.erase(m_buf.begin());
          continue;
        }

        uint16_t sum = 0xFFFF;
        for (std::size_t i = 0; i < kChecksumOffset; ++i) {
          sum = static_cast<uint16_t>(sum - m_buf[i]);
        }
        uint16_t rx = le16(m_buf[kChecksumOffset], m_buf[kChecksumOffset + 1]);
        if (sum != rx) {
          m_buf.erase(m_buf.begin());  // bad checksum, keep hunting
          continue;
        }

        for (std::size_t i = 0; i < kChannelCount; ++i) {
          out.ch[i] = le16(m_buf[2 + 2 * i], m_buf[3 + 2 * i]);
        }
        out.stamp = Clock::now();
        m_buf.erase(m_buf.begin(), m_buf.begin() + kFrameLen);
        return true;
      }
      return false;
    }

    std::string m_device;
    int m_fd = -1;
    std::vector<uint8_t> m_buf;
};

}  // namespace

int main(int argc, char **argv) {
  IBusReader rx(argc > 1 ? argv[1] : kDefaultDevice);
  if (!rx.open()) {
    return 1;
  }

  using namespace std::chrono;
  constexpr auto kStale = milliseconds(100);  // no valid frame for this long = link lost
  constexpr auto kPrintPeriod = milliseconds(100);

  IBusFrame f;
  auto lastGood = Clock::now();
  auto lastPrint = Clock::now();
  bool haveFrame = false;

  for (;;) {
    if (rx.waitFrame(f, 20)) {
      lastGood = f.stamp;
      haveFrame = true;
    }

    auto now = Clock::now();
    if (now - lastPrint < kPrintPeriod) {
      continue;
    }
    lastPrint = now;

    if (!haveFrame || now - lastGood > kStale) {
      std::printf("STALE (no valid iBUS frame)\n");
    } else {
      std::printf("CH1-6: %4u %4u %4u %4u %4u %4u\n",
                  f.ch[0], f.ch[1], f.ch[2], f.ch[3], f.ch[4], f.ch[5]);
    }
    std::fflush(stdout);
  }
}
