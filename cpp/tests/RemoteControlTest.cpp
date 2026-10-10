// File        : RemoteControlTest.cpp
// Author      : Chris Pane
// Description : RemoteControl end to end over a pseudo-terminal: start, the
//               reader thread, latest-wins publication, the stale rule, stop.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.
//
// The pty stands in for the UART: frames written to the master side arrive
// at the slave, which RemoteControl opens as its device. Line settings are
// ignored by a pty, so baud and stop bits are not exercised here. Timing
// margins are generous (several times the 20 ms read timeout) so a loaded
// host does not make the test flaky.

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include "heron/remote/RemoteControl.h"

namespace {

using namespace Heron::Remote;
using std::chrono::milliseconds;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

std::vector<std::uint8_t> makeFrame(std::uint16_t ch1) {
  std::vector<std::uint8_t> f = {kIBusHeader0, kIBusHeader1};
  for (std::size_t i = 0; i < kIBusChannelCount; ++i) {
    const std::uint16_t v = i == 0 ? ch1 : 1500;
    f.push_back(static_cast<std::uint8_t>(v & 0xFF));
    f.push_back(static_cast<std::uint8_t>(v >> 8));
  }
  std::uint32_t sum = 0;
  for (const std::uint8_t b : f) {
    sum += b;
  }
  const auto cs = static_cast<std::uint16_t>(0xFFFF - sum);
  f.push_back(static_cast<std::uint8_t>(cs & 0xFF));
  f.push_back(static_cast<std::uint8_t>(cs >> 8));
  return f;
}

/** A pseudo-terminal: write to master, the code under test reads slavePath. */
class Pty {
  public:
    Pty() {
      m_master = posix_openpt(O_RDWR | O_NOCTTY);
      if (m_master < 0 || grantpt(m_master) != 0 || unlockpt(m_master) != 0) {
        std::perror("posix_openpt");
        std::exit(2);
      }
      const char *name = ptsname(m_master);
      if (name == nullptr) {
        std::perror("ptsname");
        std::exit(2);
      }
      m_slavePath = name;
    }

    ~Pty() { closeMaster(); }

    Pty(const Pty &) = delete;
    Pty &operator=(const Pty &) = delete;

    void send(std::uint16_t ch1) {
      const std::vector<std::uint8_t> f = makeFrame(ch1);
      if (::write(m_master, f.data(), f.size()) != static_cast<ssize_t>(f.size())) {
        std::perror("write");
      }
    }

    void closeMaster() {
      if (m_master >= 0) {
        ::close(m_master);
        m_master = -1;
      }
    }

    [[nodiscard]] const std::string &getSlavePath() const { return m_slavePath; }

  private:
    int m_master{-1};
    std::string m_slavePath;
};

/** Streams frames at about the receiver's rate until destroyed. */
class Streamer {
  public:
    Streamer(Pty &pty, std::uint16_t ch1)
        : m_thread([&pty, ch1](std::stop_token stop) {
            while (!stop.stop_requested()) {
              pty.send(ch1);
              std::this_thread::sleep_for(milliseconds(8));
            }
          }) {}

  private:
    std::jthread m_thread;
};

}  // namespace

int main() {
  std::printf("Remote control\n");

  {
    ReceiverConfig config;
    config.serial.device = "/nonexistent/ibus";
    RemoteControl remote(config);
    check("missing device: start() fails", !remote.start());
    const RemoteHealth h = remote.getHealth();
    check("missing device: state FAILED", h.state == RemoteState::Failed);
    check("missing device: lastError names the device",
          h.lastError.find("/nonexistent/ibus") != std::string::npos);
    check("missing device: link NO SIGNAL", remote.getLatest().link == LinkState::NoSignal);
  }

  {
    Pty pty;
    ReceiverConfig config;
    config.serial.device = pty.getSlavePath();
    RemoteControl remote(config);
    check("pty: start() succeeds", remote.start());
    check("pty: state RUNNING", remote.getHealth().state == RemoteState::Running);
    check("pty: start() again is a no-op", remote.start());
    const RemoteSnapshot before = remote.getLatest();
    check("before any frame: NO SIGNAL, seq 0, not ok",
          before.link == LinkState::NoSignal && before.seq == 0 && !before.isOk());

    {
      Streamer streamer(pty, 1234);
      std::this_thread::sleep_for(milliseconds(200));
      const RemoteSnapshot live = remote.getLatest();
      check("streaming: link OK", live.isOk());
      check("streaming: newest frame published", live.frame.channels[0] == 1234 &&
                                                     live.frame.channels[1] == 1500);
      check("streaming: seq counts frames", live.seq >= 5);
      check("streaming: age is small", live.age < milliseconds(60));
      check("streaming: getLatest(now) agrees on an explicit time",
            remote.getLatest(Clock::now()).isOk());
      check("streaming: stats count the frames", remote.getHealth().stats.framesDecoded >= 5);
    }

    std::this_thread::sleep_for(milliseconds(250));
    const RemoteSnapshot quiet = remote.getLatest();
    check("frames stop: link STALE", quiet.link == LinkState::Stale);
    check("frames stop: last frame kept, age grows",
          quiet.frame.channels[0] == 1234 && quiet.age > milliseconds(100));
    check("frames stop: one stale event", remote.getHealth().staleEvents == 1);
    check("frames stop: still RUNNING (a quiet line is not an error)",
          remote.getHealth().state == RemoteState::Running);

    {
      Streamer streamer(pty, 1800);
      std::this_thread::sleep_for(milliseconds(150));
      check("frames resume: OK with the new value",
            remote.getLatest().isOk() && remote.getLatest().frame.channels[0] == 1800);
    }

    remote.stop();
    check("stop: state STOPPED", remote.getHealth().state == RemoteState::Stopped);
    remote.stop();
    check("stop: twice is harmless", remote.getHealth().state == RemoteState::Stopped);

    check("restart: start() succeeds", remote.start());
    check("restart: counters start again", remote.getLatest().seq == 0);
    {
      Streamer streamer(pty, 1100);
      std::this_thread::sleep_for(milliseconds(150));
      check("restart: frames flow again", remote.getLatest().frame.channels[0] == 1100);
    }
  }

  {
    // The device going away mid-run: closing the master hangs up the slave.
    Pty pty;
    ReceiverConfig config;
    config.serial.device = pty.getSlavePath();
    RemoteControl remote(config);
    remote.start();
    {
      Streamer streamer(pty, 1500);
      std::this_thread::sleep_for(milliseconds(100));
    }
    pty.closeMaster();
    std::this_thread::sleep_for(milliseconds(200));
    const RemoteHealth h = remote.getHealth();
    check("hangup: state FAILED with a reason", h.state == RemoteState::Failed &&
                                                    !h.lastError.empty());
    check("hangup: link goes STALE for consumers", remote.getLatest().link == LinkState::Stale);
    remote.stop();
    check("hangup: stop() after a failure keeps FAILED",
          remote.getHealth().state == RemoteState::Failed);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
