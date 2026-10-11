// File        : RemoteAdapter.cpp
// Author      : Chris Pane
// Description : Runs RemoteControl live and reports what a robot loop would
//               see, once a second.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> remote_adapter 20
//     scripts/deploy_cpp.sh <user>@<pi> remote_adapter 20 /dev/serial0
//
// The remote control's counterpart of lidar_adapter. Uses only
// RemoteControl.h and links only heron_remote, as robot code will. Polls
// getLatest() at 50 Hz, like a control loop, and counts how often the link
// was not OK at a poll, and how often it was in failsafe.
//
// The failsafe channel is our setup's arm switch: SwA on CH7, failing to
// 1000 (disarmed) on the transmitter (docs/remote-ibus-protocol.md §5, run
// 4). Switch the transmitter off mid-run: the link stays OK, because the
// receiver keeps sending, and failsafe goes YES about 1 s later. Flipping
// SwA up reads the same, by design. If neither shows, check the
// transmitter's Failsafe menu: run 3 found the setting lost.

#include <chrono>
#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

#include "heron/remote/RemoteControl.h"

namespace {

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

}  // namespace

int main(int argc, char **argv) {
  using namespace Heron::Remote;
  using std::chrono::milliseconds;

  const std::int32_t seconds = (argc > 1) ? static_cast<std::int32_t>(std::atoi(argv[1])) : 20;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  std::printf("========================================\n");
  std::printf(" RemoteControl, live\n");
  std::printf("========================================\n\n");

  ReceiverConfig config;  // /dev/serial0, 115200 8N1, stale after 100 ms
  config.failsafe = FailsafeConfig{6, 1000, 50};  // CH7 (SwA) at 1000 +- 50
  if (argc > 2) {
    config.serial.device = argv[2];
  }
  RemoteControl remote(config);
  if (!remote.start()) {
    std::fprintf(stderr, "ERROR: %s\n", remote.getHealth().lastError.c_str());
    return 1;
  }

  constexpr milliseconds kLoop(20);  // a 50 Hz control loop
  std::uint64_t polls = 0;
  std::uint64_t notOk = 0;
  std::uint64_t inFailsafe = 0;
  std::uint64_t lastSeq = 0;
  const auto begin = Clock::now();
  auto nextReport = begin + std::chrono::seconds(1);
  while (s_stopRequested == 0 && Clock::now() - begin < std::chrono::seconds(seconds)) {
    std::this_thread::sleep_for(kLoop);
    const auto now = Clock::now();
    const RemoteSnapshot s = remote.getLatest(now);
    ++polls;
    if (!s.isOk()) {
      ++notOk;
    }
    if (s.failsafe) {
      ++inFailsafe;
    }
    if (now < nextReport) {
      continue;
    }
    nextReport += std::chrono::seconds(1);
    const RemoteHealth h = remote.getHealth();
    std::printf("%3.0f s  %-6s link %-9s failsafe %-3s age %5.0f ms  +%3" PRIu64
                " frames  CH1-6 %4u %4u %4u %4u %4u %4u  CH7 %4u  events: stale %" PRIu64
                " failsafe %" PRIu64 "\n",
                std::chrono::duration<double>(now - begin).count(), toString(h.state),
                toString(s.link), s.failsafe ? "YES" : "no",
                std::chrono::duration<double, std::milli>(s.age).count(), s.seq - lastSeq,
                s.frame.channels[0], s.frame.channels[1], s.frame.channels[2],
                s.frame.channels[3], s.frame.channels[4], s.frame.channels[5],
                s.frame.channels[6], h.staleEvents, h.failsafeEvents);
    std::fflush(stdout);
    lastSeq = s.seq;
  }

  remote.stop();
  const RemoteHealth h = remote.getHealth();
  std::printf("\n%s. %" PRIu64 " polls, not OK at %" PRIu64 ", failsafe at %" PRIu64
              "; %" PRIu64 " frames, %" PRIu64 " checksum errors, %" PRIu64
              " stale events, %" PRIu64 " failsafe events, state %s%s%s\n",
              s_stopRequested != 0 ? "Stopped by signal" : "Finished", polls, notOk, inFailsafe,
              h.stats.framesDecoded, h.stats.checksumErrors, h.staleEvents, h.failsafeEvents,
              toString(h.state), h.lastError.empty() ? "" : ": ", h.lastError.c_str());
  return 0;
}
