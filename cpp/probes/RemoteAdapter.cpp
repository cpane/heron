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
// was not OK at a poll. Switch the transmitter off mid-run to watch the link
// state; what that shows is the subject of docs/remote-ibus-protocol.md §5.

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
    if (now < nextReport) {
      continue;
    }
    nextReport += std::chrono::seconds(1);
    const RemoteHealth h = remote.getHealth();
    std::printf("%3.0f s  %-6s link %-9s age %5.0f ms  +%3" PRIu64
                " frames  CH1-6 %4u %4u %4u %4u %4u %4u  stale events %" PRIu64 "\n",
                std::chrono::duration<double>(now - begin).count(), toString(h.state),
                toString(s.link), std::chrono::duration<double, std::milli>(s.age).count(),
                s.seq - lastSeq, s.frame.channels[0], s.frame.channels[1], s.frame.channels[2],
                s.frame.channels[3], s.frame.channels[4], s.frame.channels[5], h.staleEvents);
    std::fflush(stdout);
    lastSeq = s.seq;
  }

  remote.stop();
  const RemoteHealth h = remote.getHealth();
  std::printf("\n%s. %" PRIu64 " polls, link not OK at %" PRIu64 "; %" PRIu64
              " frames, %" PRIu64 " checksum errors, %" PRIu64 " stale events, state %s%s%s\n",
              s_stopRequested != 0 ? "Stopped by signal" : "Finished", polls, notOk,
              h.stats.framesDecoded, h.stats.checksumErrors, h.staleEvents, toString(h.state),
              h.lastError.empty() ? "" : ": ", h.lastError.c_str());
  return 0;
}
