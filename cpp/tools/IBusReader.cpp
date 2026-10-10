// File        : IBusReader.cpp
// Author      : Chris Pane
// Description : Minimal FlySky iBUS reader for the Pi's UART. A hardware
//               check for the remote control receiver, built on heron_ibus.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> ibus_reader [/dev/serial0]
//
// Prints channels 1-6 ten times a second, or STALE when no valid frame has
// arrived for 100 ms. The protocol is in docs/remote-ibus-protocol.md.

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <exception>

#include "heron/remote/IBusParser.h"
#include "heron/remote/IBusPort.h"
#include "heron/remote/LinkMonitor.h"

int main(int argc, char **argv) {
  using namespace Heron::Remote;
  using namespace std::chrono;
  constexpr auto kPrintPeriod = milliseconds(100);

  try {
    SerialConfig config;
    if (argc > 1) {
      config.device = argv[1];
    }
    IBusPort port(config);
    IBusParser parser;
    LinkMonitor link;

    IBusFrame latest;
    auto lastPrint = Clock::now();
    std::array<std::uint8_t, 128> buf{};

    for (;;) {
      const std::size_t n = port.read(buf, milliseconds(20));
      parser.feed(std::span(buf.data(), n), Clock::now(), [&](const IBusFrame &frame) {
        latest = frame;
        link.onFrame(frame.stamp);
      });

      const auto now = Clock::now();
      if (now - lastPrint < kPrintPeriod) {
        continue;
      }
      lastPrint = now;

      if (!link.isOk(now)) {
        std::printf("STALE (no valid iBUS frame)\n");
      } else {
        const auto &ch = latest.channels;
        std::printf("CH1-6: %4u %4u %4u %4u %4u %4u\n", ch[0], ch[1], ch[2], ch[3], ch[4], ch[5]);
      }
      std::fflush(stdout);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
