// File        : FailsafeTest.cpp
// Author      : Chris Pane
// Description : Radio-loss detection on a failsafe channel: match, tolerance
//               edges, the high nibble, and bad configurations.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <stdexcept>

#include "heron/remote/Failsafe.h"

namespace {

using namespace Heron::Remote;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool throwsInvalid(const std::function<void()> &fn) {
  try {
    fn();
  } catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

// Every channel at rest (1500), except the arm channel, CH7.
IBusFrame withCh7(std::uint16_t raw) {
  IBusFrame frame;
  frame.channels.fill(1500);
  frame.channels[6] = raw;
  return frame;
}

}  // namespace

int main() {
  // Our setup (docs/remote-ibus-protocol.md §5, run 4): SwA on CH7, 1000 up =
  // disarmed, 2000 down = armed, failsafe set to 1000.
  const FailsafeConfig arm{6, 1000, 50};

  std::printf("Match\n");

  {
    check("exact failsafe value is failsafe", isFailsafe(withCh7(1000), arm));
    check("armed (2000) is not failsafe", !isFailsafe(withCh7(2000), arm));
    check("centre (1500) is not failsafe", !isFailsafe(withCh7(1500), arm));
    check("only the configured channel counts", [&] {
      IBusFrame f = withCh7(2000);
      f.channels[2] = 1000;  // the throttle at its failsafe, CH3: not configured
      return !isFailsafe(f, arm);
    }());
    check("zero tolerance: exact only",
          isFailsafe(withCh7(1000), FailsafeConfig{6, 1000, 0}) &&
              !isFailsafe(withCh7(1001), FailsafeConfig{6, 1000, 0}));
    check("first and last slots are usable", [] {
      IBusFrame f = withCh7(2000);
      f.channels[0] = 1000;
      f.channels[13] = 1000;
      return isFailsafe(f, FailsafeConfig{0, 1000, 0}) &&
             isFailsafe(f, FailsafeConfig{13, 1000, 0});
    }());
  }

  std::printf("Tolerance edges\n");

  {
    check("value + tolerance is failsafe", isFailsafe(withCh7(1050), arm));
    check("value + tolerance + 1 is not", !isFailsafe(withCh7(1051), arm));
    check("value - tolerance is failsafe", isFailsafe(withCh7(950), arm));
    check("value - tolerance - 1 is not", !isFailsafe(withCh7(949), arm));
    check("tolerance wider than the value does not wrap",
          isFailsafe(withCh7(0), FailsafeConfig{6, 10, 0xFFFF}) &&
              isFailsafe(withCh7(0x0FFF), FailsafeConfig{6, 10, 0xFFFF}));
  }

  std::printf("High nibble\n");

  {
    // The IBusBM #24 report: 1000 with 0x2000 set arrives as 9192 (0x23E8).
    // Not seen on ours, but it must not read as anything but failsafe.
    check("0x2000 over the failsafe value is failsafe", isFailsafe(withCh7(0x23E8), arm));
    check("0x2000 over the armed value is failsafe", isFailsafe(withCh7(0x27D0), arm));
    check("0x1000 over the armed value is failsafe", isFailsafe(withCh7(0x17D0), arm));
    check("0xFFFF is failsafe", isFailsafe(withCh7(0xFFFF), arm));
    check("a high nibble on another channel does not count", [&] {
      IBusFrame f = withCh7(2000);
      f.channels[0] = 0x27D0;
      return !isFailsafe(f, arm);
    }());
  }

  std::printf("Configuration\n");

  {
    check("channel 14 rejected",
          throwsInvalid([] { (void)isFailsafe(withCh7(1000), FailsafeConfig{14, 1000, 50}); }));
    check("huge channel rejected", throwsInvalid([] {
            (void)isFailsafe(withCh7(1000), FailsafeConfig{static_cast<std::size_t>(-1), 1000, 50});
          }));
    check("value above 12 bits rejected",
          throwsInvalid([] { checkFailsafeConfig(FailsafeConfig{6, 0x1000, 50}); }));
    check("largest 12-bit value accepted",
          !throwsInvalid([] { checkFailsafeConfig(FailsafeConfig{6, 0x0FFF, 0}); }));
    check("our arm switch config accepted", !throwsInvalid([&] { checkFailsafeConfig(arm); }));
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
