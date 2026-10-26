// File        : DriveTest.cpp
// Author      : Chris Pane
// Description : Sticks to body velocity, mecanum mixing, and the arming rule.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Frames are built by hand with the channel values a stick would give. The
// mixing checks are the bench checks of docs/motor-design.md §4, written as
// numbers: each move must turn the right wheels the right way. Same style as
// the other tests: no framework.

#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "heron/drive/Drive.h"

namespace {

using namespace Heron::Drive;
using Heron::Remote::IBusFrame;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b) {
  return std::abs(a - b) < 1e-9;
}

bool wheelsAre(const WheelValues &m, double m1, double m2, double m3, double m4) {
  return near(m[0], m1) && near(m[1], m2) && near(m[2], m3) && near(m[3], m4);
}

// Sticks centred, throttle as given: CH1 right l-r, CH2 right u-d, CH3
// throttle, CH4 left l-r.
IBusFrame frame(std::uint16_t ch1, std::uint16_t ch2, std::uint16_t ch3, std::uint16_t ch4) {
  IBusFrame f;
  f.channels.fill(1500);
  f.channels[0] = ch1;
  f.channels[1] = ch2;
  f.channels[2] = ch3;
  f.channels[3] = ch4;
  return f;
}

constexpr std::uint16_t kFullThrottle = 2000;

}  // namespace

int main() {
  const StickConfig sticks;

  std::printf("Sticks to body velocity\n");

  {
    // Our transmitter: right stick up reads low on CH2 (measured).
    const BodyVelocity v = sticksToVelocity(frame(1500, 1000, kFullThrottle, 1500), sticks);
    check("right stick up (CH2 low): forward", near(v.vx, 1.0) && near(v.vy, 0.0) &&
                                                   near(v.wz, 0.0));
    const BodyVelocity b = sticksToVelocity(frame(1500, 2000, kFullThrottle, 1500), sticks);
    check("right stick down (CH2 high): backward", near(b.vx, -1.0));
    StickConfig other = sticks;
    other.forwardReadsLow = false;
    check("a transmitter whose up reads high: forwardReadsLow false",
          near(sticksToVelocity(frame(1500, 2000, kFullThrottle, 1500), other).vx, 1.0));
  }

  {
    const BodyVelocity r = sticksToVelocity(frame(2000, 1500, kFullThrottle, 1500), sticks);
    check("right stick right: vy negative (y is left)", near(r.vy, -1.0) && near(r.vx, 0.0));
    const BodyVelocity l = sticksToVelocity(frame(1000, 1500, kFullThrottle, 1500), sticks);
    check("right stick left: vy positive", near(l.vy, 1.0));
  }

  {
    const BodyVelocity r = sticksToVelocity(frame(1500, 1500, kFullThrottle, 2000), sticks);
    check("left stick right: wz negative (clockwise)", near(r.wz, -1.0));
    const BodyVelocity l = sticksToVelocity(frame(1500, 1500, kFullThrottle, 1000), sticks);
    check("left stick left: wz positive (anticlockwise)", near(l.wz, 1.0));
  }

  {
    const BodyVelocity half = sticksToVelocity(frame(1500, 1000, 1500, 1500), sticks);
    check("throttle half way: half the speed", near(half.vx, 0.5));
    const BodyVelocity none = sticksToVelocity(frame(1000, 1000, 1000, 2000), sticks);
    check("throttle at the bottom: no movement at all",
          near(none.vx, 0.0) && near(none.vy, 0.0) && near(none.wz, 0.0));
    check("throttle read 0..1", near(readThrottle(frame(1500, 1500, 1000, 1500), sticks), 0.0) &&
                                    near(readThrottle(frame(1500, 1500, 2000, 1500), sticks), 1.0));
  }

  {
    const BodyVelocity rest = sticksToVelocity(frame(1525, 1475, kFullThrottle, 1530), sticks);
    check("within 30 counts of centre: nothing (the dead zone)",
          near(rest.vx, 0.0) && near(rest.vy, 0.0) && near(rest.wz, 0.0));
    const BodyVelocity edge = sticksToVelocity(frame(1500, 1469, kFullThrottle, 1500), sticks);
    check("just past the dead zone: a small value, no jump",
          edge.vx > 0.0 && edge.vx < 0.01);
  }

  {
    StickConfig slow = sticks;
    slow.turnGain = 0.5;
    const BodyVelocity v = sticksToVelocity(frame(1500, 1500, kFullThrottle, 1000), slow);
    check("turn gain scales wz only", near(v.wz, 0.5));
  }

  {
    StickConfig bad = sticks;
    bad.turnChannel = 20;
    bool thrown = false;
    try {
      (void)sticksToVelocity(frame(1500, 1500, 1500, 1500), bad);
    } catch (const std::invalid_argument &) {
      thrown = true;
    }
    check("a channel slot out of range throws", thrown);
  }

  std::printf("Mixing: the bench checks, as numbers\n");

  check("forward: all four forward", wheelsAre(mix({1, 0, 0}), 1, 1, 1, 1));
  check("backward: all four backward", wheelsAre(mix({-1, 0, 0}), -1, -1, -1, -1));
  check("strafe left: 1 and 3 backward, 2 and 4 forward", wheelsAre(mix({0, 1, 0}), -1, 1, -1, 1));
  check("strafe right: the reverse", wheelsAre(mix({0, -1, 0}), 1, -1, 1, -1));
  check("anticlockwise: 1 and 4 backward, 2 and 3 forward",
        wheelsAre(mix({0, 0, 1}), -1, 1, 1, -1));
  check("clockwise: the reverse", wheelsAre(mix({0, 0, -1}), 1, -1, -1, 1));
  check("still: all zero", wheelsAre(mix({0, 0, 0}), 0, 0, 0, 0));
  check("half forward: half on every wheel", wheelsAre(mix({0.5, 0, 0}), 0.5, 0.5, 0.5, 0.5));

  {
    // Forward and left together asks front-right and back-left for 2.
    const WheelValues m = mix({1, 1, 0});
    check("diagonal, saturated: divided by the largest, direction kept",
          wheelsAre(m, 0, 1, 0, 1));
    const WheelValues all = mix({1, 1, 1});
    bool inRange = true;
    for (const double value : all) {
      inRange = inRange && std::abs(value) <= 1.0;
    }
    check("everything at once: every wheel within -1..1", inRange &&
                                                            near(std::abs(all[1]), 1.0));
    check("under full drive: not scaled", wheelsAre(mix({0.3, 0.2, 0.1}), 0.0, 0.6, 0.2, 0.4));
  }

  std::printf("Arming\n");

  {
    ArmGate gate;
    check("not OK: no driving", !gate.update(false, 0.0));
    check("armed with the throttle up: no driving", !gate.update(true, 0.6));
    check("still up: still none", !gate.update(true, 0.6));
    check("throttle brought down: ready", gate.update(true, 0.0) && gate.isReady());
    check("then up again: drives", gate.update(true, 0.8));
    check("disarmed, radio lost or stale: stops at once", !gate.update(false, 0.8));
    check("OK again with the throttle still up: not until it comes down",
          !gate.update(true, 0.8) && gate.update(true, 0.04));
    check("5% counts as the bottom, 6% does not",
          [] {
            ArmGate a;
            ArmGate b;
            return a.update(true, 0.05) && !b.update(true, 0.06);
          }());
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
