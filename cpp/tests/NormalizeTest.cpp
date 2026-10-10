// File        : NormalizeTest.cpp
// Author      : Chris Pane
// Description : Edge cases of the channel normalization helpers: center,
//               deadband boundaries, clamping and switch thresholds.
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
#include <string_view>

#include "heron/remote/Normalize.h"

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

}  // namespace

int main() {
  std::printf("Axis\n");

  {
    const AxisConfig plain{};
    check("plain: center is 0", normalizeAxis(1500, plain) == 0);
    check("plain: max endpoint is +1000", normalizeAxis(2000, plain) == 1000);
    check("plain: min endpoint is -1000", normalizeAxis(1000, plain) == -1000);
    check("plain: one count is 2 thousandths", normalizeAxis(1501, plain) == 2);
    check("plain: halfway is +500", normalizeAxis(1750, plain) == 500);
    check("plain: symmetric", normalizeAxis(1250, plain) == -500);
    check("plain: clamps above", normalizeAxis(2100, plain) == 1000);
    check("plain: clamps below", normalizeAxis(900, plain) == -1000);
    check("plain: clamps 0", normalizeAxis(0, plain) == -1000);
    check("plain: clamps 0xFFFF", normalizeAxis(0xFFFF, plain) == 1000);
  }

  {
    AxisConfig dead{};
    dead.deadband = 20;
    check("deadband: center is 0", normalizeAxis(1500, dead) == 0);
    check("deadband: +edge is still 0", normalizeAxis(1520, dead) == 0);
    check("deadband: -edge is still 0", normalizeAxis(1480, dead) == 0);
    // Span past the deadband is 480, so one count is 1000/480 = 2.08 -> 2.
    check("deadband: one past +edge is small, not a step", normalizeAxis(1521, dead) == 2);
    check("deadband: one past -edge mirrors it", normalizeAxis(1479, dead) == -2);
    check("deadband: still reaches +1000 at the endpoint", normalizeAxis(2000, dead) == 1000);
    check("deadband: still reaches -1000 at the endpoint", normalizeAxis(1000, dead) == -1000);
    check("deadband: halfway through the live span is +500", normalizeAxis(1760, dead) == 500);
  }

  {
    // An off-centre center, as a trim produces: each side scales on its own.
    AxisConfig trimmed{};
    trimmed.center = 1520;
    check("trimmed: new center is 0", normalizeAxis(1520, trimmed) == 0);
    check("trimmed: 1500 is now slightly negative", normalizeAxis(1500, trimmed) == -38);
    check("trimmed: still reaches +1000", normalizeAxis(2000, trimmed) == 1000);
    check("trimmed: still reaches -1000", normalizeAxis(1000, trimmed) == -1000);
  }

  {
    // Endpoints narrower than the stick travel: clamps at them.
    AxisConfig narrow{};
    narrow.minRaw = 1100;
    narrow.maxRaw = 1900;
    check("narrow: full scale at maxRaw", normalizeAxis(1900, narrow) == 1000);
    check("narrow: clamps past maxRaw", normalizeAxis(2000, narrow) == 1000);
    check("narrow: clamps past minRaw", normalizeAxis(1000, narrow) == -1000);
  }

  {
    AxisConfig bad{};
    bad.center = 1000;
    check("invalid: center at minRaw rejected",
          throwsInvalid([&] { (void)normalizeAxis(1500, bad); }));
    bad = AxisConfig{};
    bad.deadband = -1;
    check("invalid: negative deadband rejected",
          throwsInvalid([&] { (void)normalizeAxis(1500, bad); }));
    bad = AxisConfig{};
    bad.deadband = 500;
    check("invalid: deadband as wide as a half rejected",
          throwsInvalid([&] { (void)normalizeAxis(1500, bad); }));
    bad = AxisConfig{};
    bad.deadband = 499;
    check("valid: deadband one short of a half accepted",
          !throwsInvalid([&] { (void)normalizeAxis(1500, bad); }) &&
              normalizeAxis(2000, bad) == 1000 && normalizeAxis(1999, bad) == 0);
  }

  std::printf("Knob\n");

  {
    check("knob: min is 0", normalizeKnob(1000) == 0);
    check("knob: max is 1000", normalizeKnob(2000) == 1000);
    check("knob: middle is 500", normalizeKnob(1500) == 500);
    check("knob: clamps below", normalizeKnob(0) == 0);
    check("knob: clamps above", normalizeKnob(0xFFFF) == 1000);
    check("knob: custom range", normalizeKnob(1550, 1100, 2000) == 500);
    check("knob: empty range rejected", throwsInvalid([] { (void)normalizeKnob(1500, 1500, 1500); }));
  }

  std::printf("Switch\n");

  {
    check("switch: 1000 is LOW", decodeSwitch3(1000) == Switch3::Low);
    check("switch: 1500 is MID", decodeSwitch3(1500) == Switch3::Mid);
    check("switch: 2000 is HIGH", decodeSwitch3(2000) == Switch3::High);
    check("switch: 1249 is LOW", decodeSwitch3(1249) == Switch3::Low);
    check("switch: 1250 is MID (boundary inclusive)", decodeSwitch3(1250) == Switch3::Mid);
    check("switch: 1750 is MID (boundary inclusive)", decodeSwitch3(1750) == Switch3::Mid);
    check("switch: 1751 is HIGH", decodeSwitch3(1751) == Switch3::High);
    check("switch: 0 is LOW", decodeSwitch3(0) == Switch3::Low);
    check("switch: 0xFFFF is HIGH", decodeSwitch3(0xFFFF) == Switch3::High);
    check("switch: names", std::string_view(toString(Switch3::Mid)) == "MID");
    check("switch: crossed thresholds rejected",
          throwsInvalid([] { (void)decodeSwitch3(1500, 1800, 1200); }));
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
