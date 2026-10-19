// File        : OtosCodecTest.cpp
// Author      : Chris Pane
// Description : The OTOS codec at its edges: scalings and signs of every
//               snapshot block, scalar and offset encoding, angle wrapping.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <numbers>
#include <stdexcept>

#include "OtosCodec.h"

namespace {

using namespace Heron::Odometry;

constexpr double kPi = std::numbers::pi;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b, double tolerance = 1e-9) {
  return std::fabs(a - b) <= tolerance;
}

bool throwsInvalid(const std::function<void()> &fn) {
  try {
    fn();
  } catch (const std::invalid_argument &) {
    return true;
  }
  return false;
}

using Snapshot = std::array<std::uint8_t, kSnapshotLength>;

// Puts a raw int16 word at word index `word` (0-17) after the status byte.
void putWord(Snapshot &bytes, std::size_t word, std::int16_t value) {
  const auto u = static_cast<std::uint16_t>(value);
  bytes[1 + 2 * word] = static_cast<std::uint8_t>(u & 0xFF);
  bytes[2 + 2 * word] = static_cast<std::uint8_t>(u >> 8);
}

std::int16_t getWord(const std::array<std::uint8_t, kOffsetLength> &bytes, std::size_t word) {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(bytes[2 * word]) |
                                   (static_cast<std::uint16_t>(bytes[2 * word + 1]) << 8));
}

}  // namespace

int main() {
  std::printf("Snapshot\n");

  {
    Snapshot zero{};
    const OtosSnapshot s = decodeSnapshot(zero);
    check("all zero decodes to zero", s.status == 0 && s.pose.x == 0.0 && s.pose.h == 0.0 &&
                                          s.accelerationStdDev.h == 0.0);
  }

  {
    // Full scale on every word: +32767 and -32768 in turn, block by block.
    Snapshot b{};
    b[0] = 0xC3;
    putWord(b, 0, 32767);    // pose x
    putWord(b, 1, -32768);   // pose y
    putWord(b, 2, -32768);   // pose h
    putWord(b, 3, 16384);    // velocity x
    putWord(b, 4, -16384);   // velocity y
    putWord(b, 5, 16384);    // velocity h
    putWord(b, 6, 2048);     // acceleration x
    putWord(b, 7, -2048);    // acceleration y
    putWord(b, 8, 32);       // acceleration h
    putWord(b, 9, 1);        // pose std dev x
    putWord(b, 10, 3277);    // pose std dev y
    putWord(b, 11, 1);       // pose std dev h
    putWord(b, 12, 6554);    // velocity std dev x
    putWord(b, 13, 0);
    putWord(b, 14, 1);       // velocity std dev h
    putWord(b, 15, 2048);    // acceleration std dev x
    putWord(b, 16, 0);
    putWord(b, 17, 1);       // acceleration std dev h
    const OtosSnapshot s = decodeSnapshot(b);

    check("status byte passes through", s.status == 0xC3);
    check("pose x: +32767 is just under +10 m", near(s.pose.x, 32767.0 * 10.0 / 32768.0));
    check("pose y: -32768 is -10 m", near(s.pose.y, -10.0));
    check("pose h: -32768 is -pi", near(s.pose.h, -kPi));
    check("velocity x: 16384 is +2.5 m/s", near(s.velocity.x, 2.5));
    check("velocity y: -16384 is -2.5 m/s", near(s.velocity.y, -2.5));
    check("velocity h: 16384 is +1000 deg/s", near(s.velocity.h, 1000.0 * kPi / 180.0));
    check("acceleration x: 2048 is +1 g", near(s.acceleration.x, 9.80665));
    check("acceleration y: -2048 is -1 g", near(s.acceleration.y, -9.80665));
    check("acceleration h: 32 LSB", near(s.acceleration.h, 32.0 * 1000.0 * kPi / 32768.0));
    check("pose std dev x: one LSB, about 0.3 mm", near(s.poseStdDev.x, 10.0 / 32768.0));
    check("pose std dev y: 3277 is about 1 m", near(s.poseStdDev.y, 3277.0 * 10.0 / 32768.0));
    check("pose std dev h: one LSB of heading", near(s.poseStdDev.h, kPi / 32768.0));
    check("velocity std dev x: velocity scaling", near(s.velocityStdDev.x, 6554.0 * 5.0 / 32768.0));
    check("velocity std dev h: one LSB of rate",
          near(s.velocityStdDev.h, 2000.0 * kPi / 180.0 / 32768.0));
    check("acceleration std dev x: 1 g", near(s.accelerationStdDev.x, 9.80665));
    check("acceleration std dev h: one LSB", near(s.accelerationStdDev.h, 1000.0 * kPi / 32768.0));
  }

  {
    // Byte order: 0x34 0x12 is 0x1234, not 0x3412.
    Snapshot b{};
    b[1] = 0x34;
    b[2] = 0x12;
    check("little-endian", near(decodeSnapshot(b).pose.x, 0x1234 * 10.0 / 32768.0));
    b[1] = 0xFF;
    b[2] = 0xFF;
    check("0xFFFF is -1 LSB", near(decodeSnapshot(b).pose.x, -10.0 / 32768.0));
  }

  std::printf("Version\n");
  {
    const OtosVersion v = decodeVersion(0x11);
    check("0x11 is 1.1", v.major == 1 && v.minor == 1);
    const OtosVersion w = decodeVersion(0x2A);
    check("0x2A is 2.10", w.major == 2 && w.minor == 10);
  }

  std::printf("Scalars\n");
  {
    check("1.0 encodes as 0", encodeScalar(1.0) == 0x00);
    check("1.001 is +1", encodeScalar(1.001) == 0x01);
    check("0.999 is -1 (0xFF)", encodeScalar(0.999) == 0xFF);
    check("1.127 is +127", encodeScalar(1.127) == 0x7F);
    check("0.872 is -128 (0x80)", encodeScalar(0.872) == 0x80);
    // Rounding, not truncation: 0.9934 is -6.6 steps, which truncates to -6.
    check("0.9934 rounds to -7, not -6", encodeScalar(0.9934) == static_cast<std::uint8_t>(-7));
    check("0.9964 rounds to -4", encodeScalar(0.9964) == static_cast<std::uint8_t>(-4));
    check("1.0046 rounds to +5", encodeScalar(1.0046) == 0x05);
    check("1.1274 still rounds into range", encodeScalar(1.1274) == 0x7F);
    check("1.128 is out of range", throwsInvalid([] { (void)encodeScalar(1.128); }));
    check("0.871 is out of range", throwsInvalid([] { (void)encodeScalar(0.871); }));
    check("NaN is rejected",
          throwsInvalid([] { (void)encodeScalar(std::numeric_limits<double>::quiet_NaN()); }));
    check("infinity is rejected",
          throwsInvalid([] { (void)encodeScalar(std::numeric_limits<double>::infinity()); }));
    check("decode 0 is 1.0", near(decodeScalar(0x00), 1.0));
    check("decode 0x80 is 0.872", near(decodeScalar(0x80), 0.872));
    check("decode 0x7F is 1.127", near(decodeScalar(0x7F), 1.127));
    bool roundTrip = true;
    for (std::int32_t raw = 0; raw < 256; ++raw) {
      const auto byte = static_cast<std::uint8_t>(raw);
      roundTrip = roundTrip && encodeScalar(decodeScalar(byte)) == byte;
    }
    check("every byte round-trips", roundTrip);
  }

  std::printf("Offset\n");
  {
    const auto zero = encodeOffset(Planar{});
    check("zero offset is six zero bytes", zero == std::array<std::uint8_t, kOffsetLength>{});

    const auto o = encodeOffset(Planar{0.1, -0.05, kPi / 2.0});
    check("x 0.1 m is 328 LSB (327.68 rounded)", getWord(o, 0) == 328);
    check("y -0.05 m is -164 LSB (-163.84 rounded)", getWord(o, 1) == -164);
    check("h pi/2 is 16384", getWord(o, 2) == 16384);
    check("bytes are little-endian", o[0] == 0x48 && o[1] == 0x01);

    check("h -pi is -32768", getWord(encodeOffset(Planar{0.0, 0.0, -kPi}), 2) == -32768);
    check("h +pi wraps to -32768", getWord(encodeOffset(Planar{0.0, 0.0, kPi}), 2) == -32768);
    check("h 3pi/2 wraps to -pi/2",
          getWord(encodeOffset(Planar{0.0, 0.0, 1.5 * kPi}), 2) == -16384);
    check("h just under +pi rounds to -32768, not overflow",
          getWord(encodeOffset(Planar{0.0, 0.0, kPi - 1e-6}), 2) == -32768);
    check("x -10 m fits", getWord(encodeOffset(Planar{-10.0, 0.0, 0.0}), 0) == -32768);
    check("x +10 m is out of range", throwsInvalid([] { (void)encodeOffset(Planar{10.0, 0, 0}); }));
    check("y -10.001 m is out of range",
          throwsInvalid([] { (void)encodeOffset(Planar{0, -10.001, 0}); }));
    check("NaN heading is rejected", throwsInvalid([] {
            (void)encodeOffset(Planar{0, 0, std::numeric_limits<double>::quiet_NaN()});
          }));
  }

  std::printf("Angles\n");
  {
    check("wrap 0 is 0", near(wrapAngle(0.0), 0.0));
    check("wrap +pi is -pi", near(wrapAngle(kPi), -kPi));
    check("wrap -pi stays -pi", near(wrapAngle(-kPi), -kPi));
    check("wrap 3pi is -pi", near(wrapAngle(3.0 * kPi), -kPi));
    check("wrap 2.5 turns is half a turn", near(wrapAngle(5.0 * kPi + 0.25), -kPi + 0.25));
    check("wrap -7 rad", near(wrapAngle(-7.0), -7.0 + 2.0 * kPi));
    bool inRange = true;
    for (std::int32_t i = -2000; i <= 2000; ++i) {
      const double w = wrapAngle(i * 0.01);
      inRange = inRange && w >= -kPi && w < kPi;
    }
    check("wrap always lands in [-pi, pi)", inRange);

    check("delta 0.1 to 0.3 is +0.2", near(headingDelta(0.1, 0.3), 0.2));
    check("delta across +pi is a small positive turn",
          near(headingDelta(kPi - 0.1, -kPi + 0.1), 0.2));
    check("delta across -pi is a small negative turn",
          near(headingDelta(-kPi + 0.1, kPi - 0.1), -0.2));

    // Unwrap ten turns read every 10 degrees, as the reader thread will.
    double total = 0.0;
    double last = 0.0;
    for (std::int32_t step = 1; step <= 360; ++step) {
      const double reading = wrapAngle(step * 10.0 * kPi / 180.0);
      total += headingDelta(last, reading);
      last = reading;
    }
    check("ten turns unwrap to 20 pi", near(total, 20.0 * kPi, 1e-9));
  }

  std::printf("\n%s: %d failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
