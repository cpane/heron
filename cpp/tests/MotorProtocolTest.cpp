// File        : MotorProtocolTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the motor link protocol and its frame parser,
//               fed synthetic bytes, with no serial port and no Arduino.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as IBusParserTest.cpp -- deliberately no test framework yet.
// The protocol headers are shared with the AVR firmware; this test is free to
// use the standard library, they are not.

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"

namespace {

using namespace Heron::Motor;
using Bytes = std::vector<std::uint8_t>;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

Bytes bytesOf(const char *text) {
  Bytes out;
  for (const char *c = text; *c != '\0'; ++c) {
    out.push_back(static_cast<std::uint8_t>(*c));
  }
  return out;
}

Bytes drive(std::uint8_t sequence, std::int16_t m1, std::int16_t m2, std::int16_t m3,
            std::int16_t m4) {
  DriveCommand cmd;
  cmd.sequence = sequence;
  cmd.wheels[0] = m1;
  cmd.wheels[1] = m2;
  cmd.wheels[2] = m3;
  cmd.wheels[3] = m4;
  Bytes out(kMaxFrameLen);
  out.resize(encodeDrive(cmd, out.data(), out.size()));
  return out;
}

Bytes concat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes &p : parts) {
    out.insert(out.end(), p.begin(), p.end());
  }
  return out;
}

// Copies each frame out of the parser's buffer, which is reused after the
// callback returns.
struct Received {
  MessageType type;
  std::uint8_t version;
  Bytes payload;
};

struct Collector {
  std::vector<Received> frames;
  std::size_t feed(FrameParser &parser, const Bytes &bytes) {
    return parser.feed(bytes.data(), bytes.size(), [this](const Frame &f) {
      frames.push_back({f.type, f.version, Bytes(f.payload, f.payload + f.payloadLen)});
    });
  }
};

Frame view(const Received &r) {
  Frame f;
  f.type = r.type;
  f.version = r.version;
  f.payload = r.payload.data();
  f.payloadLen = r.payload.size();
  return f;
}

// Rewrites a frame's checksum after a deliberate edit, so the edit is what is
// tested rather than the checksum.
void reseal(Bytes &frame) {
  const std::size_t covered = frame.size() - kChecksumLen;
  putU16(frame.data() + covered, crc16(frame.data(), covered));
}

}  // namespace

int main() {
  std::printf("Checksum\n");

  {
    const Bytes check9 = bytesOf("123456789");
    check("CRC-16/CCITT-FALSE check value: \"123456789\" gives 0x29B1",
          crc16(check9.data(), check9.size()) == 0x29B1);
    check("no bytes give the initial value", crc16(nullptr, 0) == 0xFFFF);
  }

  std::printf("Encoding\n");

  {
    // Built independently in Python (docs/motor-protocol.md §3): sequence 7,
    // wheels 1000, -1000, 197 (0xC5, the start byte inside the payload), -1;
    // protocol version 2.
    const Bytes golden = {0xC5, 0x0B, 0x01, 0x02, 0x07, 0xE8, 0x03, 0x18,
                          0xFC, 0xC5, 0x00, 0xFF, 0xFF, 0xD9, 0x29};
    check("drive: matches the reference frame byte for byte", drive(7, 1000, -1000, 197, -1) == golden);
  }

  {
    check("drive: 15 bytes", drive(0, 0, 0, 0, 0).size() == 15);
    std::uint8_t out[kMaxFrameLen];
    check("stop: 7 bytes", encodeStop(StopCommand{3}, out, sizeof(out)) == 7);
    check("info request: 7 bytes", encodeInfoRequest(InfoRequest{4}, out, sizeof(out)) == 7);
    check("configure: 22 bytes", encodeConfigure(MotorConfig{}, out, sizeof(out)) == 22);
    check("status: 41 bytes", encodeStatus(Status{}, out, sizeof(out)) == 41);
    check("boot: 18 bytes", encodeBoot(BootInfo{}, out, sizeof(out)) == 18);
    check("too small a buffer: nothing written", encodeStatus(Status{}, out, 40) == 0);
    check("too long a payload: refused",
          encodeFrame(MessageType::Status, out, kMaxPayloadLen + 1, out, sizeof(out)) == 0);
  }

  {
    const Bytes f = drive(1, 1200, -1200, 32767, -32768);
    FrameParser p;
    Collector c;
    c.feed(p, f);
    DriveCommand cmd;
    const bool ok = c.frames.size() == 1 && decodeDrive(view(c.frames[0]), cmd);
    check("drive: out of range clamps when encoded",
          ok && cmd.wheels[0] == 1000 && cmd.wheels[1] == -1000 && cmd.wheels[2] == 1000 &&
              cmd.wheels[3] == -1000);
  }

  {
    // A sender that does not clamp: the receiver still never sees 1200.
    Bytes f = drive(1, 0, 0, 0, 0);
    putI16(f.data() + kPayloadOffset + 1, 1200);
    putI16(f.data() + kPayloadOffset + 3, -32768);
    reseal(f);
    FrameParser p;
    Collector c;
    c.feed(p, f);
    DriveCommand cmd;
    const bool ok = c.frames.size() == 1 && decodeDrive(view(c.frames[0]), cmd);
    check("drive: out of range clamps when decoded",
          ok && cmd.wheels[0] == 1000 && cmd.wheels[1] == -1000);
  }

  {
    // Built independently in Python, as the Drive above: sequence 9,
    // terminals 1 2 4 3, motor 3 reversed, ramp 4, the bench's dead band.
    const Bytes golden = {0xC5, 0x12, 0x04, 0x02, 0x09, 0x01, 0x02, 0x04, 0x03, 0x04, 0x04,
                          0x00, 0x4C, 0x51, 0x57, 0x5C, 0x47, 0x4C, 0x4C, 0x66, 0x4E, 0x12};
    MotorConfig config;
    config.sequence = 9;
    const std::uint8_t terminal[] = {1, 2, 4, 3};
    const std::uint8_t forward[] = {76, 81, 87, 92};
    const std::uint8_t backward[] = {71, 76, 76, 102};
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      config.terminal[i] = terminal[i];
      config.minDutyForward[i] = forward[i];
      config.minDutyBackward[i] = backward[i];
    }
    config.reversed = 0x04;
    config.rampPerMs = 4;
    std::uint8_t out[kMaxFrameLen];
    const Bytes frame(out, out + encodeConfigure(config, out, sizeof(out)));
    check("configure: matches the reference frame byte for byte", frame == golden);
    check("configure: its checksum matches the reference (Python: 0x6A5C)",
          configChecksum(config) == 0x6A5C);
    MotorConfig other = config;
    other.sequence = 200;
    check("configure: the checksum ignores the sequence",
          configChecksum(other) == configChecksum(config));
    other.minDutyBackward[3] = 101;
    check("configure: any calibration change changes it",
          configChecksum(other) != configChecksum(config));

    FrameParser p;
    Collector c;
    c.feed(p, frame);
    MotorConfig r;
    const bool ok = c.frames.size() == 1 && decodeConfigure(view(c.frames[0]), r);
    bool arrays = true;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      arrays = arrays && r.terminal[i] == terminal[i] && r.minDutyForward[i] == forward[i] &&
               r.minDutyBackward[i] == backward[i];
    }
    check("configure: every field round trips",
          ok && arrays && r.sequence == 9 && r.reversed == 0x04 && r.rampPerMs == 4);
  }

  {
    MotorConfig good;
    check("valid: the defaults (identity map, no ramp)", isValidConfig(good));
    MotorConfig c = good;
    c.terminal[3] = 1;
    check("invalid: a terminal used twice", !isValidConfig(c));
    c = good;
    c.terminal[0] = 0;
    check("invalid: terminal 0", !isValidConfig(c));
    c = good;
    c.terminal[0] = 5;
    check("invalid: terminal 5", !isValidConfig(c));
    c = good;
    c.reversed = 0x10;
    check("invalid: a reversed bit beyond the four motors", !isValidConfig(c));
    c = good;
    c.rampPerMs = kMaxRampPerMs;
    check("valid: the fastest ramp", isValidConfig(c));
    c.rampPerMs = kMaxRampPerMs + 1;
    check("invalid: faster than that", !isValidConfig(c));
  }

  std::printf("Round trips\n");

  {
    FrameParser p;
    Collector c;
    c.feed(p, drive(200, 1000, -1, 0, -999));
    DriveCommand cmd;
    const bool ok = c.frames.size() == 1 && decodeDrive(view(c.frames[0]), cmd);
    check("drive: every field", ok && cmd.sequence == 200 && cmd.wheels[0] == 1000 &&
                                    cmd.wheels[1] == -1 && cmd.wheels[2] == 0 &&
                                    cmd.wheels[3] == -999);
  }

  {
    std::uint8_t out[kMaxFrameLen];
    FrameParser p;
    Collector c;
    c.feed(p, Bytes(out, out + encodeStop(StopCommand{9}, out, sizeof(out))));
    c.feed(p, Bytes(out, out + encodeInfoRequest(InfoRequest{10}, out, sizeof(out))));
    StopCommand stop;
    InfoRequest info;
    check("stop: sequence", c.frames.size() == 2 && decodeStop(view(c.frames[0]), stop) &&
                                stop.sequence == 9);
    check("info request: sequence", c.frames.size() == 2 &&
                                        decodeInfoRequest(view(c.frames[1]), info) &&
                                        info.sequence == 10);
  }

  {
    // Every field distinct, so a swapped pair shows.
    Status s;
    s.lastSequence = 0xAB;
    s.state = MotorState::Running;
    s.stopReason = StopReason::Watchdog;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      s.target[i] = static_cast<std::int16_t>(-100 * static_cast<std::int16_t>(i + 1));
      s.applied[i] = static_cast<std::int16_t>(10 * static_cast<std::int16_t>(i + 1));
    }
    s.batteryMillivolts = 7400;
    s.watchdogStops = 2;
    s.framesReceived = 65535;
    s.checksumErrors = 3;
    s.versionMismatches = 4;
    s.bytesDiscarded = 5;
    s.longestLoopMicros = 1234;
    s.configChecksum = 0x6A5C;
    std::uint8_t out[kMaxFrameLen];
    FrameParser p;
    Collector c;
    c.feed(p, Bytes(out, out + encodeStatus(s, out, sizeof(out))));
    Status r;
    const bool ok = c.frames.size() == 1 && decodeStatus(view(c.frames[0]), r);
    bool motors = true;
    for (std::size_t i = 0; i < kMotorCount; ++i) {
      motors = motors && r.target[i] == s.target[i] && r.applied[i] == s.applied[i];
    }
    check("status: every field",
          ok && motors && r.lastSequence == 0xAB && r.state == MotorState::Running &&
              r.stopReason == StopReason::Watchdog && r.batteryMillivolts == 7400 &&
              r.watchdogStops == 2 && r.framesReceived == 65535 && r.checksumErrors == 3 &&
              r.versionMismatches == 4 && r.bytesDiscarded == 5 && r.longestLoopMicros == 1234 &&
              r.configChecksum == 0x6A5C);
  }

  {
    BootInfo b;
    b.firmwareMajor = 1;
    b.firmwareMinor = 2;
    b.firmwarePatch = 3;
    b.platform = Platform::UnoL293d;
    b.resetFlags = kResetBrownOut | kResetExternal;
    b.uptimeMillis = 0x89ABCDEF;
    b.freeRamBytes = 1500;
    std::uint8_t out[kMaxFrameLen];
    FrameParser p;
    Collector c;
    c.feed(p, Bytes(out, out + encodeBoot(b, out, sizeof(out))));
    BootInfo r;
    const bool ok = c.frames.size() == 1 && decodeBoot(view(c.frames[0]), r);
    check("boot: every field",
          ok && r.firmwareMajor == 1 && r.firmwareMinor == 2 && r.firmwarePatch == 3 &&
              r.protocolVersion == kProtocolVersion && r.platform == Platform::UnoL293d &&
              r.resetFlags == (kResetBrownOut | kResetExternal) && r.uptimeMillis == 0x89ABCDEF &&
              r.freeRamBytes == 1500);
  }

  {
    FrameParser p;
    Collector c;
    c.feed(p, drive(1, 0, 0, 0, 0));
    StopCommand stop;
    Status status;
    check("decode: another type is refused",
          c.frames.size() == 1 && !decodeStop(view(c.frames[0]), stop) &&
              !decodeStatus(view(c.frames[0]), status));
    Received shortened = c.frames[0];
    shortened.payload.pop_back();
    DriveCommand cmd;
    check("decode: a wrong payload length is refused", !decodeDrive(view(shortened), cmd));
  }

  std::printf("Parser\n");

  {
    FrameParser p;
    Collector c;
    const Bytes f = drive(5, 100, 200, 300, 400);
    std::size_t n = 0;
    for (const std::uint8_t b : f) {
      n += p.feed(b, [&c](const Frame &fr) {
        c.frames.push_back({fr.type, fr.version, Bytes(fr.payload, fr.payload + fr.payloadLen)});
      });
    }
    check("byte at a time: one frame, on the last byte", n == 1 && c.frames.size() == 1);
    check("byte at a time: nothing discarded",
          p.getStats().bytesDiscarded == 0 && p.getStats().framesReceived == 1);
  }

  {
    FrameParser p;
    Collector c;
    const std::size_t n = c.feed(p, concat({drive(1, 1, 1, 1, 1), drive(2, 2, 2, 2, 2)}));
    check("two frames in one read: both", n == 2 && c.frames.size() == 2);
  }

  {
    FrameParser p;
    Collector c;
    c.feed(p, concat({Bytes{0x00, 0xFF, 0x12, 0x34}, drive(1, 0, 0, 0, 0)}));
    check("leading noise: frame found", c.frames.size() == 1);
    check("leading noise: 4 bytes discarded, no checksum error",
          p.getStats().bytesDiscarded == 4 && p.getStats().checksumErrors == 0);
  }

  {
    FrameParser p;
    Collector c;
    Bytes bad = drive(1, 500, 500, 500, 500);
    bad[6] ^= 0x01;  // one bit flipped in motor 1
    c.feed(p, concat({bad, drive(2, 0, 0, 0, 0)}));
    DriveCommand cmd;
    check("corrupted frame: rejected, the next accepted",
          c.frames.size() == 1 && decodeDrive(view(c.frames[0]), cmd) && cmd.sequence == 2);
    check("corrupted frame: one checksum error, all 15 bytes discarded",
          p.getStats().checksumErrors == 1 && p.getStats().bytesDiscarded == 15);
  }

  {
    // A start byte in noise, followed by a length that claims most of the
    // buffer: the parser waits for that frame, the real one completes inside
    // it, and is found when the false one's checksum fails.
    FrameParser p;
    Collector c;
    const Bytes noise = {kStartByte, kMaxLengthField};
    const Bytes real = drive(3, 7, 7, 7, 7);
    Bytes filler(kMaxFrameLen - noise.size() - real.size(), 0x00);
    c.feed(p, concat({noise, real}));
    check("false start: the real frame waits behind it", c.frames.empty());
    c.feed(p, filler);
    DriveCommand cmd;
    check("false start: the real frame is found when the false one fails",
          c.frames.size() == 1 && decodeDrive(view(c.frames[0]), cmd) && cmd.sequence == 3);
    check("false start: counted as one checksum error", p.getStats().checksumErrors == 1);
  }

  {
    FrameParser p;
    Collector c;
    Bytes tooShort = {kStartByte, kMinLengthField - 1};
    Bytes tooLong = {kStartByte, kMaxLengthField + 1};
    c.feed(p, concat({tooShort, tooLong, drive(4, 0, 0, 0, 0)}));
    check("length out of range: skipped without waiting", c.frames.size() == 1);
    check("length out of range: 4 bytes discarded, no checksum error",
          p.getStats().bytesDiscarded == 4 && p.getStats().checksumErrors == 0);
  }

  {
    FrameParser p;
    Collector c;
    Bytes other = drive(1, 0, 0, 0, 0);
    other[kVersionOffset] = kProtocolVersion + 1;
    reseal(other);
    c.feed(p, concat({other, drive(2, 0, 0, 0, 0)}));
    check("other version: counted, not handed over, the next accepted",
          c.frames.size() == 1 && p.getStats().versionMismatches == 1 &&
              p.getStats().framesReceived == 1 && p.getStats().bytesDiscarded == 0);
  }

  {
    // Every possible split of two frames over two reads.
    const Bytes both = concat({drive(1, 10, 20, 30, 40), drive(2, 50, 60, 70, 80)});
    bool allOk = true;
    for (std::size_t split = 0; split <= both.size(); ++split) {
      FrameParser p;
      Collector c;
      c.feed(p, Bytes(both.begin(), both.begin() + static_cast<std::ptrdiff_t>(split)));
      c.feed(p, Bytes(both.begin() + static_cast<std::ptrdiff_t>(split), both.end()));
      allOk = allOk && c.frames.size() == 2 && p.getStats().bytesDiscarded == 0;
    }
    check("any split across reads: both frames, nothing discarded", allOk);
  }

  {
    // Every single-bit error in a frame is caught. The parser then resyncs on
    // the frame behind it.
    const Bytes good = drive(9, 123, -456, 789, -1000);
    bool allCaught = true;
    for (std::size_t bit = 0; bit < good.size() * 8; ++bit) {
      Bytes bad = good;
      bad[bit / 8] = static_cast<std::uint8_t>(bad[bit / 8] ^ (1U << (bit % 8)));
      FrameParser p;
      Collector c;
      // A flipped length bit can claim a longer frame; the filler completes
      // it, so its checksum fails and the parser moves on.
      c.feed(p, concat({bad, good, Bytes(kMaxFrameLen, 0x00)}));
      DriveCommand cmd;
      allCaught = allCaught && c.frames.size() == 1 &&
                  decodeDrive(view(c.frames[0]), cmd) && cmd.sequence == 9;
    }
    check("every single-bit error: rejected, the next frame accepted", allCaught);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
