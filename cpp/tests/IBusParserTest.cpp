// File        : IBusParserTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the iBUS parser and the stale-link rule, fed
//               synthetic bytes, with no serial port and no hardware.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <array>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <vector>

#include "heron/remote/IBusParser.h"
#include "heron/remote/LinkMonitor.h"

namespace {

using namespace Heron::Remote;
using Bytes = std::vector<std::uint8_t>;
using std::chrono::milliseconds;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

using Channels = std::array<std::uint16_t, kIBusChannelCount>;

Channels ramp() {
  Channels ch{};
  for (std::size_t i = 0; i < ch.size(); ++i) {
    ch[i] = static_cast<std::uint16_t>(1000 + 50 * i);
  }
  return ch;
}

// Builds a valid frame, independently of the parser: the checksum is
// computed here, not with IBusParser::checksum().
Bytes makeFrame(const Channels &ch) {
  Bytes f = {kIBusHeader0, kIBusHeader1};
  for (const std::uint16_t v : ch) {
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

Bytes concat(std::initializer_list<Bytes> parts) {
  Bytes out;
  for (const Bytes &p : parts) {
    out.insert(out.end(), p.begin(), p.end());
  }
  return out;
}

struct Collector {
  std::vector<IBusFrame> frames;
  IBusParser::FrameHandler handler() {
    return [this](const IBusFrame &f) { frames.push_back(f); };
  }
};

const TimePoint kT0{};

}  // namespace

int main() {
  std::printf("iBUS parser\n");

  {
    const Bytes f = makeFrame(ramp());
    check("checksum: matches an independent computation",
          IBusParser::checksum(f) == static_cast<std::uint16_t>(f[30] | (f[31] << 8)));
  }

  {
    // Captured from our FS-iA6B with `ibus_reader dump` on the Pi, 2026-10-10:
    // the worked example in docs/remote-ibus-protocol.md §3.
    const Bytes real = {0x20, 0x40, 0xDE, 0x05, 0xDC, 0x05, 0x7E, 0x04, 0xDC, 0x05, 0x93,
                        0x07, 0x7D, 0x07, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
                        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0x52, 0xF4};
    IBusParser p;
    Collector c;
    p.feed(real, kT0, c.handler());
    const Channels expected = {1502, 1500, 1150, 1500, 1939, 1917, 1500,
                               1500, 1500, 1500, 1500, 1500, 1500, 1500};
    check("real frame: checksum is 0xF452", IBusParser::checksum(real) == 0xF452);
    check("real frame: decodes to the captured channels",
          c.frames.size() == 1 && c.frames[0].channels == expected);
  }

  {
    IBusParser p;
    Collector c;
    const std::size_t n = p.feed(makeFrame(ramp()), kT0, c.handler());
    check("valid frame: one decoded", n == 1 && c.frames.size() == 1);
    check("valid frame: channels in order", c.frames.size() == 1 && c.frames[0].channels == ramp());
    check("valid frame: raw bytes kept", c.frames.size() == 1 &&
                                             Bytes(c.frames[0].bytes.begin(),
                                                   c.frames[0].bytes.end()) == makeFrame(ramp()));
    check("valid frame: no errors, nothing discarded",
          p.getStats().checksumErrors == 0 && p.getStats().discardedBytes == 0 &&
              p.getStats().resyncs == 0 && p.getStats().bytesReceived == 32);
  }

  {
    IBusParser p;
    Collector c;
    Bytes f = makeFrame(ramp());
    f[1] = 0x41;
    p.feed(f, kT0, c.handler());
    check("bad header: nothing decoded", c.frames.empty());
    check("bad header: not counted as a checksum error", p.getStats().checksumErrors == 0);
    p.feed(makeFrame(ramp()), kT0, c.handler());
    check("bad header: recovers on the next frame", c.frames.size() == 1);
    check("bad header: one resync for the whole run", p.getStats().resyncs == 1);
    check("bad header: all 32 junk bytes discarded", p.getStats().discardedBytes == 32);
  }

  {
    IBusParser p;
    Collector c;
    Bytes bad = makeFrame(ramp());
    bad[10] ^= 0x01;
    p.feed(concat({bad, makeFrame(ramp())}), kT0, c.handler());
    check("bad checksum: rejected, then the next frame decodes", c.frames.size() == 1);
    check("bad checksum: counted once", p.getStats().checksumErrors == 1);
    check("bad checksum: whole bad frame discarded", p.getStats().discardedBytes == 32);
  }

  {
    IBusParser p;
    Collector c;
    const Bytes f = concat({makeFrame(ramp()), makeFrame(ramp())});
    // Every possible split point, one byte at a time and in odd chunks.
    std::size_t total = 0;
    for (std::size_t i = 0; i < f.size(); ++i) {
      total += p.feed(std::span(&f[i], 1), kT0, c.handler());
    }
    check("split: byte by byte gives both frames", total == 2 && c.frames.size() == 2);
    bool allSplits = true;
    for (std::size_t cut = 1; cut < 32; ++cut) {
      IBusParser q;
      Collector d;
      const Bytes g = makeFrame(ramp());
      q.feed(std::span(g.data(), cut), kT0, d.handler());
      const bool none = d.frames.empty();
      q.feed(std::span(g.data() + cut, g.size() - cut), kT0, d.handler());
      allSplits = allSplits && none && d.frames.size() == 1;
    }
    check("split: every cut point 1..31 decodes once, only when complete", allSplits);
  }

  {
    IBusParser p;
    Collector c;
    // Junk that includes a lone header byte and a header pair, as a line
    // joined mid-frame would.
    const Bytes junk = {0x00, 0xFF, 0x20, 0x13, 0x20, 0x40, 0xDC, 0x05, 0x20};
    p.feed(concat({junk, makeFrame(ramp())}), kT0, c.handler());
    check("junk before a frame: frame still found", c.frames.size() == 1);
    check("junk before a frame: exactly the junk discarded",
          p.getStats().discardedBytes == junk.size());
  }

  {
    IBusParser p;
    Collector c;
    // A partial frame at startup, as when the port opens mid-packet.
    const Bytes f = makeFrame(ramp());
    const Bytes tail(f.begin() + 17, f.end());
    p.feed(concat({tail, makeFrame(ramp())}), kT0, c.handler());
    check("partial frame at startup: discarded, next frame decodes", c.frames.size() == 1);
  }

  {
    IBusParser p;
    Collector c;
    Channels a = ramp();
    Channels b = ramp();
    b[0] = 1999;
    const std::size_t n = p.feed(concat({makeFrame(a), makeFrame(b), makeFrame(a)}), kT0,
                                 c.handler());
    check("back-to-back: three frames", n == 3);
    check("back-to-back: order kept", c.frames.size() == 3 && c.frames[1].channels[0] == 1999 &&
                                          c.frames[2].channels[0] == 1000);
    check("back-to-back: nothing discarded", p.getStats().discardedBytes == 0);
  }

  {
    IBusParser p;
    Collector c;
    Channels lo{};
    Channels hi{};
    hi.fill(0xFFFF);
    p.feed(concat({makeFrame(lo), makeFrame(hi)}), kT0, c.handler());
    check("extremes: 0 and 0xFFFF decode unchanged",
          c.frames.size() == 2 && c.frames[0].channels == lo && c.frames[1].channels == hi);
  }

  {
    check("high nibble: plain value has none", channelHighNibble(1500) == 0);
    check("high nibble: 0x23E8 splits into 0x2 and 1000",
          channelHighNibble(0x23E8) == 0x2 && channelValue(0x23E8) == 1000);
    check("high nibble: 0xFFFF splits into 0xF and 0xFFF",
          channelHighNibble(0xFFFF) == 0xF && channelValue(0xFFFF) == 0x0FFF);
  }

  {
    // Stats: intervals and rate from the stamps given to feed().
    IBusParser p;
    Collector c;
    const Bytes f = makeFrame(ramp());
    p.feed(f, kT0, c.handler());
    p.feed(f, kT0 + milliseconds(7), c.handler());
    p.feed(f, kT0 + milliseconds(15), c.handler());
    p.feed(f, kT0 + milliseconds(21), c.handler());
    const IBusStats &s = p.getStats();
    check("stats: four frames", s.framesDecoded == 4 && s.intervalCount() == 3);
    check("stats: min interval 6 ms", s.minInterval == milliseconds(6));
    check("stats: max interval 8 ms", s.maxInterval == milliseconds(8));
    check("stats: mean interval 7 ms", s.meanInterval() == milliseconds(7));
    // Intervals 7, 8, 6: mean 7, population variance 2/3 ms², sd 0.8165 ms.
    const double sdMs = std::chrono::duration<double, std::milli>(s.intervalStdDev()).count();
    check("stats: interval std dev 0.816 ms", sdMs > 0.816 && sdMs < 0.817);
    check("stats: rate 3 intervals / 21 ms", s.frameRateHz() > 142.8 && s.frameRateHz() < 142.9);
    p.resetStats();
    check("stats: reset clears counts", p.getStats().framesDecoded == 0 &&
                                            p.getStats().meanInterval() == milliseconds(0) &&
                                            p.getStats().frameRateHz() == 0.0);
  }

  std::printf("Link monitor\n");

  {
    LinkMonitor link(milliseconds(100));
    check("link: no signal before any frame", link.getState(kT0) == LinkState::NoSignal);
    check("link: no signal is not ok", !link.isOk(kT0));
    link.onFrame(kT0);
    check("link: ok at the frame", link.getState(kT0) == LinkState::Ok);
    check("link: ok at exactly the limit", link.getState(kT0 + milliseconds(100)) == LinkState::Ok);
    check("link: stale just past it",
          link.getState(kT0 + milliseconds(101)) == LinkState::Stale);
    check("link: time since last frame",
          link.sinceLastFrame(kT0 + milliseconds(42)) == milliseconds(42));
    link.onFrame(kT0 + milliseconds(150));
    check("link: a new frame clears stale", link.isOk(kT0 + milliseconds(160)));
    bool threw = false;
    try {
      LinkMonitor bad(milliseconds(0));
    } catch (const std::invalid_argument &) {
      threw = true;
    }
    check("link: zero stale time rejected", threw);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
