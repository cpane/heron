// File        : ReplayTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the .rpraw format and the lockstep replay
//               channel, with no SDK driver and no hardware.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The channel is driven directly, playing the part of the SDK's two threads.
// Same style as ScanTest.cpp -- deliberately no test framework yet.

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "lidar/ReplayChannel.h"
#include "lidar/Rpraw.h"

namespace {

using Heron::Lidar::Chunk;
using Heron::Lidar::Direction;
using Heron::Lidar::ReplayChannel;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

Chunk makeChunk(Direction direction, std::uint64_t tMs, const std::string &bytes) {
  Chunk chunk;
  chunk.direction = direction;
  chunk.tMonoNs = tMs * 1'000'000ULL;
  chunk.data.assign(bytes.begin(), bytes.end());
  return chunk;
}

// What the SDK's receive thread does: wait, then read what was hinted.
std::string take(ReplayChannel &channel, sl_u32 timeoutMs) {
  std::size_t hint = 0;
  if (SL_IS_FAIL(channel.waitForDataExt(hint, timeoutMs)) || hint == 0) {
    return "";
  }
  std::string out(hint, '\0');
  const int got = channel.read(out.data(), hint);  // int: the SDK's read() signature
  out.resize(got > 0 ? static_cast<std::size_t>(got) : 0);
  return out;
}

bool send(ReplayChannel &channel, const std::string &bytes) {
  return channel.write(bytes.data(), bytes.size()) == static_cast<int>(bytes.size());
}

// A dialogue shaped like the SDK's: stale bytes, then question and answer
// twice, then a stream.
std::vector<Chunk> dialogue() {
  return {
      makeChunk(Direction::Rx, 0, "stale"), makeChunk(Direction::Tx, 10, "Q1"),
      makeChunk(Direction::Rx, 12, "A1"),   makeChunk(Direction::Tx, 20, "Q2"),
      makeChunk(Direction::Rx, 22, "A2"),   makeChunk(Direction::Rx, 30, "stream"),
  };
}

}  // namespace

int main() {
  std::printf("rpraw format and lockstep replay\n");

  {
    const std::string path = "replay_test.rpraw";
    {
      Heron::Lidar::RprawWriter writer(path, "{\"type\":\"header\",\"schema\":1}");
      writer.writeChunk(Direction::Tx, 123, "\xa5\x20", 2);
      writer.writeChunk(Direction::Rx, 456, "\xa5\x5a\x05", 3);
      writer.writeChunk(Direction::Rx, 789, "", 0);
    }
    const Heron::Lidar::Rpraw raw = Heron::Lidar::readRpraw(path);
    check("round trip: header line preserved",
          raw.headerJson == "{\"type\":\"header\",\"schema\":1}");
    check("round trip: three chunks, not truncated", raw.chunks.size() == 3 && !raw.truncated);
    check("round trip: direction, seq per direction, time, bytes",
          raw.chunks[0].direction == Direction::Tx && raw.chunks[0].seq == 0 &&
              raw.chunks[1].direction == Direction::Rx && raw.chunks[1].seq == 0 &&
              raw.chunks[2].seq == 1 && raw.chunks[1].tMonoNs == 456 &&
              raw.chunks[1].data.size() == 3 && raw.chunks[1].data[2] == 0x05);
    std::remove(path.c_str());
  }

  {
    ReplayChannel channel(dialogue(), 0.0);
    channel.open();
    check("bytes recorded before the first write are served at once",
          take(channel, 50) == "stale");
    check("the answer is withheld until its question is asked", take(channel, 50).empty());
    check("the right question is accepted", send(channel, "Q1"));
    check("...and releases its answer", take(channel, 50) == "A1");
    check("the next answer is still withheld", take(channel, 50).empty());
    send(channel, "Q2");
    check("second answer, then the stream, in recorded pieces",
          take(channel, 50) == "A2" && take(channel, 50) == "stream");
    check("finished once every write matched and every byte served",
          channel.isFinished() && channel.getDivergence().empty() &&
              channel.getWritesMatched() == 2);
    check("past the end: a timeout, never a zero-byte read", take(channel, 20).empty());
  }

  {
    ReplayChannel channel(dialogue(), 0.0);
    channel.open();
    take(channel, 50);
    send(channel, "Q9");
    const std::string divergence = channel.getDivergence();
    check("a different question is reported as divergence",
          divergence.find("write #1 differs") != std::string::npos &&
              divergence.find("5139") != std::string::npos);  // hex of "Q9"
    check("...and nothing more is served after it", take(channel, 50).empty());
    check("...and the replay is not finished", !channel.isFinished());
  }

  {
    ReplayChannel channel(dialogue(), 0.0);
    channel.open();
    take(channel, 50);
    send(channel, "Q1");
    send(channel, "Q2");
    send(channel, "Q3");
    check("a write beyond the recording is divergence",
          channel.getDivergence().find("beyond the recording") != std::string::npos);
  }

  {
    // A write that arrives before the receive thread has drained what
    // preceded it: those bytes must still come out first, in order.
    ReplayChannel channel(dialogue(), 0.0);
    channel.open();
    send(channel, "Q1");
    check("bytes before a write survive the write overtaking them",
          take(channel, 50) == "stale" && take(channel, 50) == "A1");
  }

  {
    // Pacing: at 1x, the stream chunk recorded 8 ms after A2 is not served
    // sooner than that after Q2 is matched.
    ReplayChannel channel(dialogue(), 1.0);
    channel.open();
    take(channel, 50);
    send(channel, "Q1");
    take(channel, 50);
    send(channel, "Q2");
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const bool a2 = take(channel, 100) == "A2";
    const bool stream = take(channel, 100) == "stream";
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    check("1x pacing: the stream arrives ~10 ms after its question, not at once",
          a2 && stream && ms >= 9.0 && ms < 60.0);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
