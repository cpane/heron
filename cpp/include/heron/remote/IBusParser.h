// File        : IBusParser.h
// Author      : Chris Pane
// Description : FlySky iBUS frame parser: turns a byte stream from the
//               receiver's SERVO port into channel frames, with statistics.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// No I/O and no printing: bytes go in through feed(), frames come out through
// a callback. That is what lets the tests drive it with synthetic bytes, and
// what lets the robot reuse it behind whatever reads the UART.
//
// A frame is 32 bytes: the header 0x20 0x40, fourteen little-endian 16-bit
// channels, then a little-endian checksum equal to 0xFFFF minus the sum of the
// first 30 bytes. The protocol and its sources are in
// docs/remote-ibus-protocol.md.
//
// Resync is byte by byte: whenever the front of the buffer cannot start a
// valid frame -- wrong header, or a checksum that does not match -- one byte
// is dropped and the search continues from the next. A checksum failure
// therefore costs at most one frame plus the bytes up to the next header.

#ifndef HERON_REMOTE_IBUS_PARSER_H
#define HERON_REMOTE_IBUS_PARSER_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>

namespace Heron::Remote {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

inline constexpr std::size_t kIBusFrameLen = 32;
inline constexpr std::size_t kIBusChannelCount = 14;
inline constexpr std::uint8_t kIBusHeader0 = 0x20;  // also the frame length
inline constexpr std::uint8_t kIBusHeader1 = 0x40;  // the "servo data" command
inline constexpr std::size_t kIBusChecksumOffset = 30;

/**
 * One valid frame.
 */
struct IBusFrame {
  std::array<std::uint16_t, kIBusChannelCount> channels{};  // raw, about 1000..2000
  std::array<std::uint8_t, kIBusFrameLen> bytes{};          // exactly as received
  TimePoint stamp{};  // when the read that completed this frame returned
};

/**
 * What the parser has seen since construction or the last reset().
 *
 * Intervals are between the stamps of consecutive valid frames, so they are
 * read-time intervals: two frames completed by the same read() share a stamp
 * and count as an interval of zero.
 */
struct IBusStats {
  std::uint64_t bytesReceived{0};
  std::uint64_t framesDecoded{0};
  /** Frames whose header matched but whose checksum did not. */
  std::uint64_t checksumErrors{0};
  /** Bytes dropped while hunting for a frame start. */
  std::uint64_t discardedBytes{0};
  /** How many times the parser lost sync and had to hunt: one per run of
   *  discarded bytes, however long the run. */
  std::uint64_t resyncs{0};
  TimePoint firstFrame{};
  TimePoint lastFrame{};
  Duration minInterval{Duration::max()};
  Duration maxInterval{Duration::zero()};
  Duration totalInterval{Duration::zero()};  // sum, for the mean

  /** @return Intervals measured so far: one fewer than the frames */
  [[nodiscard]] std::uint64_t intervalCount() const noexcept {
    return framesDecoded > 0 ? framesDecoded - 1 : 0;
  }

  /** @return Mean interval between frames; zero until two have arrived */
  [[nodiscard]] Duration meanInterval() const noexcept {
    const std::uint64_t n = intervalCount();
    return n > 0 ? totalInterval / static_cast<Duration::rep>(n) : Duration::zero();
  }

  /** @return Frames per second over the span from first to last frame; zero
   *  until two frames have arrived */
  [[nodiscard]] double frameRateHz() const noexcept {
    const double seconds = std::chrono::duration<double>(lastFrame - firstFrame).count();
    return seconds > 0.0 ? static_cast<double>(intervalCount()) / seconds : 0.0;
  }
};

/**
 * Decodes iBUS frames from a byte stream.
 *
 * Feed it whatever each read returned, in order, partial frames included;
 * it keeps up to one frame's worth of bytes between calls.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 *
 * Usage:
 * @code
 * IBusParser parser;
 * parser.feed(bytes, Clock::now(), [&](const IBusFrame &frame) { use(frame); });
 * @endcode
 */
class IBusParser {
  public:
    using FrameHandler = std::function<void(const IBusFrame &)>;

    /**
     * Consumes bytes, calling onFrame once for each valid frame they complete.
     *
     * @param bytes The next bytes from the stream
     * @param stamp When they were read; stamps every frame they complete
     * @param onFrame Called synchronously, in stream order
     * @return The number of frames decoded by this call
     */
    std::size_t feed(std::span<const std::uint8_t> bytes, TimePoint stamp,
                     const FrameHandler &onFrame);

    [[nodiscard]] const IBusStats &getStats() const noexcept { return m_stats; }

    /** Clears the statistics. Buffered bytes are kept, so sync is not lost. */
    void resetStats() noexcept { m_stats = IBusStats{}; }

    /**
     * @param frame At least 30 bytes: the header and channels
     * @return The checksum the frame should carry: 0xFFFF minus the byte sum
     */
    [[nodiscard]] static std::uint16_t checksum(std::span<const std::uint8_t> frame) noexcept;

  private:
    void dropFront() noexcept;
    void decodeFrame(TimePoint stamp, const FrameHandler &onFrame);

    std::array<std::uint8_t, kIBusFrameLen> m_buf{};
    std::size_t m_len{0};
    bool m_hunting{false};  // inside a run of discarded bytes
    IBusStats m_stats{};
};

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_IBUS_PARSER_H
