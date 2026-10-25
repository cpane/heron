// File        : FrameParser.h
// Author      : Chris Pane
// Description : Motor link frame parser: turns a byte stream into checked
//               frames, with the link counters that Status reports.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Used by both sides: the firmware feeds it what Serial.read() returns, the
// Pi what read() on /dev/arduino returns. No C++ standard library, as in
// Protocol.h; the callback is a template parameter, so a lambda works on the
// AVR without std::function.
//
// Resync is byte by byte, as in the iBUS parser: whenever the front of the
// buffer cannot start a valid frame -- no start byte, a length out of range,
// or a checksum that does not match -- one byte is dropped and the search
// continues from the next. A start byte inside a payload can therefore make
// the parser wait for a frame that is not there, but never accept one: the
// checksum decides. The frame hidden behind it is found when the checksum
// fails.

#ifndef HERON_MOTOR_FRAME_PARSER_H
#define HERON_MOTOR_FRAME_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "heron/motor/Protocol.h"

namespace Heron::Motor {

/**
 * What the parser has seen since construction. 16-bit counters that wrap,
 * as on the wire in Status; take differences.
 */
struct ParserStats {
  uint16_t framesReceived{0};     // checksum and version good, handed over
  uint16_t checksumErrors{0};     // a plausible frame whose checksum failed
  uint16_t versionMismatches{0};  // checksum good, another protocol version
  uint16_t bytesDiscarded{0};     // dropped while hunting for a frame start
};

/**
 * Decodes motor link frames from a byte stream.
 *
 * Feed it bytes in order, partial frames included; it keeps up to one frame
 * of bytes between calls. Frames of another protocol version are counted and
 * not handed over. The message type is not checked here: the decode*()
 * functions in Protocol.h reject a type or length they do not expect.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 *
 * Usage:
 * @code
 * FrameParser parser;
 * parser.feed(byte, [&](const Frame &frame) {
 *   DriveCommand cmd;
 *   if (decodeDrive(frame, cmd)) { apply(cmd); }
 * });
 * @endcode
 */
class FrameParser {
  public:
    /**
     * Consumes one byte, calling onFrame for each valid frame it completes.
     *
     * @param byte The next byte from the stream
     * @param onFrame Called as onFrame(const Frame &), synchronously. The frame
     *        points into the parser's buffer: copy what is needed, and do not
     *        feed this parser from inside the callback
     * @return The number of frames handed over by this call
     */
    template <typename Handler>
    size_t feed(uint8_t byte, Handler &&onFrame) {
      m_buf[m_len++] = byte;
      size_t delivered = 0;
      // Re-examine the front until it is a plausible frame still filling, or
      // the buffer is empty. Dropping one byte can expose a complete frame
      // further in, so one byte can complete more than one.
      while (m_len > 0) {
        if (m_buf[0] != kStartByte) {
          dropFront();
          continue;
        }
        if (m_len <= kLengthOffset) {
          break;
        }
        const uint8_t lengthField = m_buf[kLengthOffset];
        if (lengthField < kMinLengthField || lengthField > kMaxLengthField) {
          dropFront();
          continue;
        }
        const size_t total = kLengthOffset + 1 + lengthField + kChecksumLen;
        if (m_len < total) {
          break;
        }
        const size_t covered = total - kChecksumLen;
        if (crc16(m_buf, covered) != getU16(m_buf + covered)) {
          ++m_stats.checksumErrors;
          dropFront();
          continue;
        }
        if (m_buf[kVersionOffset] != kProtocolVersion) {
          ++m_stats.versionMismatches;
        } else {
          ++m_stats.framesReceived;
          Frame frame;
          frame.type = static_cast<MessageType>(m_buf[kTypeOffset]);
          frame.version = m_buf[kVersionOffset];
          frame.payload = m_buf + kPayloadOffset;
          frame.payloadLen = covered - kPayloadOffset;
          onFrame(frame);
          ++delivered;
        }
        consume(total);
      }
      return delivered;
    }

    /**
     * Consumes count bytes. @return The number of frames handed over
     */
    template <typename Handler>
    size_t feed(const uint8_t *bytes, size_t count, Handler &&onFrame) {
      size_t delivered = 0;
      for (size_t i = 0; i < count; ++i) {
        delivered += feed(bytes[i], onFrame);
      }
      return delivered;
    }

    [[nodiscard]] const ParserStats &getStats() const { return m_stats; }

  private:
    void dropFront() {
      ++m_stats.bytesDiscarded;
      consume(1);
    }

    void consume(size_t count) {
      for (size_t i = count; i < m_len; ++i) {
        m_buf[i - count] = m_buf[i];
      }
      m_len -= count;
    }

    uint8_t m_buf[kMaxFrameLen]{};
    size_t m_len{0};
    ParserStats m_stats{};
};

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_FRAME_PARSER_H
