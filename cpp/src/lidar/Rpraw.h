// File        : Rpraw.h
// Author      : Chris Pane
// Description : The .rpraw capture format: a byte-exact wire log, readable by
//               both python/heron/lidar/recording.py and this code.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     <one line of JSON header>\n
//     chunk*    magic(4) seq(u32) t_mono_ns(u64) length(u32) payload(length)
//
// All integers little-endian. magic is "RPTX" (host -> sensor) or "RPRX"
// (sensor -> host), and seq counts each direction separately. One chunk is
// one read() or write() as it crossed the wire; the boundaries are kept on
// purpose, because they are the evidence for USB and kernel batching.
//
// No SDK types here. The format is ours, and a tool that only inspects
// captures should not need the vendor SDK to do it.
//
// Errors are thrown as std::runtime_error, with the standard's
// "<namespace>::<class>::<method>() : ..." context format.

#ifndef HERON_LIDAR_RPRAW_H
#define HERON_LIDAR_RPRAW_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

namespace Heron::Lidar {

/**
 * Which way a chunk crossed the wire.
 */
enum class Direction : std::uint8_t {
  Tx,  // host -> sensor
  Rx   // sensor -> host
};

/**
 * One read() or write() worth of bytes, as it crossed the wire.
 */
struct Chunk {
  Direction direction{Direction::Rx};
  std::uint32_t seq{0};
  std::uint64_t tMonoNs{0};
  std::vector<std::uint8_t> data;
};

/**
 * Append-only .rpraw writer.
 *
 * Thread Safety: thread-safe. The SDK writes from the caller's thread and
 * reads on its own receive thread, and both are recorded.
 *
 * Usage:
 * @code
 * RprawWriter writer("captures/x.rpraw", "{\"type\":\"header\",\"schema\":1}");
 * writer.writeChunk(Direction::Tx, nowMonoNs(), bytes, size);
 * @endcode
 */
class RprawWriter {
  public:
    /**
     * Creates the file and writes the header line.
     *
     * @param path Where to create the capture
     * @param headerJson A single line of JSON; the newline is added here
     * @throws std::runtime_error if the header spans lines or the file cannot
     *         be created
     */
    RprawWriter(const std::string &path, const std::string &headerJson);
    ~RprawWriter();

    RprawWriter(const RprawWriter &) = delete;
    RprawWriter &operator=(const RprawWriter &) = delete;

    /**
     * Appends one chunk. Does nothing once closed.
     *
     * @param direction Which way the bytes went
     * @param tMonoNs When, in steady_clock nanoseconds
     * @param data The bytes
     * @param size How many
     */
    void writeChunk(Direction direction, std::uint64_t tMonoNs, const void *data,
                    std::size_t size);

    /**
     * Flushes and closes. Safe to call twice; the destructor calls it.
     */
    void close();

  private:
    std::FILE *m_file{nullptr};
    std::uint32_t m_seqTx{0};
    std::uint32_t m_seqRx{0};
    std::mutex m_mutex;
};

/**
 * A whole capture, read into memory.
 */
struct Rpraw {
  std::string headerJson;  // the first line, without its newline
  std::vector<Chunk> chunks;
  bool truncated{false};  // ended mid-chunk: a capture killed mid-write
};

/**
 * Reads a whole capture.
 *
 * Tolerates a truncated final chunk, as the Python reader does.
 *
 * @param path The .rpraw to read
 * @return The header line and every complete chunk
 * @throws std::runtime_error if the file cannot be opened, has no header
 *         line, or contains a bad chunk magic
 */
Rpraw readRpraw(const std::string &path);

/**
 * @return Monotonic nanoseconds, the clock both writers stamp chunks with
 */
std::uint64_t nowMonoNs();

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_RPRAW_H
