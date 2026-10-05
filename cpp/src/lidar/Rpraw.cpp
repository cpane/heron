// File        : Rpraw.cpp
// Author      : Chris Pane
// Description : Reading and writing the .rpraw capture format.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "Rpraw.h"

namespace Heron::Lidar {

namespace {

constexpr std::size_t kHeaderLen = 20;  // magic(4) + seq(4) + t(8) + length(4)
constexpr std::array<char, 4> kMagicTx = {'R', 'P', 'T', 'X'};
constexpr std::array<char, 4> kMagicRx = {'R', 'P', 'R', 'X'};

// Explicit little-endian packing rather than memcpy of a struct: the format
// is defined byte by byte, and this way host endianness and struct padding
// cannot quietly change it.
void putU32(std::uint8_t *out, std::uint32_t value) {
  for (std::uint32_t i = 0; i < 4; ++i) {
    out[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

void putU64(std::uint8_t *out, std::uint64_t value) {
  for (std::uint32_t i = 0; i < 8; ++i) {
    out[i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

std::uint32_t getU32(const std::uint8_t *in) {
  std::uint32_t value = 0;
  for (std::int32_t i = 3; i >= 0; --i) {
    value = (value << 8) | in[i];
  }
  return value;
}

std::uint64_t getU64(const std::uint8_t *in) {
  std::uint64_t value = 0;
  for (std::int32_t i = 7; i >= 0; --i) {
    value = (value << 8) | in[i];
  }
  return value;
}

}  // namespace

std::uint64_t nowMonoNs() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
}

RprawWriter::RprawWriter(const std::string &path, const std::string &headerJson) {
  if (headerJson.find('\n') != std::string::npos) {
    throw std::runtime_error("Heron::Lidar::RprawWriter::RprawWriter() : "
                             "the header must be a single line of JSON");
  }
  m_file = std::fopen(path.c_str(), "wb");
  if (m_file == nullptr) {
    throw std::runtime_error("Heron::Lidar::RprawWriter::RprawWriter() : cannot create " +
                             path + ": " + std::strerror(errno));
  }
  std::fwrite(headerJson.data(), 1, headerJson.size(), m_file);
  std::fputc('\n', m_file);
  std::fflush(m_file);
}

RprawWriter::~RprawWriter() {
  close();
}

void RprawWriter::writeChunk(Direction direction, std::uint64_t tMonoNs, const void *data,
                             std::size_t size) {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_file == nullptr) {
    return;
  }

  std::array<std::uint8_t, kHeaderLen> head{};
  const std::array<char, 4> &magic = (direction == Direction::Tx) ? kMagicTx : kMagicRx;
  std::memcpy(head.data(), magic.data(), magic.size());
  std::uint32_t &seq = (direction == Direction::Tx) ? m_seqTx : m_seqRx;
  putU32(head.data() + 4, seq++);
  putU64(head.data() + 8, tMonoNs);
  putU32(head.data() + 16, static_cast<std::uint32_t>(size));

  std::fwrite(head.data(), 1, head.size(), m_file);
  std::fwrite(data, 1, size, m_file);
}

void RprawWriter::close() {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (m_file != nullptr) {
    std::fclose(m_file);
    m_file = nullptr;
  }
}

Rpraw readRpraw(const std::string &path) {
  const char *context = "Heron::Lidar::readRpraw() : ";
  std::FILE *file = std::fopen(path.c_str(), "rb");
  if (file == nullptr) {
    throw std::runtime_error(context + std::string("cannot open ") + path + ": " +
                             std::strerror(errno));
  }

  Rpraw out;
  int c = 0;  // int, not int32_t: fgetc's return type, so EOF compares correctly
  while ((c = std::fgetc(file)) != EOF && c != '\n') {
    out.headerJson.push_back(static_cast<char>(c));
  }
  if (c == EOF) {
    std::fclose(file);
    throw std::runtime_error(context + path + ": no JSON header line");
  }

  std::array<std::uint8_t, kHeaderLen> head{};
  for (;;) {
    const std::size_t got = std::fread(head.data(), 1, head.size(), file);
    if (got == 0) {
      break;
    }
    if (got < head.size()) {
      out.truncated = true;
      break;
    }

    Chunk chunk;
    if (std::memcmp(head.data(), kMagicTx.data(), 4) == 0) {
      chunk.direction = Direction::Tx;
    } else if (std::memcmp(head.data(), kMagicRx.data(), 4) == 0) {
      chunk.direction = Direction::Rx;
    } else {
      std::fclose(file);
      throw std::runtime_error(context + path + ": bad chunk magic");
    }
    chunk.seq = getU32(head.data() + 4);
    chunk.tMonoNs = getU64(head.data() + 8);
    const std::uint32_t length = getU32(head.data() + 16);

    chunk.data.resize(length);
    if (std::fread(chunk.data.data(), 1, length, file) < length) {
      out.truncated = true;
      break;
    }
    out.chunks.push_back(std::move(chunk));
  }

  std::fclose(file);
  return out;
}

}  // namespace Heron::Lidar
