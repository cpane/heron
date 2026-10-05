// File        : LidarRecord.cpp
// Author      : Chris Pane
// Description : Probe C2. Records an SDK session to .rpraw, or replays one
//               through the SDK.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh <user>@<pi> lidar_record --mode 1 --scans 40 --tag desk
//     build-host/bin/lidar_record --replay captures/desk-sdk1_<stamp>.rpraw
//
// Live, the SDK talks to the sensor through a TapChannel, and every byte in
// both directions lands in captures/<tag>-sdk<mode>_<stamp>.rpraw. Replayed,
// the SDK talks to a ReplayChannel serving that file in lockstep. Either way
// the session is the same function, so the SDK issues the same calls in the
// same order, and the scans it hands back are written to a .sdkdump beside
// the capture:
//
//     <stem>.sdkdump          from the live run
//     <stem>.replay.sdkdump   from a replay
//
// Two checks follow from that, and they test different things:
//
//   * live vs replay -- does the replay reproduce what the SDK saw live?
//     python/tools/lidar_sdkcheck.py compares the two dumps.
//   * SDK vs Python  -- does the SDK decode the same bytes to the same
//     numbers as this repository's decoder? lidar_sdkcheck.py again, against
//     the .rpraw. Legacy and express only: the Python library has no ultra decoder.
//
// The dump holds the nodes exactly as grabScanDataHq() returns them, before
// ascendScanData() rewrites any angles.
//
// This file includes SDK headers directly. Probes are allowed to; production
// code under cpp/src/ is not -- the channels it uses live below the boundary.

#include <cinttypes>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <sl_lidar.h>

#include "lidar/ReplayChannel.h"
#include "lidar/Rpraw.h"
#include "lidar/TapChannel.h"

namespace {

const char *kDefaultPort = "/dev/rplidar";  // never ttyUSB0: enumeration order
constexpr std::int32_t kBaud = 115200;
constexpr std::size_t kMaxNodes = 8192;  // the SDK's own demo uses 8192
constexpr sl_u32 kGrabTimeoutMs = 3000;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

// Owns a FILE so every return path closes it.
struct FileCloser {
  void operator()(std::FILE *file) const { std::fclose(file); }
};
using FilePtr = std::unique_ptr<std::FILE, FileCloser>;

struct Session {
  sl_u16 mode{1};
  std::int32_t scans{40};
};

struct Options {
  Session session;
  std::string tag{"sdk"};
  std::string port{kDefaultPort};
  std::string replay;  // empty = live
  double speed{1.0};
};

[[noreturn]] void usage() {
  std::fprintf(stderr, "Usage:\n"
                       "  lidar_record [--mode N] [--scans K] [--tag NAME] [--port DEV]\n"
                       "  lidar_record --replay FILE.rpraw [--speed X]\n");
  std::exit(2);
}

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {  // int: matches argc
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
      }
      return argv[++i];
    };
    if (arg == "--mode") {
      options.session.mode = static_cast<sl_u16>(std::stoi(next()));
    } else if (arg == "--scans") {
      options.session.scans = static_cast<std::int32_t>(std::stoi(next()));
    } else if (arg == "--tag") {
      options.tag = next();
    } else if (arg == "--port") {
      options.port = next();
    } else if (arg == "--replay") {
      options.replay = next();
    } else if (arg == "--speed") {
      options.speed = std::stod(next());
    } else {
      usage();
    }
  }
  return options;
}

// The recording's header carries the session parameters, so a replay runs
// exactly the session that was recorded without anyone retyping them. A
// targeted lookup rather than a JSON parser: this file writes the header, so
// its shape is known.
std::int32_t getHeaderInt(const std::string &json, const char *key, std::int32_t fallback) {
  const std::string needle = std::string("\"") + key + "\":";
  const std::size_t at = json.find(needle);
  if (at == std::string::npos) {
    return fallback;
  }
  return static_cast<std::int32_t>(std::atoi(json.c_str() + at + needle.size()));
}

std::string utcStamp() {
  const std::time_t now = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&now, &tm);
  char buf[32];
  std::strftime(buf, sizeof buf, "%Y%m%dT%H%M%SZ", &tm);
  return buf;
}

std::string stemOf(const std::string &path) {
  const std::string ext = ".rpraw";
  if (path.size() > ext.size() &&
      path.compare(path.size() - ext.size(), ext.size(), ext) == 0) {
    return path.substr(0, path.size() - ext.size());
  }
  return path;
}

// THE session. Live and replay both run this and nothing else against the
// driver: lockstep replay only works if the SDK makes identical calls.
std::int32_t runSession(sl::ILidarDriver &lidar, const Session &session, std::FILE *dump) {
  // connect() is not a connectivity test (see LidarInfo.cpp); a query is.
  sl_lidar_response_device_info_t info;
  if (SL_IS_FAIL(lidar.getDeviceInfo(info))) {
    std::fprintf(stderr, "ERROR: no answer to GET_INFO\n");
    return 1;
  }
  std::printf("  device          model 0x%02X, firmware %u.%02u, hardware %u\n", info.model,
              info.firmware_version >> 8, info.firmware_version & 0xFF, info.hardware_version);

  sl::LidarScanMode used{};
  sl_result res = lidar.startScanExpress(false, session.mode, 0, &used);
  if (SL_IS_FAIL(res)) {
    std::fprintf(stderr, "ERROR: startScanExpress(mode %u) failed (0x%08" PRIX32 ")\n",
                 session.mode, static_cast<std::uint32_t>(res));
    return 1;
  }
  std::printf("  scan mode       %u %s, %.1f us/sample, answer type 0x%02X\n", used.id,
              used.scan_mode, static_cast<double>(used.us_per_sample), used.ans_type);
  std::fprintf(dump, "# mode %u %s ans_type 0x%02X\n", used.id, used.scan_mode, used.ans_type);

  std::vector<sl_lidar_response_measurement_node_hq_t> nodes(kMaxNodes);
  std::int32_t grabbed = 0;
  std::int32_t status = 0;
  while (grabbed < session.scans && s_stopRequested == 0) {
    std::size_t count = nodes.size();
    sl_u64 tUs = 0;
    res = lidar.grabScanDataHqWithTimeStamp(nodes.data(), count, tUs, kGrabTimeoutMs);
    if (SL_IS_FAIL(res)) {
      std::fprintf(stderr, "ERROR: grab %" PRId32 " failed (0x%08" PRIX32 ")\n", grabbed,
                   static_cast<std::uint32_t>(res));
      status = 1;
      break;
    }
    std::fprintf(dump, "scan %" PRId32 " %zu %" PRIu64 "\n", grabbed, count,
                 static_cast<std::uint64_t>(tUs));
    for (std::size_t i = 0; i < count; ++i) {
      const sl_lidar_response_measurement_node_hq_t &node = nodes[i];
      std::fprintf(dump, "%u %u %u %u\n", node.angle_z_q14, node.dist_mm_q2, node.quality,
                   node.flag);
    }
    ++grabbed;
  }
  std::printf("  scans grabbed   %" PRId32 " of %" PRId32 "%s\n", grabbed, session.scans,
              s_stopRequested != 0 ? " (interrupted)" : "");

  lidar.stop();
  // On the A1, stop() does not stop the motor: that is DTR, via
  // setMotorSpeed(0). A no-op against a replay, which reports a non-serial
  // channel type and so is never sent DTR.
  lidar.setMotorSpeed(0);
  return status;
}

std::int32_t record(const Options &options) {
  const std::string stem = "captures/" + options.tag + "-sdk" +
                           std::to_string(options.session.mode) + "_" + utcStamp();
  const std::string rawPath = stem + ".rpraw";
  const std::string dumpPath = stem + ".sdkdump";

  std::printf("  port            %s @ %" PRId32 " 8N1\n", options.port.c_str(), kBaud);
  std::printf("  recording to    %s\n", rawPath.c_str());

  // Python's RawReader requires a JSON first line, and the replay reads the
  // session back out of it.
  const std::string header =
      std::string("{\"type\":\"header\",\"schema\":1,\"capture_id\":\"") +
      stem.substr(std::strlen("captures/")) + "\",\"probe\":\"lidar_record\"," +
      "\"recorder\":\"sdk-tap\",\"sdk_version\":\"" + std::string(SL_LIDAR_SDK_VERSION) +
      "\"," + "\"port\":\"" + options.port + "\",\"baud\":" + std::to_string(kBaud) + "," +
      "\"session_mode\":" + std::to_string(options.session.mode) + "," +
      "\"session_scans\":" + std::to_string(options.session.scans) + "}";

  sl::Result<sl::IChannel *> serial = sl::createSerialPortChannel(options.port, kBaud);
  if (!serial) {
    std::fprintf(stderr, "ERROR: cannot create a channel for %s\n", options.port.c_str());
    return 1;
  }
  // Adopting a pointer the SDK allocated, so make_unique does not apply.
  std::unique_ptr<sl::IChannel> serialOwner(*serial);
  sl::ISerialPortChannel *serialPort = dynamic_cast<sl::ISerialPortChannel *>(serialOwner.get());
  if (serialPort == nullptr) {
    std::fprintf(stderr, "ERROR: the SDK's serial channel is not an ISerialPortChannel\n");
    return 1;
  }

  Heron::Lidar::RprawWriter writer(rawPath, header);
  Heron::Lidar::TapChannel tap(*serialPort, writer);

  sl::Result<sl::ILidarDriver *> driver = sl::createLidarDriver();
  if (!driver) {
    std::fprintf(stderr, "ERROR: createLidarDriver failed\n");
    return 1;
  }
  std::unique_ptr<sl::ILidarDriver> lidar(*driver);  // adopted, as above

  FilePtr dump(std::fopen(dumpPath.c_str(), "w"));
  if (!dump) {
    std::fprintf(stderr, "ERROR: cannot create %s\n", dumpPath.c_str());
    return 1;
  }

  std::int32_t status = 1;
  if (SL_IS_OK(lidar->connect(&tap))) {
    status = runSession(*lidar, options.session, dump.get());
    lidar->disconnect();
  } else {
    std::fprintf(stderr, "ERROR: connect failed\n");
  }
  dump.reset();
  writer.close();

  std::printf("  wrote           %s\n", dumpPath.c_str());
  return status;
}

std::int32_t replay(const Options &options) {
  Heron::Lidar::Rpraw raw = Heron::Lidar::readRpraw(options.replay);
  const std::int32_t mode = getHeaderInt(raw.headerJson, "session_mode", -1);
  Session session;
  session.scans = getHeaderInt(raw.headerJson, "session_scans", -1);
  if (mode < 0 || session.scans < 0) {
    std::fprintf(stderr,
                 "ERROR: %s was not recorded by lidar_record "
                 "(no session in its header)\n",
                 options.replay.c_str());
    return 1;
  }
  session.mode = static_cast<sl_u16>(mode);

  const std::string dumpPath = stemOf(options.replay) + ".replay.sdkdump";
  std::printf("  replaying       %s (%zu chunks%s), speed %.2fx\n", options.replay.c_str(),
              raw.chunks.size(), raw.truncated ? ", truncated" : "", options.speed);

  Heron::Lidar::ReplayChannel channel(std::move(raw.chunks), options.speed);

  sl::Result<sl::ILidarDriver *> driver = sl::createLidarDriver();
  if (!driver) {
    std::fprintf(stderr, "ERROR: createLidarDriver failed\n");
    return 1;
  }
  std::unique_ptr<sl::ILidarDriver> lidar(*driver);  // adopting the SDK's pointer

  FilePtr dump(std::fopen(dumpPath.c_str(), "w"));
  if (!dump) {
    std::fprintf(stderr, "ERROR: cannot create %s\n", dumpPath.c_str());
    return 1;
  }

  std::int32_t status = 1;
  if (SL_IS_OK(lidar->connect(&channel))) {
    status = runSession(*lidar, session, dump.get());
    lidar->disconnect();
  }
  dump.reset();

  std::printf("  writes matched  %zu of %zu\n", channel.getWritesMatched(),
              channel.getWritesRecorded());
  const std::string divergence = channel.getDivergence();
  if (!divergence.empty()) {
    std::printf("  DIVERGED        %s\n", divergence.c_str());
    status = 1;
  } else if (channel.getWritesMatched() != channel.getWritesRecorded()) {
    std::printf("  INCOMPLETE      the SDK stopped before making every recorded write\n");
    status = 1;
  }
  std::printf("  wrote           %s\n", dumpPath.c_str());
  return status;
}

}  // namespace

int main(int argc, char **argv) {
  // Ctrl-C ends the grab loop, so the session still stops the scan and the
  // motor. Without this a killed run leaves the motor spinning.
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  try {
    const Options options = parse(argc, argv);

    std::printf("========================================\n");
    std::printf(" Probe C2 -- SDK session %s\n", options.replay.empty() ? "record" : "replay");
    std::printf("========================================\n\n");
    std::printf("  sdk version     %s\n", SL_LIDAR_SDK_VERSION);

    const std::int32_t status = options.replay.empty() ? record(options) : replay(options);
    std::printf("\n  %s\n", status == 0 ? "OK" : "FAILED");
    return status;
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
}
