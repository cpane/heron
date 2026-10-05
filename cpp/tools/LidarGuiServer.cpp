// File        : LidarGuiServer.cpp
// Author      : Chris Pane
// Description : Serves the Lidar adapter's scans to the existing viewer,
//               python/tools/lidar_gui.py, over its newline-JSON protocol.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh cpane@<pi> lidar_gui_server
//     .venv/bin/python python/tools/lidar_gui.py --host <pi> --connect
//
// Or entirely on the host, from a recording:
//
//     build-host/bin/lidar_gui_server --replay captures/<stem>.rpraw
//     .venv/bin/python python/tools/lidar_gui.py --host 127.0.0.1 --connect
//
// The protocol is python/heron/link/wire.py, version 1. The C++ adapter
// publishes robot-frame scans, which the Python server never had, so scan
// messages carry additive optional fields (frame, rpts, coverage_ok,
// unobserved, dropped) that the updated viewer draws; see wire.py.
//
// One client at a time. The motor runs only while scanning: the adapter has
// no separate motor control, by design (docs/lidar-cpp-design.md, §2.5).
//
// Uses only the public LiDAR headers, plus POSIX sockets, which the standard
// library does not provide.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "heron/lidar/Lidar.h"

namespace {

using Heron::Clock;
using Heron::Lidar::Lidar;
using Heron::Lidar::LidarConfig;
using Heron::Lidar::LidarState;

volatile std::sig_atomic_t s_stopRequested = 0;

extern "C" void onSignal(int) {
  s_stopRequested = 1;
}

constexpr std::int32_t kProtocolVersion = 1;  // wire.PROTOCOL_VERSION
constexpr auto kPublishTick = std::chrono::milliseconds(20);
constexpr double kRadToCdeg = 18000.0 / 3.14159265358979;

/**
 * This unit's five scan modes, as lidar_info reported them (2026-10-03).
 * The SDK decodes all five; the ultra ones (0x84) are flagged because nothing
 * independent has checked that decode yet.
 */
struct ModeEntry {
  std::int32_t id;
  const char *name;
  double usPerSample;
  std::int32_t answerType;
};
constexpr ModeEntry kModes[] = {
    {0, "Standard", 508.0, 0x81},
    {1, "Express", 254.0, 0x82},
    {2, "Boost (ultra, decode unchecked)", 127.0, 0x84},
    {3, "Sensitivity (ultra, decode unchecked)", 127.0, 0x84},
    {4, "Stability (ultra, decode unchecked)", 201.0, 0x84},
};

struct Options {
  std::uint16_t tcpPort{5555};
  std::string device{"/dev/rplidar"};
  std::string replay;
};

// -- minimal JSON: enough for this protocol, no library ----------------------

std::string jsonString(const std::string &text) {
  std::string out = "\"";
  for (const char c : text) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<std::uint8_t>(c) < 0x20) {
      char esc[8];
      std::snprintf(esc, sizeof esc, "\\u%04" PRIx32,
                    static_cast<std::uint32_t>(static_cast<std::uint8_t>(c)));
      out += esc;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

// Finds `"key":` in a flat JSON object, tolerating whitespace. The commands
// this server reads are tiny objects from the viewer, so a targeted lookup is
// enough; anything it cannot read is treated as absent.
std::string::size_type findValue(const std::string &json, const std::string &key) {
  const std::string needle = "\"" + key + "\"";
  std::string::size_type at = json.find(needle);
  if (at == std::string::npos) {
    return at;
  }
  at = json.find(':', at + needle.size());
  if (at == std::string::npos) {
    return at;
  }
  at = json.find_first_not_of(" \t", at + 1);
  return at;
}

std::string readString(const std::string &json, const std::string &key) {
  const std::string::size_type at = findValue(json, key);
  if (at == std::string::npos || json[at] != '"') {
    return "";
  }
  const std::string::size_type end = json.find('"', at + 1);
  return end == std::string::npos ? "" : json.substr(at + 1, end - at - 1);
}

std::int32_t readInt(const std::string &json, const std::string &key, std::int32_t fallback) {
  const std::string::size_type at = findValue(json, key);
  if (at == std::string::npos) {
    return fallback;
  }
  return static_cast<std::int32_t>(std::strtol(json.c_str() + at, nullptr, 10));
}

bool readBool(const std::string &json, const std::string &key) {
  const std::string::size_type at = findValue(json, key);
  return at != std::string::npos && json.compare(at, 4, "true") == 0;
}

// -- one viewer connection -----------------------------------------------------

/**
 * Serves one connected viewer until it disconnects or the server stops.
 */
class Session {
  public:
    Session(int fd, const Options &options) : m_fd(fd), m_options(options) {}
    ~Session();

    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    void run();

  private:
    void send(const std::string &line);
    void sendHello();
    void sendStatus(const std::string &detail);
    void sendScan(const Heron::Scan &scan);
    void handle(const std::string &command);
    void startScan(std::int32_t mode);
    void stopScan();
    void publish(std::stop_token stop);
    std::shared_ptr<Lidar> currentLidar() const;

    const int m_fd;  // int: the POSIX socket API
    const Options m_options;
    std::mutex m_sendMutex;
    mutable std::mutex m_lidarMutex;  // guards m_lidar and m_mode
    std::shared_ptr<Lidar> m_lidar;
    std::int32_t m_mode{1};
    Heron::Lidar::LidarInfo m_lastInfo;
    std::atomic<bool> m_failureReported{false};
    std::jthread m_publisher;  // declared last: joined first
};

Session::~Session() {
  if (m_publisher.joinable()) {
    m_publisher.request_stop();
    m_publisher.join();
  }
  stopScan();
  ::close(m_fd);
}

void Session::send(const std::string &line) {
  std::lock_guard<std::mutex> lock(m_sendMutex);
  const std::string framed = line + "\n";
  std::size_t sent = 0;
  while (sent < framed.size()) {
    const ssize_t n = ::send(m_fd, framed.data() + sent, framed.size() - sent, 0);
    if (n <= 0) {
      return;  // the reader notices the closed socket and ends the session
    }
    sent += static_cast<std::size_t>(n);
  }
}

std::shared_ptr<Lidar> Session::currentLidar() const {
  std::lock_guard<std::mutex> lock(m_lidarMutex);
  return m_lidar;
}

void Session::sendHello() {
  const std::shared_ptr<Lidar> lidar = currentLidar();
  if (lidar && lidar->getHealth().state == LidarState::Running) {
    m_lastInfo = lidar->getInfo();
  }
  const Heron::Lidar::LidarInfo &info = m_lastInfo;

  std::string modes = "[";
  std::string decodable = "[";
  bool firstMode = true;
  for (const ModeEntry &mode : kModes) {
    // A replay can only play the mode it recorded.
    if (!m_options.replay.empty() && mode.id != m_mode) {
      continue;
    }
    const char *kind = mode.answerType == 0x81   ? "\"legacy\""
                       : mode.answerType == 0x82 ? "\"express\""
                                                 : "\"ultra\"";
    char entry[256];
    std::snprintf(entry, sizeof entry,
                  "{\"id\":%" PRId32 ",\"name\":%s,\"us_per_sample\":%.1f,\"max_distance_m\":12.0,"
                  "\"ans_type\":%" PRId32 ",\"kind\":%s}",
                  mode.id, jsonString(mode.name).c_str(), mode.usPerSample, mode.answerType, kind);
    modes += (firstMode ? "" : ",") + std::string(entry);
    decodable += (firstMode ? "" : ",") + std::to_string(mode.id);
    firstMode = false;
  }
  modes += "]";
  decodable += "]";

  const std::string port = m_options.replay.empty() ? m_options.device : m_options.replay;
  const std::string firmware =
      info.firmware.empty() ? "? (starts with the first scan)" : info.firmware;
  send("{\"type\":\"hello\",\"version\":" + std::to_string(kProtocolVersion) +
       ",\"device\":{\"model\":" + std::to_string(info.model) + ",\"firmware\":" +
       jsonString(firmware) + ",\"hardware\":" + std::to_string(info.hardware) +
       ",\"serial\":" + jsonString(info.serialHex) + "}" +
       ",\"health\":{\"status\":0,\"state\":\"not queried by the C++ adapter\"}" +
       ",\"modes\":" + modes + ",\"decodable\":" + decodable +
       ",\"typical_mode\":" + std::to_string(m_mode) + ",\"port\":" + jsonString(port) +
       ",\"server\":\"cpp-adapter\"}");
}

void Session::sendStatus(const std::string &detail) {
  const std::shared_ptr<Lidar> lidar = currentLidar();
  const bool running = lidar && lidar->getHealth().state == LidarState::Running;
  std::int32_t mode = 0;
  {
    std::lock_guard<std::mutex> lock(m_lidarMutex);
    mode = m_mode;
  }
  send(std::string("{\"type\":\"status\",\"motor\":") + (running ? "true" : "false") +
       ",\"scanning\":" + (running ? "true" : "false") +
       ",\"mode\":" + (running ? std::to_string(mode) : "null") + ",\"detail\":" +
       jsonString(detail) + "}");
}

void Session::sendScan(const Heron::Scan &scan) {
  std::string line;
  line.reserve(48 + scan.points.size() * 16);
  char head[320];
  const auto ns = [](Heron::TimePoint t) {
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
  };
  const auto rounded = [](double v) { return static_cast<std::int64_t>(std::llround(v)); };
  const double gapDeg = static_cast<double>(scan.worstSamplingGapRad) * 180.0 / 3.14159265358979;
  std::int32_t mode = 0;
  {
    std::lock_guard<std::mutex> lock(m_lidarMutex);
    mode = m_mode;
  }
  std::snprintf(head, sizeof head,
                "{\"type\":\"scan\",\"seq\":%" PRIu64 ",\"mode\":%" PRId32
                ",\"t_start_ns\":%" PRId64 ",\"t_end_ns\":%" PRId64 ",\"total\":%" PRIu32
                ",\"max_gap_q6\":%" PRId64 ",\"frame\":\"robot\",\"coverage_ok\":%s"
                ",\"dropped\":%" PRIu32 ",\"pts\":[],\"rpts\":[",
                scan.seq, mode, ns(scan.tStart), ns(scan.tEnd), scan.rawSampleCount,
                rounded(gapDeg * 64.0), scan.isCoverageOk() ? "true" : "false",
                scan.droppedEstimate);
  line += head;
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    char point[48];
    std::snprintf(point, sizeof point, "%s[%" PRId64 ",%" PRId64 "]", i ? "," : "",
                  rounded(static_cast<double>(scan.points[i].bearingRad) * kRadToCdeg),
                  rounded(static_cast<double>(scan.points[i].rangeM) * 1000.0));
    line += point;
  }
  line += "],\"unobserved\":[";
  for (std::size_t i = 0; i < scan.unobserved.size(); ++i) {
    char arc[48];
    std::snprintf(arc, sizeof arc, "%s[%" PRId64 ",%" PRId64 "]", i ? "," : "",
                  rounded(static_cast<double>(scan.unobserved[i].startRad) * kRadToCdeg),
                  rounded(static_cast<double>(scan.unobserved[i].widthRad) * kRadToCdeg));
    line += arc;
  }
  line += "]}";
  send(line);
}

void Session::startScan(std::int32_t mode) {
  stopScan();
  LidarConfig config;
  config.port = m_options.device;
  config.scanMode = mode;
  config.replayFile = m_options.replay;
  auto lidar = std::make_shared<Lidar>(config);
  const bool ok = lidar->start();
  {
    std::lock_guard<std::mutex> lock(m_lidarMutex);
    m_mode = mode;
    m_lidar = lidar;
  }
  m_failureReported = false;
  if (ok) {
    sendHello();  // now with the device's own identity
    sendStatus("scanning mode " + std::to_string(mode) + " through the C++ adapter");
  } else {
    send("{\"type\":\"error\",\"message\":" + jsonString(lidar->getHealth().lastError) +
         ",\"fatal\":false}");
    sendStatus("start failed");
  }
}

void Session::stopScan() {
  std::shared_ptr<Lidar> lidar;
  {
    std::lock_guard<std::mutex> lock(m_lidarMutex);
    lidar.swap(m_lidar);
  }
  if (lidar) {
    lidar->stop();  // ~1 s; outside the lock so publishing never waits on it
  }
}

void Session::handle(const std::string &command) {
  const std::string name = readString(command, "cmd");
  if (name == "hello" || name == "refresh") {
    sendHello();
    sendStatus("");
  } else if (name == "scan_start") {
    startScan(readInt(command, "mode", 1));
  } else if (name == "scan_stop") {
    stopScan();
    sendStatus("idle");
  } else if (name == "motor") {
    if (!readBool(command, "on")) {
      stopScan();
      sendStatus("motor off");
    } else {
      sendStatus("the C++ adapter runs the motor only while scanning: start a scan");
    }
  } else if (name == "reset") {
    stopScan();
    sendStatus("reset is not offered by the C++ adapter; stopped instead");
  } else {
    send("{\"type\":\"error\",\"message\":" + jsonString("unknown command " + name) +
         ",\"fatal\":false}");
  }
}

void Session::publish(std::stop_token stop) {
  std::uint64_t lastSeq = 0;
  while (!stop.stop_requested()) {
    std::this_thread::sleep_for(kPublishTick);
    const std::shared_ptr<Lidar> lidar = currentLidar();
    if (!lidar) {
      lastSeq = 0;
      continue;
    }
    const Heron::Lidar::LidarHealth health = lidar->getHealth();
    if (health.state == LidarState::Failed && !m_failureReported.exchange(true)) {
      send("{\"type\":\"error\",\"message\":" + jsonString(health.lastError) + ",\"fatal\":false}");
      sendStatus("the adapter stopped on an error");
      continue;
    }
    const std::shared_ptr<const Heron::Scan> scan = lidar->getLatest();
    if (scan && scan->seq != lastSeq) {
      lastSeq = scan->seq;
      sendScan(*scan);
    }
  }
}

void Session::run() {
  sendHello();
  sendStatus("connected to the C++ adapter");
  m_publisher = std::jthread([this](std::stop_token stop) { publish(stop); });

  std::string buffer;
  char chunk[4096];
  while (s_stopRequested == 0) {
    pollfd pfd{m_fd, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, 250);
    if (ready < 0 && errno != EINTR) {
      break;
    }
    if (ready <= 0) {
      continue;
    }
    const ssize_t n = ::recv(m_fd, chunk, sizeof chunk, 0);
    if (n <= 0) {
      break;  // the viewer closed the connection
    }
    buffer.append(chunk, static_cast<std::size_t>(n));
    std::string::size_type newline;
    while ((newline = buffer.find('\n')) != std::string::npos) {
      const std::string line = buffer.substr(0, newline);
      buffer.erase(0, newline + 1);
      if (!line.empty()) {
        handle(line);
      }
    }
  }
}

// -- the listening socket --------------------------------------------------------

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {  // int: matches argc
    const std::string arg = argv[i];
    if (arg == "--tcp-port" && i + 1 < argc) {
      options.tcpPort = static_cast<std::uint16_t>(std::atoi(argv[++i]));
    } else if (arg == "--device" && i + 1 < argc) {
      options.device = argv[++i];
    } else if (arg == "--replay" && i + 1 < argc) {
      options.replay = argv[++i];
    } else {
      std::fprintf(stderr, "Usage: lidar_gui_server [--tcp-port N] [--device PATH] "
                           "[--replay FILE.rpraw]\n");
      std::exit(2);
    }
  }
  return options;
}

}  // namespace

int main(int argc, char **argv) {
  const Options options = parse(argc, argv);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  std::signal(SIGPIPE, SIG_IGN);  // a vanished viewer is a closed socket, not a crash

  const int listener = ::socket(AF_INET, SOCK_STREAM, 0);
  const int one = 1;
  ::setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  address.sin_port = htons(options.tcpPort);
  if (::bind(listener, reinterpret_cast<sockaddr *>(&address), sizeof address) != 0 ||
      ::listen(listener, 1) != 0) {
    std::fprintf(stderr, "ERROR: cannot listen on port %" PRIu16 ": %s\n", options.tcpPort,
                 std::strerror(errno));
    return 1;
  }

  std::printf("========================================\n");
  std::printf(" LiDAR viewer server (C++ adapter)\n");
  std::printf("========================================\n\n");
  std::printf("  source     %s\n", options.replay.empty() ? options.device.c_str()
                                                         : options.replay.c_str());
  std::printf("  listening  0.0.0.0:%" PRIu16 ", one viewer at a time\n", options.tcpPort);
  std::printf("  Ctrl-C to stop\n\n");
  std::fflush(stdout);

  while (s_stopRequested == 0) {
    pollfd pfd{listener, POLLIN, 0};
    if (::poll(&pfd, 1, 500) <= 0) {
      continue;
    }
    sockaddr_in peer{};
    socklen_t peerLength = sizeof peer;
    const int fd = ::accept(listener, reinterpret_cast<sockaddr *>(&peer), &peerLength);
    if (fd < 0) {
      continue;
    }
    char who[INET_ADDRSTRLEN] = "?";
    ::inet_ntop(AF_INET, &peer.sin_addr, who, sizeof who);
    std::printf("  viewer connected from %s\n", who);
    std::fflush(stdout);
    {
      Session session(fd, options);
      session.run();
    }  // stops the scan and the motor before the next viewer
    std::printf("  viewer disconnected; sensor idle\n");
    std::fflush(stdout);
  }
  ::close(listener);
  std::printf("\n  stopped\n");
  return 0;
}
