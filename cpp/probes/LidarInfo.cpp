// File        : LidarInfo.cpp
// Author      : Chris Pane
// Description : Probe C1. Does the SDK build, link, and talk to the device?
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

//     scripts/deploy_cpp.sh cpane@<pi> lidar_info
//
// The C++ counterpart of python/probes/lidar_01_port.py and the scan-mode
// half of lidar_06_capabilities.py. It proved the toolchain end to end before
// the adapter existed, and remains the quickest check that the build, the
// link and the device are all sound.
//
// Deliberately does not spin the motor or start a scan. Identity, health and
// the scan-mode table are all answerable with the head stationary, which
// makes this safe to run without watching the robot.
//
// This file includes SDK headers directly. Probes are allowed to; production
// code under cpp/src/ is not. See docs/lidar-cpp-design.md.

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <sl_lidar.h>

namespace {

const char *kDefaultPort = "/dev/rplidar";  // never ttyUSB0: enumeration order
constexpr std::int32_t kDefaultBaud = 115200;

// The findings document records which of this unit's five modes carry a
// payload this project can decode. Label them, so the table below says
// something useful rather than just echoing the device.
const char *describeAnsType(sl_u8 ansType) {
  switch (ansType) {
    case SL_LIDAR_ANS_TYPE_MEASUREMENT:
      return "legacy 0x81";
    case SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED:
      return "express 0x82";
    case SL_LIDAR_ANS_TYPE_MEASUREMENT_HQ:
      return "hq 0x83";
    case SL_LIDAR_ANS_TYPE_MEASUREMENT_CAPSULED_ULTRA:
      return "ultra 0x84";
    case SL_LIDAR_ANS_TYPE_MEASUREMENT_DENSE_CAPSULED:
      return "dense 0x85";
    default:
      return "unknown";
  }
}

void printDeviceInfo(const sl_lidar_response_device_info_t &info) {
  // Both byte orders, deliberately, the same way probe 1 does it.
  //
  // They disagree, and nothing here can settle which is right. The SDK's own
  // demo (app/ultra_simple/main.cpp) prints serialnum[0..15] forward;
  // python/heron/lidar/protocol.py reverses it, and that reversed form is
  // what docs/rplidar-a1m8-findings.md records. Only the sticker on the
  // housing decides, and that check has not been done -- so print both and
  // let the reader compare, rather than pick one and look authoritative.
  std::string forward;
  std::string reversed;
  forward.reserve(32);
  reversed.reserve(32);
  for (std::size_t i = 0; i < 16; ++i) {
    char hex[3];
    std::snprintf(hex, sizeof hex, "%02X", info.serialnum[i]);
    forward += hex;
    std::snprintf(hex, sizeof hex, "%02X", info.serialnum[15 - i]);
    reversed += hex;
  }

  std::printf("  model           0x%02X\n", info.model);
  std::printf("  firmware        %u.%02u\n", info.firmware_version >> 8,
              info.firmware_version & 0xFF);
  std::printf("  hardware        %u\n", info.hardware_version);
  std::printf("  serial (fwd)    %s   <- as the SDK demo prints it\n", forward.c_str());
  std::printf("  serial (rev)    %s   <- as this repo's Python prints it\n", reversed.c_str());
  std::printf("                  compare against the housing label; the\n");
  std::printf("                  findings document records the reversed form\n");
}

// Everything after connect(). Separate from main() so that every early return
// leaves the driver to be disconnected and freed in exactly one place.
std::int32_t queryDevice(sl::ILidarDriver &lidar, const char *port) {
  // A real round trip, treated as the connectivity test, because connect()
  // is not one.
  //
  // sl_async_transceiver.cpp openChannelAndBind() declares `u_result ans`
  // and then shadows it with `Result<nullptr_t> ans` inside its do-block.
  // The open() failure is assigned to the shadowing inner variable, so the
  // function returns the outer RESULT_OK regardless -- and connect() then
  // sets _isConnected = true, so isConnected() agrees. Verified by reading
  // the source and by running this probe with no device attached: connect
  // succeeded and only the queries failed.
  //
  // So: ask the device something and require an answer.
  sl_lidar_response_device_info_t info;
  const sl_result res = lidar.getDeviceInfo(info);
  if (SL_IS_FAIL(res)) {
    std::fflush(stdout);
    std::fprintf(stderr,
                 "ERROR: no response from %s (0x%08" PRIX32 ")\n"
                 "  The device did not answer GET_INFO. Note that connect()\n"
                 "  reports success even when the port cannot be opened, so\n"
                 "  this is the first point at which a missing, busy or\n"
                 "  unpowered device shows up.\n"
                 "  Check the device is attached, that %s exists, and that\n"
                 "  you are in the 'dialout' group.\n",
                 port, static_cast<std::uint32_t>(res), port);
    return 1;
  }
  printDeviceInfo(info);

  std::int32_t status = 0;

  sl_lidar_response_device_health_t health;
  if (SL_IS_OK(lidar.getHealth(health))) {
    const char *state = (health.status == SL_LIDAR_STATUS_OK)        ? "Good"
                        : (health.status == SL_LIDAR_STATUS_WARNING) ? "Warning"
                        : (health.status == SL_LIDAR_STATUS_ERROR)   ? "Error"
                                                                     : "Unknown";
    std::printf("  health          %s (status %u, error 0x%04X)\n", state, health.status,
                health.error_code);
  } else {
    std::fflush(stdout);
    std::fprintf(stderr, "  WARNING: getHealth failed\n");
    status = 1;
  }

  std::vector<sl::LidarScanMode> modes;
  sl_u16 typical = 0;
  if (SL_IS_OK(lidar.getAllSupportedScanModes(modes)) &&
      SL_IS_OK(lidar.getTypicalScanMode(typical))) {
    std::printf("\n  scan modes      %zu (device recommends %u)\n\n", modes.size(), typical);
    std::printf("    id  name                us/sample   max range   streams\n");
    for (const sl::LidarScanMode &mode : modes) {
      std::printf("    %2u%s %-18s %8.2f    %6.1f m   %s\n", mode.id,
                  (mode.id == typical) ? " *" : "  ", mode.scan_mode,
                  static_cast<double>(mode.us_per_sample),
                  static_cast<double>(mode.max_distance), describeAnsType(mode.ans_type));
    }
    std::printf("\n    * = the mode the device recommends. Note it streams\n");
    std::printf("        ultra capsules; the SDK decodes them, but nothing\n");
    std::printf("        independent has checked that -- see docs/rplidar-a1m8-findings.md\n");
  } else {
    std::fflush(stdout);
    std::fprintf(stderr, "  WARNING: scan mode query failed\n");
    status = 1;
  }
  return status;
}

}  // namespace

int main(int argc, char **argv) {
  const char *port = (argc > 1) ? argv[1] : kDefaultPort;

  std::printf("========================================\n");
  std::printf(" Probe C1 -- SDK link and device identity\n");
  std::printf("========================================\n\n");
  std::printf("  sdk version     %s\n", SL_LIDAR_SDK_VERSION);
  std::printf("  port            %s @ %" PRId32 " 8N1\n\n", port, kDefaultBaud);

  sl::Result<sl::IChannel *> channel = sl::createSerialPortChannel(port, kDefaultBaud);
  if (!channel) {
    std::fprintf(stderr,
                 "ERROR: cannot open %s (0x%08" PRIX32 ")\n"
                 "  Check the device is attached, that /dev/rplidar exists,\n"
                 "  and that you are in the 'dialout' group.\n",
                 port, static_cast<std::uint32_t>(channel.err));
    return 1;
  }
  // Adopting pointers the SDK allocated, so make_unique does not apply. The
  // driver is declared after the channel so it is destroyed first.
  std::unique_ptr<sl::IChannel> channelOwner(*channel);

  sl::Result<sl::ILidarDriver *> driver = sl::createLidarDriver();
  if (!driver) {
    std::fprintf(stderr, "ERROR: createLidarDriver failed (0x%08" PRIX32 ")\n",
                 static_cast<std::uint32_t>(driver.err));
    return 1;
  }
  std::unique_ptr<sl::ILidarDriver> lidar(*driver);

  // The SDK returns sl_result codes rather than throwing, so every call is
  // checked; there is no exception to catch.
  const sl_result res = lidar->connect(channelOwner.get());
  if (SL_IS_FAIL(res)) {
    std::fprintf(stderr, "ERROR: connect failed (0x%08" PRIX32 ")\n",
                 static_cast<std::uint32_t>(res));
    return 1;
  }

  const std::int32_t status = queryDevice(*lidar, port);
  lidar->disconnect();

  std::printf("\n  %s\n", status == 0 ? "All queries answered." : "Some queries failed.");
  return status;
}
