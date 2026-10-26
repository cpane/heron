// File        : RigProfileTool.cpp
// Author      : Chris Pane
// Description : rig_profile: checks profile files, and shows a rig's profile.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The command-line face of heron_profile (include/heron/profile/RigProfile.h).
// On the host it checks a profile after an edit, before it is committed; a
// ctest runs "check" on every file in profiles/, so a broken profile fails
// the tests. On a Pi it shows which rig the Pi is and what its profile says.
//
//   build-host/bin/rig_profile check profiles/*.conf
//   build-host/bin/rig_profile show profiles/bench.conf
//   scripts/deploy_cpp.sh <user>@<pi> rig_profile show      (this Pi's rig)
//
// check prints one line per file and exits 1 if any is broken; show prints
// every value, as the programs that use the profile will read it.

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

#include "heron/profile/RigProfile.h"

namespace {

using namespace Heron::Profile;

[[noreturn]] void usage(int status) {
  std::fprintf(status == 0 ? stdout : stderr,
               "Usage: rig_profile check FILE...   check profile files\n"
               "       rig_profile show [FILE]     show a profile; default this Pi's,\n"
               "                                   ~/heron/profiles/<the rig in ~/heron/rig>.conf\n");
  std::exit(status);
}

int runCheck(const std::vector<std::string> &files) {
  if (files.empty()) {
    usage(2);
  }
  int broken = 0;  // int: the exit status
  for (const std::string &file : files) {
    try {
      const RigProfile profile = loadRigProfile(file);
      std::printf("OK      %s  (rig %s, platform %s)\n", file.c_str(), profile.name.c_str(),
                  profile.platform.c_str());
    } catch (const std::exception &e) {
      std::printf("BROKEN  %s\n        %s\n", file.c_str(), e.what());
      ++broken;
    }
  }
  return broken == 0 ? 0 : 1;
}

// One row of the motor table: a label, then a column per motor.
void printFour(const char *label, const auto &values) {
  std::printf("  %-19s", label);
  for (const auto value : values) {
    std::printf("%12d", static_cast<int>(value));
  }
  std::printf("\n");
}

int runShow(const std::vector<std::string> &files) {
  if (files.size() > 1) {
    usage(2);
  }
  const std::filesystem::path file = files.empty() ? findRigProfile() : std::filesystem::path(files[0]);
  const RigProfile profile = loadRigProfile(file);

  std::printf("%s\n\n", profile.source.c_str());
  std::printf("rig       %s\nplatform  %s\n\n", profile.name.c_str(), profile.platform.c_str());
  std::printf("  %-19s%12s%12s%12s%12s\n", "motor", "1", "2", "3", "4");
  std::printf("  %-19s%12s%12s%12s%12s\n", "", "front-left", "front-right", "back-right",
              "back-left");
  printFour("terminal (M1..M4)", profile.motor.terminal);
  std::printf("  %-19s", "sign");
  for (const auto sign : profile.motor.sign) {
    std::printf("%12s", sign < 0 ? "-" : "+");
  }
  std::printf("\n");
  printFour("min duty forward", profile.motor.minDutyForward);
  printFour("min duty backward", profile.motor.minDutyBackward);
  std::printf("  %-19s%u per ms (0 to full in %u ms)\n", "ramp", profile.motor.rampPerMs,
              profile.motor.rampPerMs == 0 ? 0U : 1000U / profile.motor.rampPerMs);
  if (!profile.motorCalibration.empty()) {
    std::printf("\ncalibration\n");
    for (const auto &[key, value] : profile.motorCalibration) {
      std::printf("  %-19s%s\n", key.c_str(), value.c_str());
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    usage(2);
  }
  const std::string command = argv[1];
  if (command == "-h" || command == "--help") {
    usage(0);
  }
  const std::vector<std::string> files(argv + 2, argv + argc);
  try {
    if (command == "check") {
      return runCheck(files);
    }
    if (command == "show") {
      return runShow(files);
    }
  } catch (const std::exception &e) {
    std::fprintf(stderr, "ERROR: %s\n", e.what());
    return 1;
  }
  std::fprintf(stderr, "ERROR: unknown command '%s'\n", command.c_str());
  usage(2);
}
