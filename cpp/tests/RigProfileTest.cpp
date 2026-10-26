// File        : RigProfileTest.cpp
// Author      : Chris Pane
// Description : Reading and checking rig profiles: a good file, and each rule
//               a bad one can break.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Most cases parse text held here, changed one line at a time from a good
// profile, and check that the error names the right line and problem. The
// file functions (loading, the rig name) use a scratch directory under the
// system's temporary directory. Same style as the other tests: no framework.

#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include "heron/profile/RigProfile.h"

namespace {

using namespace Heron::Profile;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

const std::string kGood =
    "# a comment\n"
    "format = 1\n"
    "\n"
    "[rig]\n"
    "name = bench\n"
    "platform = uno-l293d\n"
    "\n"
    "[motor]\n"
    "terminal = 1 2 4 3\n"
    "sign = + - + +\n"
    "ramp_per_ms = 4\n"
    "min_duty_forward = 76 81 87 92\n"
    "min_duty_backward = 71 76 76 102\n"
    "\n"
    "[motor.calibration]\n"
    "date = 2026-10-25\n"
    "method = sweep # with a hash, kept\n";

// kGood with one line replaced: line is 1-based, as in error messages.
std::string withLine(std::int32_t line, const std::string &replacement) {
  std::string out;
  std::int32_t number = 0;
  std::size_t start = 0;
  while (start < kGood.size()) {
    const std::size_t end = kGood.find('\n', start);
    ++number;
    out += (number == line) ? replacement : kGood.substr(start, end - start);
    out += '\n';
    start = end + 1;
  }
  return out;
}

// The error message parsing text gives, or "" if it parses.
std::string errorOf(const std::string &text) {
  try {
    (void)parseRigProfile(text, "test.conf");
    return "";
  } catch (const std::runtime_error &e) {
    return e.what();
  }
}

bool contains(const std::string &text, const std::string &part) {
  return text.find(part) != std::string::npos;
}

// The error names the line and says what is wrong.
void checkError(const char *what, const std::string &text, const std::string &where,
                const std::string &part) {
  const std::string error = errorOf(text);
  const bool ok = contains(error, "test.conf" + where + ": ") && contains(error, part);
  check(what, ok);
  if (!ok) {
    std::printf("        got: %s\n", error.empty() ? "(no error)" : error.c_str());
  }
}

void writeFile(const std::filesystem::path &path, const std::string &text) {
  std::ofstream(path, std::ios::binary) << text;
}

}  // namespace

int main() {
  std::printf("A good profile\n");

  {
    const RigProfile p = parseRigProfile(kGood, "test.conf");
    check("rig and platform", p.name == "bench" && p.platform == "uno-l293d");
    check("terminal, in protocol order", p.motor.terminal[0] == 1 && p.motor.terminal[2] == 4 &&
                                             p.motor.terminal[3] == 3);
    check("sign: + is 1, - is -1", p.motor.sign[0] == 1 && p.motor.sign[1] == -1);
    check("ramp", p.motor.rampPerMs == 4);
    check("min duty, both directions",
          p.motor.minDutyForward[3] == 92 && p.motor.minDutyBackward[3] == 102);
    check("calibration record kept in order, '#' inside a value kept",
          p.motorCalibration.size() == 2 && p.motorCalibration[0].first == "date" &&
              p.motorCalibration[1].second == "sweep # with a hash, kept");
    check("source kept for messages", p.source == "test.conf");
  }

  {
    std::string crlf;
    for (const char c : kGood) {
      if (c == '\n') {
        crlf += '\r';
      }
      crlf += c;
    }
    check("Windows line endings read the same", errorOf(crlf).empty());
    check("spaces around keys and values are trimmed",
          errorOf(withLine(11, "   ramp_per_ms    =    4   ")).empty());
    check("no calibration section is fine",
          errorOf(kGood.substr(0, kGood.find("[motor.calibration]"))).empty());
  }

  std::printf("Lines that are not entries\n");

  checkError("a line with no '='", withLine(11, "ramp_per_ms 4"), ":11", "expected 'key = value'");
  checkError("an empty value", withLine(11, "ramp_per_ms ="), ":11", "has no value");
  checkError("a bad key", withLine(11, "Ramp = 4"), ":11", "not a valid key");
  checkError("an unclosed header", withLine(8, "[motor"), ":8", "must end with ']'");

  std::printf("Strictness\n");

  checkError("an unknown section", withLine(15, "[geometry]"), ":15", "unknown section [geometry]");
  checkError("an unknown key", withLine(11, "ramp_per_sec = 4"), ":11",
             "unknown key 'ramp_per_sec' in [motor]");
  checkError("an unknown key in [rig]", withLine(6, "plaform = uno-l293d"), ":6",
             "unknown key 'plaform' in [rig]");
  checkError("a key twice", withLine(12, "ramp_per_ms = 5"), ":12", "appears twice");
  checkError("a section twice", withLine(15, "[motor]"), ":15", "[motor] appears twice");
  checkError("a key missing", withLine(11, "# none"), ":8", "[motor] is missing 'ramp_per_ms'");
  checkError("[rig] missing", withLine(4, "# none") , ":5", "unknown key 'name' before the first section");
  checkError("format missing", withLine(2, "# none"), "", "'format = 1' is missing");
  checkError("format 2", withLine(2, "format = 2"), ":2", "format 2 is not supported");
  checkError("format inside a section", withLine(7, "format = 1"), ":7", "unknown key 'format' in [rig]");

  std::printf("Values\n");

  checkError("an unknown platform", withLine(6, "platform = arduino"), ":6", "unknown platform 'arduino'");
  checkError("a bad rig name", withLine(5, "name = Bench"), ":5", "not a valid rig name");
  checkError("three terminals", withLine(9, "terminal = 1 2 3"), ":9", "needs 4 values");
  checkError("a terminal of 5", withLine(9, "terminal = 1 2 3 5"), ":9", "5 is outside 1..4");
  checkError("a terminal twice", withLine(9, "terminal = 1 2 2 3"), ":9", "2 is used twice");
  checkError("a sign that is not + or -", withLine(10, "sign = + + 1 +"), ":10", "must be + or -");
  checkError("a duty over 255", withLine(12, "min_duty_forward = 76 81 87 256"), ":12",
             "256 is outside 0..255");
  checkError("a negative duty", withLine(13, "min_duty_backward = -1 76 76 102"), ":13",
             "-1 is outside 0..255");
  checkError("not a number", withLine(11, "ramp_per_ms = four"), ":11", "'four' is not a whole number");
  checkError("a number with junk after it", withLine(11, "ramp_per_ms = 4ms"), ":11",
             "'4ms' is not a whole number");
  checkError("a ramp too fast", withLine(11, "ramp_per_ms = 2001"), ":11", "outside 0..2000");
  check("a ramp of 0 (none) is allowed", errorOf(withLine(11, "ramp_per_ms = 0")).empty());

  std::printf("Rig names\n");

  check("lower case, digits and '-'", isValidRigName("bench") && isValidRigName("mini-2") &&
                                          isValidRigName("2nd"));
  check("not empty, upper case, '_', a leading '-' or over 32",
        !isValidRigName("") && !isValidRigName("Bench") && !isValidRigName("mini_2") &&
            !isValidRigName("-mini") && !isValidRigName(std::string(33, 'a')));

  std::printf("Files\n");

  {
    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() / "heron-rig-profile-test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "profiles");

    writeFile(dir / "profiles" / "bench.conf", kGood);
    check("load: reads and checks a file",
          loadRigProfile(dir / "profiles" / "bench.conf").motor.rampPerMs == 4);

    writeFile(dir / "profiles" / "mini.conf", kGood);
    bool nameChecked = false;
    try {
      (void)loadRigProfile(dir / "profiles" / "mini.conf");
    } catch (const std::runtime_error &e) {
      nameChecked = contains(e.what(), "named after its rig: bench.conf");
    }
    check("load: the rig name must match the file name", nameChecked);

    bool missing = false;
    try {
      (void)loadRigProfile(dir / "profiles" / "none.conf");
    } catch (const std::runtime_error &e) {
      missing = contains(e.what(), "cannot read");
    }
    check("load: a missing file says so", missing);

    bool noRig = false;
    try {
      (void)findRigProfile(dir);
    } catch (const std::runtime_error &e) {
      noRig = contains(e.what(), "scripts/set_rig.sh");
    }
    check("no rig file: the error says how to set it", noRig);

    writeFile(dir / "rig", "bench\n");
    check("the rig file names the profile",
          findRigProfile(dir) == dir / "profiles" / "bench.conf" &&
              readRigName(dir / "rig") == "bench");

    writeFile(dir / "rig", "bench mini\n");
    bool twoNames = false;
    try {
      (void)readRigName(dir / "rig");
    } catch (const std::runtime_error &e) {
      twoNames = contains(e.what(), "must hold one rig name");
    }
    check("a rig file with two names is refused", twoNames);

    std::filesystem::remove_all(dir);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
