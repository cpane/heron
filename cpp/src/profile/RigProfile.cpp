// File        : RigProfile.cpp
// Author      : Chris Pane
// Description : Reading and checking a rig's profile.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Two passes. The first splits the text into sections of key = value
// entries, keeping each line number, and rejects anything that is not a
// comment, a header or an entry. The second walks the entries against the
// table of known sections and keys, converting and range-checking each
// value, then checks nothing required is missing. Every error names the
// file and the line.

#include "heron/profile/RigProfile.h"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace Heron::Profile {

namespace {

constexpr std::string_view kTopLevel = "";
constexpr std::string_view kRig = "rig";
constexpr std::string_view kMotor = "motor";
constexpr std::string_view kMotorCalibration = "motor.calibration";
constexpr std::size_t kMaxRigNameLength = 32;

struct Entry {
  std::string key;
  std::string value;
  std::int32_t line{0};
};

struct Section {
  std::string name;  // "" for the lines before the first header
  std::int32_t line{0};
  std::vector<Entry> entries;
};

// Throws with the file and, when line is not 0, the line.
[[noreturn]] void fail(const std::string &source, std::int32_t line, const std::string &what) {
  std::string where = source;
  if (line > 0) {
    where += ":" + std::to_string(line);
  }
  throw std::runtime_error("Heron::Profile::parseRigProfile() : " + where + ": " + what);
}

std::string_view trim(std::string_view text) {
  const auto isSpace = [](char c) { return std::isspace(static_cast<unsigned char>(c)) != 0; };
  while (!text.empty() && isSpace(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && isSpace(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

// Lower-case names: a letter, then letters, digits, '_' and (for section
// names) '.'.
bool isName(std::string_view text, bool allowDot) {
  if (text.empty() || std::islower(static_cast<unsigned char>(text.front())) == 0) {
    return false;
  }
  return std::all_of(text.begin(), text.end(), [allowDot](char c) {
    const auto u = static_cast<unsigned char>(c);
    return std::islower(u) != 0 || std::isdigit(u) != 0 || c == '_' || (allowDot && c == '.');
  });
}

std::string sectionLabel(std::string_view name) {
  return name.empty() ? std::string("the top of the file") : "[" + std::string(name) + "]";
}

// -- pass 1: lines to sections -------------------------------------------------

std::vector<Section> split(std::string_view text, const std::string &source) {
  std::vector<Section> sections;
  sections.push_back(Section{std::string(kTopLevel), 0, {}});
  std::set<std::string, std::less<>> seen;

  std::int32_t lineNumber = 0;
  while (!text.empty()) {
    ++lineNumber;
    const std::size_t end = text.find('\n');
    std::string_view line = text.substr(0, end);
    text.remove_prefix(end == std::string_view::npos ? text.size() : end + 1);

    line = trim(line);  // also drops a Windows '\r'
    if (line.empty() || line.front() == '#') {
      continue;
    }

    if (line.front() == '[') {
      if (line.back() != ']') {
        fail(source, lineNumber, "a section header must end with ']'");
      }
      const std::string_view name = trim(line.substr(1, line.size() - 2));
      if (!isName(name, true)) {
        fail(source, lineNumber, "'" + std::string(line) + "' is not a valid section header");
      }
      if (!seen.insert(std::string(name)).second) {
        fail(source, lineNumber, "[" + std::string(name) + "] appears twice");
      }
      sections.push_back(Section{std::string(name), lineNumber, {}});
      continue;
    }

    const std::size_t equals = line.find('=');
    if (equals == std::string_view::npos) {
      fail(source, lineNumber, "expected 'key = value' or '[section]', got '" + std::string(line) + "'");
    }
    const std::string_view key = trim(line.substr(0, equals));
    const std::string_view value = trim(line.substr(equals + 1));
    if (!isName(key, false)) {
      fail(source, lineNumber, "'" + std::string(key) + "' is not a valid key");
    }
    if (value.empty()) {
      fail(source, lineNumber, "'" + std::string(key) + "' has no value");
    }
    Section &current = sections.back();
    for (const Entry &earlier : current.entries) {
      if (earlier.key == key) {
        fail(source, lineNumber,
             "'" + std::string(key) + "' appears twice in " + sectionLabel(current.name) +
                 " (first on line " + std::to_string(earlier.line) + ")");
      }
    }
    current.entries.push_back(Entry{std::string(key), std::string(value), lineNumber});
  }
  return sections;
}

// -- pass 2: values --------------------------------------------------------------

std::int64_t toInteger(const Entry &entry, std::string_view token, std::int64_t min,
                       std::int64_t max, const std::string &source) {
  std::int64_t value = 0;
  const auto [end, error] = std::from_chars(token.data(), token.data() + token.size(), value);
  if (error != std::errc() || end != token.data() + token.size()) {
    fail(source, entry.line,
         "'" + entry.key + "': '" + std::string(token) + "' is not a whole number");
  }
  if (value < min || value > max) {
    fail(source, entry.line,
         "'" + entry.key + "': " + std::to_string(value) + " is outside " + std::to_string(min) +
             ".." + std::to_string(max));
  }
  return value;
}

std::vector<std::string_view> tokens(std::string_view text) {
  std::vector<std::string_view> out;
  while (true) {
    text = trim(text);
    if (text.empty()) {
      return out;
    }
    std::size_t end = 0;
    while (end < text.size() && std::isspace(static_cast<unsigned char>(text[end])) == 0) {
      ++end;
    }
    out.push_back(text.substr(0, end));
    text.remove_prefix(end);
  }
}

// Four values for the four motors.
std::vector<std::string_view> fourTokens(const Entry &entry, const std::string &source) {
  const auto out = tokens(entry.value);
  if (out.size() != kProfileMotorCount) {
    fail(source, entry.line,
         "'" + entry.key + "' needs " + std::to_string(kProfileMotorCount) +
             " values, one per motor; it has " + std::to_string(out.size()));
  }
  return out;
}

std::array<std::uint8_t, kProfileMotorCount> fourIntegers(const Entry &entry, std::int64_t min,
                                                          std::int64_t max,
                                                          const std::string &source) {
  const auto parts = fourTokens(entry, source);
  std::array<std::uint8_t, kProfileMotorCount> out{};
  for (std::size_t i = 0; i < kProfileMotorCount; ++i) {
    out[i] = static_cast<std::uint8_t>(toInteger(entry, parts[i], min, max, source));
  }
  return out;
}

void readTopLevel(const Section &section, bool &formatSeen, const std::string &source) {
  for (const Entry &entry : section.entries) {
    if (entry.key != "format") {
      fail(source, entry.line,
           "unknown key '" + entry.key + "' before the first section; only 'format' goes there");
    }
    const std::int64_t format = toInteger(entry, entry.value, 0, 1000, source);
    if (format != kProfileFormat) {
      fail(source, entry.line,
           "format " + std::to_string(format) + " is not supported; this program reads format " +
               std::to_string(kProfileFormat));
    }
    formatSeen = true;
  }
}

void readRig(const Section &section, RigProfile &profile, const std::string &source) {
  for (const Entry &entry : section.entries) {
    if (entry.key == "name") {
      if (!isValidRigName(entry.value)) {
        fail(source, entry.line,
             "'" + entry.value + "' is not a valid rig name: up to " +
                 std::to_string(kMaxRigNameLength) + " of a-z, 0-9 and '-'");
      }
      profile.name = entry.value;
    } else if (entry.key == "platform") {
      if (std::find(kKnownPlatforms.begin(), kKnownPlatforms.end(), entry.value) ==
          kKnownPlatforms.end()) {
        std::string known;
        for (const std::string_view platform : kKnownPlatforms) {
          known += (known.empty() ? "" : ", ") + std::string(platform);
        }
        fail(source, entry.line, "unknown platform '" + entry.value + "'; known: " + known);
      }
      profile.platform = entry.value;
    } else {
      fail(source, entry.line, "unknown key '" + entry.key + "' in [rig]");
    }
  }
}

void readMotor(const Section &section, MotorProfile &motor, const std::string &source) {
  for (const Entry &entry : section.entries) {
    if (entry.key == "terminal") {
      motor.terminal = fourIntegers(entry, 1, kProfileMotorCount, source);
      std::array<bool, kProfileMotorCount + 1> used{};
      for (const std::uint8_t terminal : motor.terminal) {
        if (used[terminal]) {
          fail(source, entry.line,
               "'terminal': " + std::to_string(terminal) +
                   " is used twice; each terminal drives one motor");
        }
        used[terminal] = true;
      }
    } else if (entry.key == "sign") {
      const auto parts = fourTokens(entry, source);
      for (std::size_t i = 0; i < kProfileMotorCount; ++i) {
        if (parts[i] == "+") {
          motor.sign[i] = 1;
        } else if (parts[i] == "-") {
          motor.sign[i] = -1;
        } else {
          fail(source, entry.line,
               "'sign': '" + std::string(parts[i]) + "' must be + or -");
        }
      }
    } else if (entry.key == "ramp_per_ms") {
      motor.rampPerMs = static_cast<std::uint16_t>(toInteger(entry, entry.value, 0, kMaxRampPerMs, source));
    } else if (entry.key == "min_duty_forward") {
      motor.minDutyForward = fourIntegers(entry, 0, 255, source);
    } else if (entry.key == "min_duty_backward") {
      motor.minDutyBackward = fourIntegers(entry, 0, 255, source);
    } else {
      fail(source, entry.line, "unknown key '" + entry.key + "' in [motor]");
    }
  }
}

// Every key a section needs; a missing one is reported at the section's
// header line.
void requireKeys(const Section &section, std::initializer_list<std::string_view> keys,
                 const std::string &source) {
  for (const std::string_view key : keys) {
    const bool present = std::any_of(section.entries.begin(), section.entries.end(),
                                     [key](const Entry &e) { return e.key == key; });
    if (!present) {
      fail(source, section.line,
           "[" + section.name + "] is missing '" + std::string(key) + "'");
    }
  }
}

}  // namespace

RigProfile parseRigProfile(std::string_view text, const std::string &source) {
  const std::vector<Section> sections = split(text, source);

  RigProfile profile;
  profile.source = source;
  bool formatSeen = false;
  const Section *rig = nullptr;
  const Section *motor = nullptr;

  for (const Section &section : sections) {
    if (section.name == kTopLevel) {
      readTopLevel(section, formatSeen, source);
    } else if (section.name == kRig) {
      readRig(section, profile, source);
      rig = &section;
    } else if (section.name == kMotor) {
      readMotor(section, profile.motor, source);
      motor = &section;
    } else if (section.name == kMotorCalibration) {
      for (const Entry &entry : section.entries) {
        profile.motorCalibration.emplace_back(entry.key, entry.value);
      }
    } else {
      fail(source, section.line,
           "unknown section [" + section.name + "]; known: [rig], [motor], [motor.calibration]");
    }
  }

  if (!formatSeen) {
    fail(source, 0, "'format = " + std::to_string(kProfileFormat) +
                        "' is missing; it goes before the first section");
  }
  if (rig == nullptr) {
    fail(source, 0, "the [rig] section is missing");
  }
  if (motor == nullptr) {
    fail(source, 0, "the [motor] section is missing");
  }
  requireKeys(*rig, {"name", "platform"}, source);
  requireKeys(*motor, {"terminal", "sign", "ramp_per_ms", "min_duty_forward", "min_duty_backward"},
              source);
  return profile;
}

RigProfile loadRigProfile(const std::filesystem::path &file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Heron::Profile::loadRigProfile() : cannot read " + file.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  RigProfile profile = parseRigProfile(text.str(), file.string());

  if (profile.name != file.stem().string()) {
    throw std::runtime_error("Heron::Profile::loadRigProfile() : " + file.string() +
                             ": its [rig] name is '" + profile.name +
                             "', but a profile must be named after its rig: " + profile.name +
                             ".conf");
  }
  return profile;
}

bool isValidRigName(std::string_view name) {
  if (name.empty() || name.size() > kMaxRigNameLength || name.front() == '-') {
    return false;
  }
  return std::all_of(name.begin(), name.end(), [](char c) {
    const auto u = static_cast<unsigned char>(c);
    return std::islower(u) != 0 || std::isdigit(u) != 0 || c == '-';
  });
}

std::string readRigName(const std::filesystem::path &file) {
  const std::string hint = "set it from the host with: scripts/set_rig.sh <user>@<pi> <rig>";
  std::ifstream in(file);
  if (!in) {
    throw std::runtime_error("Heron::Profile::readRigName() : " + file.string() +
                             " not found, so this Pi's rig is not known; " + hint);
  }
  std::ostringstream text;
  text << in.rdbuf();
  const std::string_view name = trim(text.str());
  if (!isValidRigName(name)) {
    throw std::runtime_error("Heron::Profile::readRigName() : " + file.string() +
                             " must hold one rig name; " + hint);
  }
  return std::string(name);
}

std::filesystem::path heronHome() {
  const char *home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    throw std::runtime_error("Heron::Profile::heronHome() : $HOME is not set");
  }
  return std::filesystem::path(home) / "heron";
}

std::filesystem::path findRigProfile(const std::filesystem::path &home) {
  return home / "profiles" / (readRigName(home / "rig") + ".conf");
}

}  // namespace Heron::Profile
