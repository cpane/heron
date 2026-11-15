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
// file and the line. The four sections that hold only numbers
// ([geometry], [lidar], [odometry], [navigation]) share one reader,
// readReals(), driven by a table of key, range and destination each.

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
constexpr std::string_view kGeometry = "geometry";
constexpr std::string_view kLidar = "lidar";
constexpr std::string_view kOdometry = "odometry";
constexpr std::string_view kNavigation = "navigation";
constexpr std::string_view kCalibrationSuffix = ".calibration";
constexpr std::string_view kSurfacePrefix = "surface.";
constexpr std::size_t kMaxRigNameLength = 32;

// The OTOS's scalar register holds 0.872..1.127 (Odometry::OtosConfig).
constexpr double kMinOtosScalar = 0.872;
constexpr double kMaxOtosScalar = 1.127;

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
    // A section header may also hold '-', for a surface's name
    // ([surface.hardwood-1]); a key may not.
    return std::islower(u) != 0 || std::isdigit(u) != 0 || c == '_' ||
           (allowDot && (c == '.' || c == '-'));
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
    } else if (entry.key == "drive" || entry.key == "surface") {
      if (!isValidRigName(entry.value)) {
        fail(source, entry.line,
             "'" + entry.value + "' is not a valid " + entry.key + " name: up to " +
                 std::to_string(kMaxRigNameLength) + " of a-z, 0-9 and '-'");
      }
      (entry.key == "drive" ? profile.drive : profile.surface) = entry.value;
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

// A number with a fraction: an optional sign, digits, at most one '.', with
// a digit before it. Checked by hand so strtod's extras (1e3, inf, hex, .5)
// are refused, as a whole number's junk is.
double toReal(const Entry &entry, double min, double max, const std::string &source) {
  const std::string_view text = entry.value;
  std::size_t i = (!text.empty() && (text[0] == '-' || text[0] == '+')) ? 1 : 0;
  const std::size_t firstDigit = i;
  bool dot = false;
  bool ok = i < text.size();
  for (; ok && i < text.size(); ++i) {
    if (text[i] == '.') {
      ok = !dot && i > firstDigit;
      dot = true;
    } else {
      ok = std::isdigit(static_cast<unsigned char>(text[i])) != 0;
    }
  }
  if (!ok) {
    fail(source, entry.line, "'" + entry.key + "': '" + entry.value + "' is not a number");
  }
  const double value = std::strtod(entry.value.c_str(), nullptr);
  if (value < min || value > max) {
    std::ostringstream range;
    range << min << ".." << max;
    fail(source, entry.line,
         "'" + entry.key + "': " + entry.value + " is outside " + range.str());
  }
  return value;
}

// One key of a section that holds only numbers, and where its value goes.
struct RealKey {
  std::string_view key;
  double min;
  double max;
  double *value;
};

void readReals(const Section &section, std::initializer_list<RealKey> keys,
               const std::string &source) {
  for (const Entry &entry : section.entries) {
    const auto known = std::find_if(keys.begin(), keys.end(),
                                    [&entry](const RealKey &k) { return k.key == entry.key; });
    if (known == keys.end()) {
      fail(source, entry.line, "unknown key '" + entry.key + "' in [" + section.name + "]");
    }
    *known->value = toReal(entry, known->min, known->max, source);
  }
}

GeometryProfile readGeometry(const Section &section, const std::string &source) {
  GeometryProfile g;
  readReals(section,
            {{"length", 0.05, 3.0, &g.length},
             {"width", 0.05, 3.0, &g.width},
             {"max_speed", 0.01, 5.0, &g.maxSpeed},
             {"min_speed", 0.0, 5.0, &g.minSpeed},
             {"max_turn_rate", 0.01, 20.0, &g.maxTurnRate},
             {"accel", 0.01, 10.0, &g.accel},
             {"brake", 0.01, 20.0, &g.brake}},
            source);
  return g;
}

LidarProfile readLidar(const Section &section, const std::string &source) {
  LidarProfile l;
  readReals(section,
            {{"x", -2.0, 2.0, &l.x},
             {"y", -2.0, 2.0, &l.y},
             {"yaw_deg", -180.0, 180.0, &l.yawDeg},
             {"height", 0.0, 2.0, &l.height}},
            source);
  return l;
}

OdometryProfile readOdometry(const Section &section, const std::string &source) {
  OdometryProfile o;
  readReals(section,
            {{"x", -2.0, 2.0, &o.x},
             {"y", -2.0, 2.0, &o.y},
             {"yaw_deg", -180.0, 180.0, &o.yawDeg},
             {"linear_scalar", kMinOtosScalar, kMaxOtosScalar, &o.linearScalar},
             {"angular_scalar", kMinOtosScalar, kMaxOtosScalar, &o.angularScalar}},
            source);
  return o;
}

NavigationProfile readNavigation(const Section &section, const std::string &source) {
  NavigationProfile n;
  readReals(section,
            {{"stop_margin", 0.0, 1.0, &n.stopMargin},
             {"slow_margin", 0.0, 2.0, &n.slowMargin}},
            source);
  return n;
}

// "p" and digits: a motion table point's key.
bool isPointKey(std::string_view key) {
  return key.size() > 1 && key[0] == 'p' &&
         std::all_of(key.begin() + 1, key.end(),
                     [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; });
}

SurfaceProfile readSurface(const Section &section, const std::string &name, const std::string &source) {
  SurfaceProfile surface;
  surface.name = name;
  surface.source = source;
  for (const Entry &entry : section.entries) {
    const auto real = [&](double min, double max) { return toReal(entry, min, max, source); };
    if (entry.key == "max_speed") {
      surface.maxSpeed = real(0.01, 5.0);
    } else if (entry.key == "min_speed") {
      surface.minSpeed = real(0.0, 5.0);
    } else if (entry.key == "max_turn_rate") {
      surface.maxTurnRate = real(0.01, 20.0);
    } else if (entry.key == "brake") {
      surface.brake = real(0.01, 20.0);
    } else if (entry.key == "linear_scalar") {
      surface.linearScalar = real(kMinOtosScalar, kMaxOtosScalar);
    } else if (entry.key == "angular_scalar") {
      surface.angularScalar = real(kMinOtosScalar, kMaxOtosScalar);
    } else if (entry.key == "mirror") {
      if (entry.value != "0" && entry.value != "1") {
        fail(source, entry.line, "'mirror': '" + entry.value + "' must be 0 or 1");
      }
      surface.mirror = entry.value == "1";
    } else if (isPointKey(entry.key)) {
      // <direction deg> <size> <vx> <vy> <wz>, each checked as a number.
      std::vector<std::string> words;
      std::istringstream in(entry.value);
      for (std::string w; in >> w;) {
        words.push_back(w);
      }
      if (words.size() != 5) {
        fail(source, entry.line,
             "'" + entry.key + "': five numbers, <direction deg> <size> <vx> <vy> <wz>, not '" +
                 entry.value + "'");
      }
      const auto word = [&](std::size_t k, double min, double max) {
        return toReal(Entry{entry.key, words[k], entry.line}, min, max, source);
      };
      const SurfacePoint point{word(0, -180.0, 180.0), word(1, 0.0, 1.5), word(2, -5.0, 5.0),
                               word(3, -5.0, 5.0), word(4, -20.0, 20.0)};
      if (!(point.size > 0.0)) {
        fail(source, entry.line, "'" + entry.key + "': a size must be above 0");
      }
      surface.points.push_back(point);
    } else {
      fail(source, entry.line, "unknown key '" + entry.key + "' in [" + section.name + "]");
    }
  }
  return surface;
}

// "surface.<name>" or "surface.<name>.calibration": the name, and whether
// it is the record. No value if the section is not a surface's.
std::optional<std::pair<std::string, bool>> surfaceSection(const Section &section,
                                                           const std::string &source) {
  const std::string_view full = section.name;
  if (full.substr(0, kSurfacePrefix.size()) != kSurfacePrefix) {
    return std::nullopt;
  }
  std::string_view rest = full.substr(kSurfacePrefix.size());
  bool calibration = false;
  if (rest.size() > kCalibrationSuffix.size() &&
      rest.substr(rest.size() - kCalibrationSuffix.size()) == kCalibrationSuffix) {
    rest.remove_suffix(kCalibrationSuffix.size());
    calibration = true;
  }
  if (!isValidRigName(rest)) {
    fail(source, section.line,
         "[" + section.name + "]: '" + std::string(rest) + "' is not a valid surface name: up to " +
             std::to_string(kMaxRigNameLength) + " of a-z, 0-9 and '-'");
  }
  return std::make_pair(std::string(rest), calibration);
}

// Each surface's record joined to its surface after the loop, so the two
// may come in either order. A record without its surface fails.
void joinSurfaceRecords(std::vector<SurfaceProfile> &surfaces,
                        const std::vector<std::pair<const Section *, std::string>> &records,
                        const std::string &source) {
  for (const auto &[section, name] : records) {
    const auto found = std::find_if(surfaces.begin(), surfaces.end(),
                                    [&name](const SurfaceProfile &s) { return s.name == name; });
    if (found == surfaces.end()) {
      fail(source, section->line,
           "[" + section->name + "] has no [surface." + name + "] section to record");
    }
    for (const Entry &entry : section->entries) {
      found->calibration.emplace_back(entry.key, entry.value);
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
  const Section *geometry = nullptr;
  const Section *lidar = nullptr;
  const Section *odometry = nullptr;
  const Section *navigation = nullptr;

  // The sections that may have a [<name>.calibration], and where it goes.
  const std::array<std::pair<std::string_view, CalibrationRecord *>, 5> calibrations = {{
      {kMotor, &profile.motorCalibration},
      {kGeometry, &profile.geometryCalibration},
      {kLidar, &profile.lidarCalibration},
      {kOdometry, &profile.odometryCalibration},
      {kNavigation, &profile.navigationCalibration},
  }};

  std::vector<std::pair<const Section *, std::string>> surfaceRecords;
  for (const Section &section : sections) {
    if (const auto surface = surfaceSection(section, source)) {
      if (surface->second) {
        surfaceRecords.emplace_back(&section, surface->first);
      } else {
        profile.surfaces.push_back(readSurface(section, surface->first, source));
      }
    } else if (section.name == kTopLevel) {
      readTopLevel(section, formatSeen, source);
    } else if (section.name == kRig) {
      readRig(section, profile, source);
      rig = &section;
    } else if (section.name == kMotor) {
      readMotor(section, profile.motor, source);
      motor = &section;
    } else if (section.name == kGeometry) {
      profile.geometry = readGeometry(section, source);
      geometry = &section;
    } else if (section.name == kLidar) {
      profile.lidar = readLidar(section, source);
      lidar = &section;
    } else if (section.name == kOdometry) {
      profile.odometry = readOdometry(section, source);
      odometry = &section;
    } else if (section.name == kNavigation) {
      profile.navigation = readNavigation(section, source);
      navigation = &section;
    } else {
      const auto record = std::find_if(
          calibrations.begin(), calibrations.end(), [&section](const auto &c) {
            return section.name == std::string(c.first) + std::string(kCalibrationSuffix);
          });
      if (record == calibrations.end()) {
        fail(source, section.line,
             "unknown section [" + section.name +
                 "]; known: [rig], [motor], [geometry], [lidar], [odometry], [navigation], "
                 "each but [rig] with a [<section>.calibration], and [surface.<name>]");
      }
      for (const Entry &entry : section.entries) {
        record->second->emplace_back(entry.key, entry.value);
      }
    }
  }

  joinSurfaceRecords(profile.surfaces, surfaceRecords, source);

  // A calibration record needs its section: checked after the loop, so the
  // two may come in either order.
  for (const Section &section : sections) {
    for (const auto &calibration : calibrations) {
      const std::string_view parent = calibration.first;
      if (section.name != std::string(parent) + std::string(kCalibrationSuffix)) {
        continue;
      }
      const bool parentSeen = std::any_of(sections.begin(), sections.end(),
                                          [parent](const Section &s) { return s.name == parent; });
      if (!parentSeen) {
        fail(source, section.line,
             "[" + section.name + "] has no [" + std::string(parent) + "] section to record");
      }
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
  if (geometry != nullptr) {
    // min_speed is optional: 0 is speed in proportion to the command.
    requireKeys(*geometry, {"length", "width", "max_speed", "max_turn_rate", "accel", "brake"},
                source);
    if (profile.geometry->minSpeed >= profile.geometry->maxSpeed) {
      fail(source, geometry->line,
           "[geometry] min_speed must be below max_speed: it is the slowest the robot moves");
    }
  }
  if (lidar != nullptr) {
    requireKeys(*lidar, {"x", "y", "yaw_deg", "height"}, source);
  }
  if (odometry != nullptr) {
    requireKeys(*odometry, {"x", "y", "yaw_deg", "linear_scalar", "angular_scalar"}, source);
  }
  if (navigation != nullptr) {
    requireKeys(*navigation, {"stop_margin", "slow_margin"}, source);
    if (profile.navigation->slowMargin < profile.navigation->stopMargin) {
      fail(source, navigation->line,
           "[navigation] slow_margin must be at least stop_margin: the slow zone lies outside "
           "the stop zone");
    }
  }
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

  // The drive's library of known surfaces, after the rig's own, which win
  // on a name; in name order, so the result does not depend on the disk.
  if (!profile.drive.empty()) {
    const std::filesystem::path library = file.parent_path() / "surfaces" / profile.drive;
    std::error_code ec;
    if (std::filesystem::is_directory(library, ec)) {
      std::vector<std::filesystem::path> files;
      for (const auto &item : std::filesystem::directory_iterator(library)) {
        if (item.path().extension() == ".conf") {
          files.push_back(item.path());
        }
      }
      std::sort(files.begin(), files.end());
      for (const std::filesystem::path &path : files) {
        std::ifstream surfaceIn(path, std::ios::binary);
        std::ostringstream surfaceText;
        surfaceText << surfaceIn.rdbuf();
        SurfaceProfile surface = parseSurfaceFile(surfaceText.str(), path.string());
        if (surface.name != path.stem().string()) {
          throw std::runtime_error("Heron::Profile::loadRigProfile() : " + path.string() +
                                   ": its surface is '" + surface.name +
                                   "', but a surface file must be named after it: " +
                                   surface.name + ".conf");
        }
        if (findSurface(profile, surface.name) == nullptr) {
          profile.surfaces.push_back(std::move(surface));
        }
      }
    }
  }
  if (!profile.surface.empty() && findSurface(profile, profile.surface) == nullptr) {
    throw std::runtime_error("Heron::Profile::loadRigProfile() : " + file.string() +
                             ": [rig] surface is '" + profile.surface +
                             "', but neither the profile nor the library of drive '" +
                             profile.drive + "' has it");
  }
  return profile;
}

SurfaceProfile parseSurfaceFile(std::string_view text, const std::string &source) {
  const std::vector<Section> sections = split(text, source);
  bool formatSeen = false;
  std::vector<SurfaceProfile> surfaces;
  std::vector<std::pair<const Section *, std::string>> records;
  for (const Section &section : sections) {
    if (const auto surface = surfaceSection(section, source)) {
      if (surface->second) {
        records.emplace_back(&section, surface->first);
      } else {
        surfaces.push_back(readSurface(section, surface->first, source));
      }
    } else if (section.name == kTopLevel) {
      readTopLevel(section, formatSeen, source);
    } else {
      fail(source, section.line,
           "[" + section.name + "] in a surface file: only one [surface.<name>] and its record");
    }
  }
  if (!formatSeen) {
    fail(source, 0, "'format = " + std::to_string(kProfileFormat) +
                        "' is missing; it goes before the first section");
  }
  if (surfaces.size() != 1) {
    fail(source, 0,
         "a surface file holds one [surface.<name>], not " + std::to_string(surfaces.size()));
  }
  joinSurfaceRecords(surfaces, records, source);
  return surfaces.front();
}

const SurfaceProfile *findSurface(const RigProfile &profile, std::string_view name) {
  const auto found = std::find_if(profile.surfaces.begin(), profile.surfaces.end(),
                                  [name](const SurfaceProfile &s) { return s.name == name; });
  return found == profile.surfaces.end() ? nullptr : &*found;
}

RigProfile onSurface(const RigProfile &profile, const std::string &name) {
  const SurfaceProfile *surface = findSurface(profile, name);
  if (surface == nullptr) {
    throw std::invalid_argument("Heron::Profile::onSurface() : " + profile.source +
                                " has no surface '" + name + "'");
  }
  RigProfile on = profile;
  on.activeSurface = name;
  if (on.geometry) {
    on.geometry->maxSpeed = surface->maxSpeed.value_or(on.geometry->maxSpeed);
    on.geometry->minSpeed = surface->minSpeed.value_or(on.geometry->minSpeed);
    on.geometry->maxTurnRate = surface->maxTurnRate.value_or(on.geometry->maxTurnRate);
    on.geometry->brake = surface->brake.value_or(on.geometry->brake);
  }
  if (on.odometry) {
    on.odometry->linearScalar = surface->linearScalar.value_or(on.odometry->linearScalar);
    on.odometry->angularScalar = surface->angularScalar.value_or(on.odometry->angularScalar);
  }
  return on;
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
