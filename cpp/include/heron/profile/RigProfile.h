// File        : RigProfile.h
// Author      : Chris Pane
// Description : A rig's profile: the measured numbers of one physical Heron,
//               read and checked from profiles/<rig>.conf.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// Heron's code runs on several physical robots ("rigs"): the bench rig, the
// mini robot, later the 4-ft robot. Rigs of the same design ("platform")
// run the same build, but each has its own measured numbers -- which shield
// terminal each motor is wired to, where each motor starts to turn, how fast
// it may ramp. Those numbers live in one text file per rig, its profile,
// kept in the repository under profiles/ so every calibration has a history.
// This library reads a profile and checks every value in it.
//
//   profiles/bench.conf  --loadRigProfile()-->  RigProfile  -->  heron_motor,
//                                                                motor_link, ...
//
// The design, and why calibration is data rather than code, is
// docs/rig-profiles.md.
//
// The file
// --------
// Lines of "key = value" under "[section]" headers, and "#" comments:
//
//   # profiles/bench.conf
//   format = 1
//
//   [rig]
//   name = bench
//   platform = uno-l293d
//
//   [motor]
//   terminal = 1 2 4 3
//   sign = + + + +
//   ramp_per_ms = 4
//   min_duty_forward = 76 81 87 92
//   min_duty_backward = 71 76 76 102
//
//   [motor.calibration]
//   date = 2026-10-25
//   method = motor_link sweep, step 20, from rest; +10 margin
//
// The rules, each one an error if broken:
//
//   - A "#" starts a comment only at the start of a line, so a value may
//     contain one. Blank lines are ignored; spaces around keys and values
//     are trimmed.
//   - format must be 1, before any section.
//   - The sections are [rig], [motor] and [motor.calibration]; no others.
//     [rig] and [motor] are required, with every key below. No section or
//     key may appear twice.
//   - [motor.calibration] is a free-text record of how the numbers were
//     measured: any keys, any values, all optional.
//
// The keys, in protocol motor order (1 front-left, 2 front-right, 3
// back-right, 4 back-left; docs/motor-protocol.md §1):
//
//   [rig]     name               the rig: lower case, digits and "-", and the
//                                same as the file's name without ".conf"
//             platform           the hardware design: uno-l293d
//   [motor]   terminal           the driver terminal each motor is wired to:
//                                four of 1..4 (M1..M4), each used once
//             sign               + or -, four: - for a motor that turns
//                                backwards when driven forward
//             ramp_per_ms        0..2000 wheel units per ms; 0 is no ramp
//             min_duty_forward   PWM duty, four of 0..255: where each motor
//             min_duty_backward  starts turning from rest (the dead band)
//
// Strict on purpose: a misspelt key that was silently ignored would leave a
// motor running on a default. When a later workstream needs a section
// ([odometry], [lidar], [geometry]), it is added here, with its checks.
//
// Which rig this Pi is
// --------------------
// Each Pi holds its rig's name in ~/heron/rig, one line, written once by
// scripts/set_rig.sh. deploy_cpp.sh copies profiles/ to ~/heron/profiles/.
// findRigProfile() puts the two together:
//
//   const auto profile = Heron::Profile::loadRigProfile(Heron::Profile::findRigProfile());
//   //  reads ~/heron/rig ("bench"), loads ~/heron/profiles/bench.conf
//
// Errors
// ------
// Every problem throws std::runtime_error, whose message names the file and,
// where there is one, the line:
//
//   Heron::Profile::parseRigProfile() : profiles/bench.conf:14: unknown key
//   'ramp_per_sec' in [motor]
//
// A profile is read once, at start-up, so a program reports the message and
// stops. Nothing here is a hot path, and nothing is thread-unsafe: every
// function is pure apart from reading files and $HOME.

#ifndef HERON_PROFILE_RIG_PROFILE_H
#define HERON_PROFILE_RIG_PROFILE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Heron::Profile {

/** The file format this library reads: the value of "format". */
inline constexpr std::int32_t kProfileFormat = 1;

/** Motors in a profile, in protocol order. */
inline constexpr std::size_t kProfileMotorCount = 4;

/** Platforms a profile may name. One build of the firmware per platform. */
inline constexpr std::array<std::string_view, 1> kKnownPlatforms = {"uno-l293d"};

/** The highest ramp_per_ms: a full reversal, -1000 to 1000, in 1 ms. */
inline constexpr std::uint16_t kMaxRampPerMs = 2000;

/**
 * The [motor] section. Arrays are in protocol order: index 0 is motor 1,
 * front-left.
 */
struct MotorProfile {
  std::array<std::uint8_t, kProfileMotorCount> terminal{};   // 1..4 for M1..M4, each once
  std::array<std::int8_t, kProfileMotorCount> sign{};        // +1, or -1 for a reversed motor
  std::uint16_t rampPerMs{0};                                // 0..kMaxRampPerMs; 0 is no ramp
  std::array<std::uint8_t, kProfileMotorCount> minDutyForward{};
  std::array<std::uint8_t, kProfileMotorCount> minDutyBackward{};
};

/**
 * One rig's profile, every value checked.
 */
struct RigProfile {
  std::string source;    // where it was read from, for messages
  std::string name;      // [rig] name
  std::string platform;  // [rig] platform, one of kKnownPlatforms
  MotorProfile motor;
  /** [motor.calibration], key and value, in file order; empty if absent. */
  std::vector<std::pair<std::string, std::string>> motorCalibration;
};

/**
 * Parses and checks a profile's text.
 *
 * @param text The file's contents
 * @param source What to call it in messages, normally its path
 * @return The profile
 * @throws std::runtime_error naming source and line, for any broken rule
 *         (see the file header)
 */
[[nodiscard]] RigProfile parseRigProfile(std::string_view text, const std::string &source);

/**
 * Reads, parses and checks a profile file, and checks that its [rig] name is
 * the file's name: bench.conf must say name = bench.
 *
 * @param file The profile, e.g. profiles/bench.conf
 * @return The profile
 * @throws std::runtime_error if the file cannot be read or breaks a rule
 */
[[nodiscard]] RigProfile loadRigProfile(const std::filesystem::path &file);

/**
 * @return true if name is a valid rig name: 1 to 32 of a-z, 0-9 and "-",
 *         starting with a letter or digit
 */
[[nodiscard]] bool isValidRigName(std::string_view name);

/**
 * Reads the rig name a Pi holds, written by scripts/set_rig.sh.
 *
 * @param file Normally ~/heron/rig
 * @return The name, without its newline
 * @throws std::runtime_error if the file is missing or does not hold one
 *         valid name; the message says how to set it
 */
[[nodiscard]] std::string readRigName(const std::filesystem::path &file);

/**
 * @return ~/heron, where deployment puts Heron's files on a Pi
 * @throws std::runtime_error if $HOME is not set
 */
[[nodiscard]] std::filesystem::path heronHome();

/**
 * This Pi's profile: ~/heron/profiles/<rig>.conf, the rig named in
 * ~/heron/rig. Only finds the path; load it with loadRigProfile().
 *
 * @param home Heron's directory; heronHome() unless a test says otherwise
 * @throws std::runtime_error as readRigName()
 */
[[nodiscard]] std::filesystem::path findRigProfile(const std::filesystem::path &home = heronHome());

}  // namespace Heron::Profile

#endif  // HERON_PROFILE_RIG_PROFILE_H
