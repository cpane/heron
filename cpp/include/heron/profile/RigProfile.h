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
//   [geometry]
//   length = 0.24
//   width = 0.20
//   ...
//
// The rules, each one an error if broken:
//
//   - A "#" starts a comment only at the start of a line, so a value may
//     contain one. Blank lines are ignored; spaces around keys and values
//     are trimmed.
//   - format must be 1, before any section.
//   - The sections are [rig], [motor], [geometry], [lidar], [odometry] and
//     [navigation], each with an optional [<section>.calibration]; no
//     others. [rig] and [motor] are required; the other four are optional,
//     but one that is present has every key below. No section or key may
//     appear twice.
//   - A [<section>.calibration] is a free-text record of how that section's
//     numbers were measured: any keys, any values, all optional. It needs
//     its section: [lidar.calibration] without [lidar] is an error.
//   - A number with a fraction is digits with at most one ".", and an
//     optional leading sign: 0.24, -0.05, 1. Not 1e3, not .5.
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
// The robot's own sections, read by heron_robot for navigation
// (docs/navigation-design.md §10). Lengths in metres, in the robot frame:
// x forward, y left, from the wheelbase centre; angles in degrees,
// positive turning left. How each is measured: docs/rig-calibration.md §8.
//
//   [geometry]   length         0.05..3     the outline, wheels included
//                width          0.05..3
//                max_speed      0.01..5     m/s at full command
//                max_turn_rate  0.01..20    rad/s at full command
//                accel          0.01..10    m/s^2, the planner's window
//                brake          0.01..20    m/s^2 from full speed
//   [lidar]      x, y           -2..2       the spin axis
//                yaw_deg        -180..180   the housing's front
//                height         0..2        above the floor
//   [odometry]   x, y           -2..2       the OTOS's centre
//                yaw_deg        -180..180
//                linear_scalar  0.872..1.127, true / reported, as the OTOS
//                angular_scalar takes them (Odometry::OtosConfig)
//   [navigation] stop_margin    0..1        m round the outline
//                slow_margin    0..2        at least stop_margin
//
// [geometry] is shared: the mixing's wheel keys (wheel radius, track,
// wheelbase) join it when the mixing needs them, so max_speed is held once
// (decided 2026-11-11).
//
// Surfaces (navigation design §19, P18): the numbers that depend on the
// floor, one named set per floor, any number of them:
//
//   [rig]        drive          optional: the drive's name (motors, wheels,
//                               controller), as the library's folder
//                surface        optional: the default surface's name
//   [surface.<name>]            <name> as a rig name; every key optional
//                max_speed, min_speed, max_turn_rate, brake  override
//                               [geometry]'s, same ranges (min_speed 0..5)
//                linear_scalar, angular_scalar  override [odometry]'s
//                mirror         0 or 1: whether left may be taken as right
//                               mirrored where only one side was measured
//                p<n>           the motion table, one point a key:
//                               <direction deg -180..180> <size 0..1.5>
//                               <vx -5..5> <vy -5..5> <wz -20..20>
//   [surface.<name>.calibration] free text, as the others; needs its surface
//
// Known surfaces also live in a library, one file each:
// profiles/surfaces/<drive>/<name>.conf, holding "format = 1", one
// [surface.<name>] and its record (parseSurfaceFile()). loadRigProfile()
// adds every surface of the rig's drive from the library, after the rig's
// own (which win on a name), and checks [rig] surface names one of them.
// onSurface() gives the profile as seen on one floor (I22).
//
// Strict on purpose: a misspelt key that was silently ignored would leave a
// motor running on a default. A new section is added here, with its
// checks.
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
// Which floor this Pi is on
// -------------------------
// The profile's [rig] surface is the floor its rig usually runs on. A Pi
// can be told another, until told otherwise, by scripts/set_surface.sh,
// which writes its name to ~/heron/surface: the robot taken from the
// hardwood to the Maker Faire's rubber floor and back, with no change to
// the profile and no deploy, and the boot service, which starts
// heron_robot with no arguments, follows it. readChosenSurface() reads it:
//
//   const auto chosen = Heron::Profile::readChosenSurface(Heron::Profile::heronHome() / "surface");
//   const RigProfile on = onSurface(profile, chosen.value_or(profile.surface));
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
#include <optional>
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

/** The [geometry] section: the robot's outline and how fast it moves. */
struct GeometryProfile {
  double length{0.0};       // m, the outline along x, wheels included
  double width{0.0};        // m, along y
  double maxSpeed{0.0};     // m/s at full command
  /** m/s at the smallest command that moves it: the motors' dead band
   *  lifts every command above zero to at least this. Optional; 0 (the
   *  default) is speed in proportion to the command. */
  double minSpeed{0.0};
  double maxTurnRate{0.0};  // rad/s at full command
  double accel{0.0};        // m/s^2
  double brake{0.0};        // m/s^2, stopping from full speed
};

/** The [lidar] section: where the LiDAR is mounted. */
struct LidarProfile {
  double x{0.0};       // m, the spin axis, robot frame
  double y{0.0};       // m
  double yawDeg{0.0};  // degrees, the housing's front from the robot's x
  double height{0.0};  // m above the floor
};

/** The [odometry] section: where the OTOS is mounted, and its scalars. */
struct OdometryProfile {
  double x{0.0};              // m, the sensor's centre, robot frame
  double y{0.0};              // m
  double yawDeg{0.0};         // degrees
  double linearScalar{1.0};   // true / reported distance
  double angularScalar{1.0};  // true / reported angle
};

/** The [navigation] section: the margins kept round the outline. */
struct NavigationProfile {
  double stopMargin{0.0};  // m: nothing comes nearer
  double slowMargin{0.0};  // m: slower within this, at least stopMargin
};

/** A [<section>.calibration]: key and value, in file order. */
using CalibrationRecord = std::vector<std::pair<std::string, std::string>>;

/** One point of a surface's motion table: a command and what it made.
 *  Plain numbers here; heron_motion's MotionTable reads them (I18). */
struct SurfacePoint {
  double directionDeg{0.0};  // the command's, counterclockwise from forward
  double size{0.0};          // its length, fractions of full drive
  double vx{0.0};            // m/s, robot frame: what the robot made
  double vy{0.0};            // m/s
  double wz{0.0};            // rad/s
};

/** A [surface.<name>]: one floor's numbers (design §19, P18). A set value
 *  overrides [geometry]'s or [odometry]'s on that floor; points empty means
 *  no motion table. */
struct SurfaceProfile {
  std::string name;
  std::string source;                // the file it came from
  std::vector<SurfacePoint> points;  // p01, p02, ... in file order
  bool mirror{false};                // mirror = 1; off unless said
  std::optional<double> maxSpeed;
  std::optional<double> minSpeed;
  std::optional<double> maxTurnRate;
  std::optional<double> brake;
  std::optional<double> linearScalar;
  std::optional<double> angularScalar;
  CalibrationRecord calibration;     // [surface.<name>.calibration]
};

/**
 * One rig's profile, every value checked. The optional sections are empty
 * when the file has none; a program that needs one says so (heron_robot
 * refuses Assisted and Navigate without them).
 */
struct RigProfile {
  std::string source;    // where it was read from, for messages
  std::string name;      // [rig] name
  std::string platform;  // [rig] platform, one of kKnownPlatforms
  std::string drive;     // [rig] drive, or empty: its surfaces' library folder
  std::string surface;   // [rig] surface, or empty: the default surface
  /** The rig's own [surface.*], then its drive's library's (loadRigProfile()). */
  std::vector<SurfaceProfile> surfaces;
  /** The surface onSurface() applied, or empty. */
  std::string activeSurface;
  MotorProfile motor;
  std::optional<GeometryProfile> geometry;
  std::optional<LidarProfile> lidar;
  std::optional<OdometryProfile> odometry;
  std::optional<NavigationProfile> navigation;
  /** Each section's [<section>.calibration]; empty if absent. */
  CalibrationRecord motorCalibration;
  CalibrationRecord geometryCalibration;
  CalibrationRecord lidarCalibration;
  CalibrationRecord odometryCalibration;
  CalibrationRecord navigationCalibration;
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
 * Parses a library surface file: "format = 1", one [surface.<name>] and
 * optionally its [surface.<name>.calibration]; nothing else.
 *
 * @throws std::runtime_error naming source and line, for any broken rule
 */
[[nodiscard]] SurfaceProfile parseSurfaceFile(std::string_view text, const std::string &source);

/** @return The surface of that name, or nullptr. */
[[nodiscard]] const SurfaceProfile *findSurface(const RigProfile &profile, std::string_view name);

/**
 * The profile as seen on one floor (I22): [geometry] and [odometry] with
 * the surface's values in place, activeSurface set. Sections the profile
 * lacks stay absent.
 *
 * @throws std::invalid_argument if the profile has no surface of that name
 */
[[nodiscard]] RigProfile onSurface(const RigProfile &profile, const std::string &name);

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
 * Reads the floor a Pi was told it is on, written by scripts/set_surface.sh.
 *
 * @param file Normally ~/heron/surface
 * @return The surface's name; no value when the file is missing, so the
 *         profile's [rig] surface stands
 * @throws std::runtime_error if the file is there but does not hold one
 *         valid name (isValidRigName()'s rule); the message says how to
 *         set it
 */
[[nodiscard]] std::optional<std::string> readChosenSurface(const std::filesystem::path &file);

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
