// File        : Motors.h
// Author      : Chris Pane
// Description : The motors as the robot sees them: start them, drive them
//               every 20 ms, and read the newest state from any thread.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// Heron's four wheels are driven by an Arduino on /dev/arduino. The Pi does
// the motion maths and sends four wheel values every 20 ms; the Arduino
// turns them into PWM. Motors is the Pi's side of that link, the one class
// robot code uses to drive. It hides the wire format (Protocol.h), the
// start-up handshake, and keeping the Arduino configured with the rig's
// calibration.
//
// It is shaped like Lidar, RemoteControl and Odometry -- start(), stop(),
// getLatest(), getHealth(), a reader thread publishing the newest state --
// with one addition: drive(), through which the robot's control loop sends.
// The design and its reasons are docs/motor-cpp-design.md.
//
// The ideas behind it
// -------------------
//   The robot's loop is the heartbeat. Each drive() call sends one Drive
//   frame, at once. Call it every 20 ms, even when nothing changes. If the
//   robot's loop stalls, the frames stop, and the Arduino's own watchdog
//   releases the motors 250 ms later. There is deliberately no thread here
//   that repeats the last command: it would keep the watchdog fed while the
//   robot hangs.
//
//   The Arduino holds no calibration of its own. After every reset it
//   refuses Drive until it is sent the rig's [motor] numbers (which terminal
//   each motor is on, which way round, its dead band, the ramp) in a
//   Configure message, and it reports a checksum of what it holds in every
//   Status. Opening the port resets it. So start() opens the port, waits
//   for the Arduino's Boot, sends Configure, and returns only when Status
//   shows the checksum: about 1.6 s. While running, the reader thread sends
//   Configure again whenever the checksum is not ours -- after a reset (a
//   brown-out under load) or a lost frame -- and counts the resets.
//
// How the pieces fit
// ------------------
//   RigProfile      the rig's profile, loaded by the caller (heron_profile)
//     -> toMotorsConfig()   the device, the platform and the [motor] section
//   Motors          start() / stop()            the owning thread
//                   drive() / stopMotors()      the control loop, every 20 ms
//                   getLatest() / getHealth()   any thread
//
// Usage
// -----
//   const auto profile = Heron::Profile::loadRigProfile(Heron::Profile::findRigProfile());
//   Heron::Motor::Motors motors(Heron::Motor::toMotorsConfig(profile));
//   if (!motors.start()) {                    // blocks ~1.6 s
//     report(motors.getHealth().lastError);   // says what to do: flash, reflash, ...
//   }
//
//   // every 20 ms, from the control loop
//   motors.drive({0.5, 0.5, 0.5, 0.5});       // front-left, front-right, back-right,
//                                             // back-left; -1.0..1.0, + is forward
//   if (!motors.getLatest().isOk()) {
//     // the board is gone, or reset and not configured again yet
//   }
//
//   motors.stopMotors();                      // release all four now
//   motors.stop();                            // sends Stop, closes the port
//
// Constraints and pitfalls
// ------------------------
// - start() and stop() belong to one owning thread. drive() and
//   stopMotors() are for the control loop; they are safe from any thread,
//   since writes go under a lock, but one sender keeps the frames in order.
//   getLatest() and getHealth() may be called from anywhere.
// - drive() returning true means the frame was sent, not that the motors
//   obeyed: after a reset the Arduino refuses Drive until it is configured
//   again. getLatest().isOk() says whether it will obey.
// - Nothing else may have /dev/arduino open: motor_link and flash_firmware.sh
//   both use it.
// - Every start() resets the Arduino, which releases the motors.

#ifndef HERON_MOTOR_MOTORS_H
#define HERON_MOTOR_MOTORS_H

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"
#include "heron/profile/RigProfile.h"

namespace Heron::Motor {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

/** Wheels in a drive() call: front-left, front-right, back-right, back-left. */
using WheelValues = std::array<double, kMotorCount>;

/**
 * Settings fixed for the life of a Motors instance. Build it from the rig's
 * profile with toMotorsConfig(), then change the device or the timings if
 * needed.
 */
struct MotorsConfig {
  std::string device{"/dev/arduino"};
  /** The profile's [rig] platform: the Arduino's Boot must report the same. */
  std::string platform;
  /** The profile's [motor] section: what Configure sends. */
  Heron::Profile::MotorProfile motor;
  /** How long start() waits for the Boot that opening the port causes. */
  Duration startTimeout{std::chrono::seconds(3)};
  /** No Status for longer than this, and the link reads as stale: three of
   *  the Arduino's 100 ms periods. */
  Duration staleAfter{std::chrono::milliseconds(300)};
};

/**
 * Lifecycle state.
 */
enum class MotorsState : std::uint8_t {
  Stopped,  // constructed, or stopped cleanly
  Running,  // started and configured; drive() sends
  Failed    // start() failed, or the reader stopped on an error; see lastError
};

/**
 * The newest state of the motors, as of the moment it was asked for.
 */
struct MotorsSnapshot {
  /** A Status arrived within staleAfter of the call. Judged at the call, so
   *  a dead reader or a vanished board reads as not fresh. */
  bool fresh{false};
  /** The newest Status carries the checksum of this rig's calibration: the
   *  Arduino will act on Drive. */
  bool configured{false};
  /** Status messages so far; 0 before the first. */
  std::uint64_t seq{0};
  /** Time from the newest Status to the call; zero before the first. */
  Duration age{};
  /** The newest Status as the Arduino sent it: state, stop reason, target
   *  and applied wheel values (-1000..1000), the board's link counters.
   *  Defaults before the first. */
  Status status{};

  /** @return true when the board is alive and holds this rig's calibration:
   *  the one test for "the motors will do what I send". Stopped motors are
   *  still OK; why they stopped is status.stopReason. */
  [[nodiscard]] bool isOk() const noexcept { return fresh && configured; }
};

/**
 * Counters and state, for diagnostics and the robot's health reporting.
 */
struct MotorsHealth {
  MotorsState state{MotorsState::Stopped};
  /** The newest Boot: firmware version, protocol, platform. */
  BootInfo boot{};
  /** Boots seen since start(), the one opening the port causes included. */
  std::uint64_t boots{0};
  /** Boots after start() succeeded: the Arduino reset while running. Under
   *  load, most likely a brown-out from motor current. */
  std::uint64_t resets{0};
  std::uint64_t configuresSent{0};
  /** Times the link went from fresh to stale while running. */
  std::uint64_t staleEvents{0};
  /** The Pi side's frame counters. */
  ParserStats link{};
  std::string lastError;
};

/**
 * The Arduino motor controller, driven by the robot's control loop and read
 * by a thread of its own. See the file header.
 *
 * Thread Safety: start() and stop() are for one owning thread. drive(),
 * stopMotors(), getLatest() and getHealth() may be called from any thread
 * and never block for long.
 */
class Motors {
  public:
    /** @param config Checked by start(), which reports a bad one through lastError */
    explicit Motors(const MotorsConfig &config);

    /** Sends Stop if running, stops the reader and closes the port. */
    ~Motors();

    Motors(const Motors &) = delete;
    Motors &operator=(const Motors &) = delete;

    /**
     * Opens the port (which resets the Arduino), waits for its Boot, checks
     * its protocol and platform, sends the rig's calibration and waits for
     * Status to confirm it. Blocks for about 1.6 s on the bench rig, at most
     * startTimeout plus one second. Restarting a stopped or failed instance
     * is allowed; the counters start again from zero.
     *
     * @return true when running; false with getHealth().lastError set
     */
    bool start();

    /**
     * Sends Stop, stops the reader thread and closes the port. Safe to call
     * more than once, and on an instance that never started.
     */
    void stop();

    /**
     * Sends one Drive frame now. Call every 20 ms (see the file header).
     *
     * @param wheels -1.0..1.0 of full drive per wheel, + driving the robot
     *               forward; scaled to -1000..1000, rounded and clamped. If
     *               any is NaN or infinite, Stop is sent instead.
     * @return true if a frame was sent; false, sending nothing, unless running
     */
    bool drive(const WheelValues &wheels);

    /**
     * Sends the Stop message: all four motors released now, until the next
     * drive(). Not stop(), which is the lifecycle.
     *
     * @return true if it was sent; false unless running
     */
    bool stopMotors();

    /** @return The newest state, judged now */
    [[nodiscard]] MotorsSnapshot getLatest() const;

    /**
     * @param now The time to judge freshness at, steady_clock; for a loop
     *            that stamps each cycle once
     * @return The newest state, judged at now
     */
    [[nodiscard]] MotorsSnapshot getLatest(TimePoint now) const;

    /** @return A snapshot of the counters and state */
    [[nodiscard]] MotorsHealth getHealth() const;

  private:
    struct Impl;  // the port, the thread and the lock live here
    std::unique_ptr<Impl> m_impl;
};

/**
 * The configuration for a rig: the default device, and the platform and
 * [motor] section of its profile.
 */
[[nodiscard]] MotorsConfig toMotorsConfig(const Heron::Profile::RigProfile &profile);

/**
 * A profile's [motor] section as the Configure message carries it. Shared
 * with motor_link, so the two cannot convert differently.
 */
[[nodiscard]] MotorConfig toMotorConfig(const Heron::Profile::MotorProfile &motor);

/** @return The platform a profile names, as Boot reports it; empty if unknown */
[[nodiscard]] std::optional<Platform> toPlatform(std::string_view name);

/** @return The profile's name for a platform: "uno-l293d", or "unknown" */
[[nodiscard]] const char *toString(Platform platform) noexcept;

/** @return "STOPPED", "RUNNING" or "FAILED" */
[[nodiscard]] const char *toString(MotorsState state) noexcept;

}  // namespace Heron::Motor

#endif  // HERON_MOTOR_MOTORS_H
