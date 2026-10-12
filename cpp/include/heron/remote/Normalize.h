// File        : Normalize.h
// Author      : Chris Pane
// Description : Turns raw iBUS channel values into control values: centred
//               axes, knobs and three-position switches.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Pure integer functions. Raw values are nominally 1000..2000 with 1500 at
// centre, but trims, endpoints and subtrim move all three, so each is a
// parameter rather than a constant. Integer output in thousandths keeps the
// arithmetic exact and the tests free of tolerances.
//
// This is design guidance for robot control, not protocol: see
// docs/remote-ibus-protocol.md §7.

#ifndef HERON_REMOTE_NORMALIZE_H
#define HERON_REMOTE_NORMALIZE_H

#include <cstdint>

namespace Heron::Remote {

/** Full scale of a normalized value: an axis spans -1000..+1000, a knob 0..1000. */
inline constexpr std::int32_t kNormFullScale = 1000;

/**
 * How to map one raw axis to -1000..+1000.
 */
struct AxisConfig {
  std::int32_t minRaw{1000};   // raw value that maps to -1000; anything lower clamps
  std::int32_t center{1500};   // raw value that maps to 0
  std::int32_t maxRaw{2000};   // raw value that maps to +1000; anything higher clamps
  std::int32_t deadband{0};    // within this distance of center the output is 0
};

/**
 * Maps a raw axis value to -1000..+1000.
 *
 * Inside the deadband the result is 0. Outside it the output is rescaled so
 * it rises from 0 at the deadband edge to full scale at the endpoint, with no
 * step, and each side is scaled on its own so an off-centre center still
 * reaches both ends. Beyond an endpoint the result clamps.
 *
 * @param raw The channel value as received
 * @param config Endpoints, center and deadband
 * @return The normalized value, -1000..+1000, rounded to nearest
 * @throws std::invalid_argument unless minRaw < center < maxRaw, deadband >= 0,
 *         and the deadband is narrower than both halves
 */
[[nodiscard]] std::int32_t normalizeAxis(std::uint16_t raw, const AxisConfig &config);

/**
 * Maps a raw knob or slider value to 0..1000, clamping outside the endpoints.
 *
 * @param raw The channel value as received
 * @param minRaw Raw value that maps to 0
 * @param maxRaw Raw value that maps to 1000
 * @return The normalized value, 0..1000, rounded to nearest
 * @throws std::invalid_argument unless minRaw < maxRaw
 */
[[nodiscard]] std::int32_t normalizeKnob(std::uint16_t raw, std::int32_t minRaw = 1000,
                                         std::int32_t maxRaw = 2000);

enum class Switch3 : std::uint8_t {
  Low,
  Mid,
  High
};

/**
 * Decodes a three-position switch. A two-position switch decodes as Low or High.
 *
 * @param raw The channel value as received
 * @param lowBelow Values below this are Low
 * @param highAbove Values above this are High; anything between is Mid
 * @return The switch position
 * @throws std::invalid_argument unless lowBelow <= highAbove
 */
[[nodiscard]] Switch3 decodeSwitch3(std::uint16_t raw, std::int32_t lowBelow = 1250,
                                    std::int32_t highAbove = 1750);

/** @return "LOW", "MID" or "HIGH" */
[[nodiscard]] const char *toString(Switch3 position) noexcept;

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_NORMALIZE_H
