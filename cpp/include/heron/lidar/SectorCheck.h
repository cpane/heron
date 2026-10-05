// File        : SectorCheck.h
// Author      : Chris Pane
// Description : Is a sector around the robot clear, and how close is the
//               nearest return in it? The consumer rules, as one function.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Header-only and SDK-free: it works on any Scan, live or replayed.
//
// It applies the four consumer rules of docs/lidar-cpp-design.md §7.1, in
// order, and answers Unknown at the first one that fails:
//
//   1. fresh      the scan ended no longer ago than maxAge
//   2. covered    the whole revolution was observed (Scan::isCoverageOk)
//   3. observed   no unobserved arc overlaps the sector
//   4. nearest    only then are distances used
//
// A known answer with no point in the sector is real, with an infinite
// distance: the sensor looked there and nothing echoed back. What distance
// counts as too close is the caller's policy, not this function's.

#ifndef HERON_LIDAR_SECTOR_CHECK_H
#define HERON_LIDAR_SECTOR_CHECK_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include "heron/lidar/Scan.h"

namespace Heron::Lidar {

/**
 * A sector around the robot, in the robot frame.
 */
struct Sector {
  float centerRad{0.0f};               // 0 is straight ahead, positive to the left
  float halfWidthRad{kPi / 6.0f};      // 30 degrees each side: a 60 degree arc
};

/**
 * Why a check could not give a known answer.
 */
enum class SectorReason : std::uint8_t {
  None,               // the answer is known
  NoScan,             // nothing published yet
  Stale,              // rule 1
  NotCovered,         // rule 2
  SectorNotObserved   // rule 3
};

/**
 * The answer to one check.
 */
struct SectorResult {
  /** True when all three gates passed: nearestM is then the answer, and
   *  infinity means nothing is there. False means the sector is unknown. */
  bool known{false};
  SectorReason reason{SectorReason::NoScan};
  /** Nearest valid return inside the sector; infinity when there is none. */
  float nearestM{std::numeric_limits<float>::infinity()};
  float nearestBearingRad{0.0f};
  /** Which revolution this came from, so a caller can tell a repeat. */
  std::uint64_t seq{0};
  /** How old the scan was at the check, tEnd to now. */
  Duration age{};
};

/** The proposed freshness limit: about two revolutions at ~7.3 Hz. */
inline constexpr Duration kDefaultMaxScanAge = std::chrono::milliseconds(275);

/**
 * @param bearingRad A robot-frame bearing
 * @param sector The sector to test against
 * @return true when the bearing lies inside the sector, edges included
 */
[[nodiscard]] inline bool isInSector(float bearingRad, const Sector &sector) noexcept {
  return std::fabs(wrapBearing(bearingRad - sector.centerRad)) <= sector.halfWidthRad;
}

/**
 * Was every direction in the sector looked at in this revolution?
 *
 * Exact: each unobserved arc is tested against the sector as an interval on
 * the circle, so an arc that only clips the sector's edge is still caught.
 *
 * @param scan The revolution
 * @param sector The sector
 * @return false if any unobserved arc overlaps the sector, or nothing was sampled
 */
[[nodiscard]] inline bool isSectorObserved(const Scan &scan, const Sector &sector) noexcept {
  if (scan.rawSampleCount == 0) {
    return false;
  }
  const float sectorStart = sector.centerRad - sector.halfWidthRad;
  const float sectorWidth = 2.0f * sector.halfWidthRad;
  for (const Arc &arc : scan.unobserved) {
    // Where the arc starts, measured anticlockwise from the sector's start.
    float offset = std::fmod(arc.startRad - sectorStart, kTwoPi);
    if (offset < 0.0f) {
      offset += kTwoPi;
    }
    // Overlap if the arc starts inside the sector, or runs on past 2pi back
    // round into it.
    if (offset < sectorWidth || offset + arc.widthRad > kTwoPi) {
      return false;
    }
  }
  return true;
}

/**
 * Checks a sector against a scan.
 *
 * @param scan The newest scan, or nullptr if there is none yet
 * @param sector Where to look
 * @param now The current time, steady_clock
 * @param maxAge Scans older than this answer Unknown (Stale)
 * @return A known answer with the nearest return, or unknown with the reason
 */
[[nodiscard]] inline SectorResult checkSector(const Scan *scan, const Sector &sector,
                                              TimePoint now,
                                              Duration maxAge = kDefaultMaxScanAge) noexcept {
  SectorResult result;
  if (scan == nullptr) {
    return result;  // NoScan
  }
  result.seq = scan->seq;
  result.age = now - scan->tEnd;
  if (result.age > maxAge) {
    result.reason = SectorReason::Stale;
    return result;
  }
  if (!scan->isCoverageOk()) {
    result.reason = SectorReason::NotCovered;
    return result;
  }
  if (!isSectorObserved(*scan, sector)) {
    result.reason = SectorReason::SectorNotObserved;
    return result;
  }
  for (const Point &point : scan->points) {
    if (isInSector(point.bearingRad, sector) && point.rangeM < result.nearestM) {
      result.nearestM = point.rangeM;
      result.nearestBearingRad = point.bearingRad;
    }
  }
  result.known = true;
  result.reason = SectorReason::None;
  return result;
}

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_SECTOR_CHECK_H
