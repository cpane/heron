// File        : Scan.h
// Author      : Chris Pane
// Description : The LiDAR boundary type: one revolution, in the robot frame.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// This is the contract between the LiDAR adapter and the rest of the robot.
// Everything below it may know about the vendor SDK; everything above it sees
// only what is here. Rationale in docs/lidar-cpp-design.md; the measurements
// every decision rests on are in docs/rplidar-a1m8-findings.md.
//
// It is the one decision that gets more expensive with time: the adapter,
// SectorCheck.h, the viewer server, the test programs and, later, robot code
// are all written against it, and a change to a field touches every one.
// That is why it was settled first, in two reviews against the recordings
// and the SDK source (2026-10-01).
//
// ---------------------------------------------------------------------------
// DECISIONS -- each records the evidence it was settled on
// ---------------------------------------------------------------------------
//
//  1. Bearing is in [-pi, +pi), not [0, 2pi). Chosen because it matches REP
//     103 yaw and because the first planned behaviour is "nearest obstacle in
//     a forward 60 degree arc", which then reduces to std::abs(bearing) <
//     0.52. Under [0, 2pi) that test straddles the wrap and becomes two
//     comparisons anyone can get wrong once. The half-open end is real:
//     std::remainder, the obvious library call, returns exactly +pi for +pi.
//     So wrapBearing() is the one implementation, and the adapter uses it.
//
//  2. No quality field. The strawman had one, documented as a "1-15
//     reflectivity proxy". Checked 2026-10-01 against the captures and the
//     SDK source, it means something different in every mode:
//
//       legacy   6 bits on the wire; 90-97% of valid returns read exactly
//                15. The SDK hands it back multiplied by 4.
//       express  NO quality field in the capsule. The SDK substitutes the
//                constant 188 for every valid return (handler_capsules.cpp).
//       ultra    real, but its resolution changes with distance scale.
//
//     A consumer filtering on it would be filtering on a constant in one
//     mode and a saturated value in another. Reinstate it when a consumer
//     needs it AND someone has measured what it means in the mode in use.
//
//  3. float, not double. Range is <= 12 m at 1/4 mm native resolution, so
//     float's ~7 significant digits are ample, and Point is 8 bytes -- a
//     530-point express revolution fits in ~4 KB and stays cache-friendly.
//
//  4. No pose, deliberately. Deskewing needs to know where the robot was
//     when each point was measured, but that is the robot's knowledge, not
//     the sensor's. The sensor reports when it saw things -- timeOf() --
//     and the core decides where it was.
//
//  5. isCoverageOk() takes a threshold rather than baking one in, because
//     how wide a hole is tolerable is a policy decision belonging to the
//     consumer, not to the sensor.
//
//  6. Coverage is measured over EVERY sample, no-returns included -- never
//     over the valid points. A no-return still proves the beam pointed
//     there; only a sample that never arrived leaves a hole. The first draft
//     of this file got that wrong, and measuring it against the captures
//     showed how badly: gaps between valid points are 30-115 degrees on
//     perfectly healthy revolutions (61-84% of samples return nothing), so
//     coverage failed 981 out of 981 real revolutions. Over all samples the
//     same revolutions read 8.0-8.9 degrees, every one.
//
//  7. The unobserved arcs are listed, not just counted. "Is the path ahead
//     clear?" needs "did I look ahead?", and a single worst-gap number
//     cannot answer that for a particular bearing. Measured correctly,
//     unobserved arcs are rare (overrun only), so the list is nearly always
//     empty and costs nothing.
//
//  8. The order points were measured in survives sorting. Sorting by
//     bearing throws away time order, and the sweep neither starts at 0 nor
//     runs toward increasing bearing: the sync sample lands anywhere from
//     351 to 359.5 degrees in the sensor frame, and the sensor turns
//     clockwise, so time runs toward DECREASING robot-frame bearing.
//     sweepStartRad records where it began; timeOf() does the arithmetic.
//
//  9. No scan mode. Quality was the main thing a consumer could misread
//     without knowing the mode, and it is gone (2). Expected point density
//     is a diagnostic and belongs on a health/status channel, which keeps
//     this boundary sensor-neutral.
//
//  NOT A CLAIM: a no-return arc is "looked, saw nothing within range", which
//  is not the same as "clear" -- glass and dark matte surfaces also return
//  nothing. That is a question about surfaces, not coverage, and this type
//  does not try to answer it.
//
// Naming: Point, Arc and Scan are plain aggregates whose data members are the
// public interface, so those members are camelCase without the m_ prefix,
// following the standard's own aggregate example (struct Config { mode }).
// The m_ prefix is kept for the private members of classes.

#ifndef HERON_LIDAR_SCAN_H
#define HERON_LIDAR_SCAN_H

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Heron {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration = Clock::duration;

inline constexpr float kPi = 3.14159265f;
inline constexpr float kTwoPi = 6.28318531f;

/**
 * Folds any angle into [-pi, +pi).
 *
 * The one implementation, so the half-open range every Point promises is
 * kept in one place. Measured 2026-10-01 over ~1.9e9 inputs per variant --
 * every float within 1e-4 of each multiple of pi out to +/-6pi, plus a sweep
 * of +/-40 rad: std::remainder broke the range (it returns +pi for +pi; its
 * range is closed at both ends), while this fmod form, atan2(sin, cos) and
 * while-loops did not. The final guard has never been seen to fire. It is
 * there because float rounding near the seam is easy to get wrong and one
 * comparison is cheap, not because an input is known to need it.
 *
 * @param rad Any angle, in radians
 * @return The same direction, in [-pi, +pi)
 */
[[nodiscard]] inline float wrapBearing(float rad) noexcept {
  float r = std::fmod(rad + kPi, kTwoPi);
  if (r < 0.0f) {
    r += kTwoPi;
  }
  r -= kPi;
  return r >= kPi ? -kPi : r;
}

/**
 * One valid return, in the ROBOT frame.
 */
struct Point {
  /**
   * Bearing in radians, range [-pi, +pi). 0 is straight ahead, positive is
   * to the LEFT (REP 103: x forward, y left, z up, yaw anticlockwise).
   *
   * The sensor's own frame is left-handed -- 0 out the front of the housing,
   * increasing CLOCKWISE, measured 2026-09-27 -- so the adapter negates on
   * the way in. That negation happens exactly once, here at the boundary.
   * Get it wrong and every obstacle mirrors across the forward axis with no
   * compile error and no exception; the robot simply steers toward things
   * instead of away.
   */
  float bearingRad{0.0f};

  /**
   * Distance in metres. Always > 0: invalid returns never reach a Scan.
   */
  float rangeM{0.0f};
};

/**
 * The sampling gap past which the adapter reports an arc as unobserved, and
 * the default for Scan::isCoverageOk(). 10 degrees.
 *
 * Healthy revolutions top out at 8.9 degrees -- every revolution in all 11
 * captures, legacy and express, 2026-10-01 -- and a starved reader produced
 * 174 degrees, so there is a wide margin either side. Note express does NOT
 * halve the worst gap, although it halves the median spacing: both modes read
 * ~8-9 degrees, for reasons not yet understood. 10 is also the threshold the
 * original overrun measurement used.
 */
inline constexpr float kUnobservedGapRad = 0.17453293f;

/**
 * An angular interval in the ROBOT frame, as an unobserved region.
 */
struct Arc {
  /**
   * Where the arc begins, in [-pi, +pi). It extends anticlockwise (towards
   * increasing bearing) from here.
   */
  float startRad{0.0f};

  /**
   * Always > 0. startRad + widthRad may exceed +pi: an arc behind the robot
   * straddles the seam, and contains() handles that so callers do not each
   * get it wrong in their own way.
   */
  float widthRad{0.0f};

  /**
   * Whether a bearing lies strictly inside the arc.
   *
   * Strictly: the samples bounding a gap were observed; only what lies
   * between them was not.
   *
   * @param bearingRad A robot-frame bearing in [-pi, +pi)
   * @return true if the bearing is inside, ends excluded
   */
  [[nodiscard]] bool contains(float bearingRad) const noexcept {
    float offset = bearingRad - startRad;
    if (offset < 0.0f) {
      offset += kTwoPi;
    }
    return offset > 0.0f && offset < widthRad;
  }
};

/**
 * One revolution of the LiDAR, ready for a consumer to use.
 *
 * Invariants the adapter guarantees, so that no consumer has to re-check
 * them:
 *
 *   - `points` is sorted by ascending `bearingRad`. The wire order is NOT
 *     angular order -- about 12% of samples arrive behind the previous angle
 *     -- so this is real work, not a formality.
 *   - every `bearingRad`, `sweepStartRad` and `Arc::startRad` is in
 *     [-pi, +pi) -- never exactly +pi -- because each went through
 *     wrapBearing().
 *   - every `rangeM` is > 0.
 *   - `tStart <= tEnd`.
 *
 * Thread Safety: a plain value. Safe to read from several threads once
 * published; not safe to modify concurrently.
 */
struct Scan {
  /**
   * Monotonic, one per revolution produced. A gap means the consumer fell
   * behind and skipped some: timestamps tell you *when*, this tells you
   * whether you missed anything.
   */
  std::uint64_t seq{0};

  /**
   * A revolution takes ~137 ms at ~7.3 Hz, so it is a span, not an instant.
   * Once the robot moves, points within one Scan come from different poses;
   * treating a revolution as a moment smears the map. Whether to deskew is a
   * later decision -- but a type that cannot express the span makes that
   * decision permanently, and by accident.
   *
   * Both are estimated MEASUREMENT times, not arrival times:
   *
   *   tStart  when the beam pointed at sweepStartRad. This is the SDK's scan
   *           timestamp: the sync sample's arrival, back-dated by the mode's
   *           per-sample delay (sl_lidar_driver.cpp, pushScanNodeData()).
   *   tEnd    when the beam came back round to sweepStartRad -- a full turn,
   *           not the last sample. The SDK gives no end time, so the adapter
   *           estimates it.
   *
   * The SDK stamps with CLOCK_MONOTONIC. Measured 2026-10-01 to be
   * steady_clock's clock on the Pi build: same epoch, and every scan stamp
   * lands 0.3-17 ms before the bytes that carried it.
   *
   * Note the Python server's t_start_ns/t_end_ns are host ARRIVAL times, so a
   * replay diff against it will be offset by the sensor's latency.
   */
  TimePoint tStart{};
  TimePoint tEnd{};

  /**
   * Robot-frame bearing of the first sample in time -- where the sweep began.
   * The adapter reads it off the sync sample before ascendScanData() sorts
   * the buffer, after which it cannot be recovered.
   */
  float sweepStartRad{0.0f};

  /**
   * Valid returns only, ascending by bearing.
   */
  std::vector<Point> points;

  /**
   * How many samples the sensor actually delivered for this revolution,
   * including invalid ones.
   *
   * This matters more than it looks. Invalid returns are 61-84% of samples
   * depending on the scene, and dropping them silently would leave a consumer
   * unable to distinguish "nothing is at 90 degrees" from "I did not look at
   * 90 degrees". An unobserved sector is not a clear sector.
   */
  std::uint32_t rawSampleCount{0};

  /**
   * Estimated samples lost to buffer overrun, from the angular-gap detector.
   *
   * Overrun on this device is completely silent: measured, ~128 consecutive
   * samples vanished with no error, no status flag, and byte alignment intact
   * afterwards. A suspiciously wide angular gap is the only evidence it
   * happened, and nothing in the vendor SDK provides it.
   */
  std::uint32_t droppedEstimate{0};

  /**
   * Widest angular gap between consecutive SAMPLES -- all of them, no-returns
   * included, closing the circle across the seam. The evidence behind
   * droppedEstimate, exposed so a consumer can apply its own judgement rather
   * than trusting ours. Healthy is ~8-9 degrees.
   *
   * Two ways to get this wrong, both of which look plausible:
   *
   *   - Measuring it on the valid points. Healthy revolutions then read
   *     30-115 degrees, and real loss is indistinguishable from a room with
   *     nothing in it.
   *   - Measuring it after the SDK's ascendScanData(). That function
   *     overwrites the angle of every no-return sample, in place, with an
   *     evenly spaced guess (sl_lidar_driver.cpp, ascendScanData_()), and
   *     those guesses land inside exactly the hole being looked for.
   *
   * So the adapter measures it on the buffer grabScanDataHq() returns, before
   * ascendScanData() touches it.
   */
  float worstSamplingGapRad{0.0f};

  /**
   * Every sampling gap wider than kUnobservedGapRad, robot frame. Nearly
   * always empty; when it is not, those bearings were not looked at, and an
   * obstacle there would be invisible. Use isObserved() rather than walking
   * this by hand.
   */
  std::vector<Arc> unobserved;

  /**
   * @return true if the revolution produced no valid returns
   */
  [[nodiscard]] bool isEmpty() const noexcept { return points.empty(); }

  /**
   * @return The number of valid returns
   */
  [[nodiscard]] std::size_t size() const noexcept { return points.size(); }

  /**
   * @return Wall time spanned by the revolution, tEnd - tStart
   */
  [[nodiscard]] Duration duration() const noexcept { return tEnd - tStart; }

  /**
   * Revolutions per second implied by this scan's span.
   *
   * @return The rate in Hz, or 0 if the span is unknown
   */
  [[nodiscard]] float rateHz() const noexcept {
    const float span =
        std::chrono::duration_cast<std::chrono::duration<float>>(duration()).count();
    return span > 0.0f ? 1.0f / span : 0.0f;
  }

  /**
   * Estimated time the beam pointed at a bearing.
   *
   * The sweep runs from sweepStartRad toward DECREASING bearing (the sensor
   * turns clockwise), one full turn between tStart and tEnd. Assumes constant
   * angular speed within the turn: revolution periods are measured steady to
   * 1-2 ms, but speed within one turn is not measured.
   *
   * @param bearingRad A robot-frame bearing in [-pi, +pi)
   * @return The estimated measurement time, in [tStart, tEnd]
   */
  [[nodiscard]] TimePoint timeOf(float bearingRad) const noexcept {
    float swept = sweepStartRad - bearingRad;
    if (swept < 0.0f) {
      swept += kTwoPi;
    }
    const std::chrono::duration<float, Duration::period> span(duration());
    return tStart + std::chrono::duration_cast<Duration>(span * (swept / kTwoPi));
  }

  /**
   * @return The fraction of samples that produced no return, in [0, 1]
   */
  [[nodiscard]] float invalidFraction() const noexcept {
    if (rawSampleCount == 0) {
      return 0.0f;
    }
    const float valid = static_cast<float>(points.size());
    const float total = static_cast<float>(rawSampleCount);
    return (total - valid) / total;
  }

  /**
   * Did this revolution actually look all the way round?
   *
   * A different question from "are there many points", and the one that
   * matters for safety: a scan with 400 dropped samples and a scan of an
   * empty corridor can both arrive nearly empty. Anything steering the robot
   * should consult this, not just size().
   *
   * Deliberately independent of how many points there are. A revolution in
   * which every sample was a no-return looked everywhere and saw nothing in
   * range -- full coverage, zero points. Whether that is believable indoors
   * is a separate judgement; invalidFraction() is the input for it.
   *
   * @param maxGapRad The widest tolerable sampling gap. The default is
   *        kUnobservedGapRad; do not go below ~9 degrees without new
   *        measurements, since that is where healthy revolutions already sit.
   * @return true if samples arrived and no sampling gap exceeds maxGapRad
   */
  [[nodiscard]] bool isCoverageOk(float maxGapRad = kUnobservedGapRad) const noexcept {
    return rawSampleCount > 0 && worstSamplingGapRad <= maxGapRad;
  }

  /**
   * Was this bearing looked at in this revolution?
   *
   * "Is anything at bearing B?" is only answerable when this is true.
   * Resolution is kUnobservedGapRad: a gap narrower than that counts as
   * observed, as the ordinary unevenness of this sensor's sampling.
   *
   * @param bearingRad A robot-frame bearing in [-pi, +pi)
   * @return true unless the bearing lies inside an unobserved arc, or
   *         nothing was sampled at all
   */
  [[nodiscard]] bool isObserved(float bearingRad) const noexcept {
    if (rawSampleCount == 0) {
      return false;
    }
    for (const Arc &arc : unobserved) {
      if (arc.contains(bearingRad)) {
        return false;
      }
    }
    return true;
  }
};

// Point is copied in bulk and lives in vectors thousands long; if it ever
// grows past a machine word pair, that is a decision someone should make
// deliberately rather than discover in a profile.
static_assert(sizeof(Point) <= 8, "Point grew -- was that intended?");

}  // namespace Heron

#endif  // HERON_LIDAR_SCAN_H
