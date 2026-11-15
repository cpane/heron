// File        : BenchTable.h
// Author      : Chris Pane
// Description : For the tests: the bench rig's hardwood, its motion table
//               and [geometry], read from the profiles as heron_robot reads
//               them.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// What this is
// ------------
// The simulator, the planner and the scenarios test against the robot as
// measured (design §19, P17), so they need the bench rig's table: the one
// in profiles/surfaces/bench-mecanum/hardwood-1.conf, measured 2026-11-14.
// This reads profiles/bench.conf and puts the hardwood's numbers in place
// (heron_profile's onSurface(), I22), the way heron_robot does at
// start-up, so a test never carries a copy of the table that could drift
// from the profile. heron_robot_core's toMotionTable() does the same
// conversion; it is repeated here so these tests need only heron_profile
// and heron_motion.
//
// The profile is found in the repository on the Mac (HERON_PROFILE_DIR,
// set by CMake for the tests that include this) and in ~/heron/profiles on
// a Pi, where deploy_cpp.sh puts it.
//
// Usage
// -----
//   const Heron::Test::BenchTable bench = Heron::Test::benchHardwood();
//   bench.motion->predict(0.0, -1.0);   // a full strafe right: about 0.27 m/s
//   bench.geometry.maxSpeed;            // 0.42

#ifndef HERON_TESTS_BENCH_TABLE_H
#define HERON_TESTS_BENCH_TABLE_H

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

#include "heron/motion/MotionTable.h"
#include "heron/profile/RigProfile.h"

namespace Heron::Test {

/** The bench rig on its hardwood: the table and the floor's [geometry]. */
struct BenchTable {
  std::shared_ptr<const Motion::MotionTable> motion;
  Profile::GeometryProfile geometry;
};

/** @throws std::runtime_error if the profile cannot be read or has no table */
inline BenchTable benchHardwood() {
  std::filesystem::path file = Profile::heronHome() / "profiles" / "bench.conf";
#ifdef HERON_PROFILE_DIR
  const std::filesystem::path repo = std::filesystem::path(HERON_PROFILE_DIR) / "bench.conf";
  if (std::filesystem::exists(repo)) {
    file = repo;
  }
#endif
  const Profile::RigProfile hw = Profile::onSurface(Profile::loadRigProfile(file), "hardwood-1");
  const Profile::SurfaceProfile *surface = Profile::findSurface(hw, "hardwood-1");
  if (surface == nullptr || surface->points.empty() || !hw.geometry) {
    throw std::runtime_error("benchHardwood() : " + file.string() + " has no hardwood-1 table");
  }
  std::vector<Motion::MotionPoint> points;
  for (const Profile::SurfacePoint &p : surface->points) {
    points.push_back(Motion::MotionPoint{p.directionDeg * Geometry::kDegToRad, p.size,
                                         Geometry::Twist{p.vx, p.vy, p.wz}});
  }
  return BenchTable{std::make_shared<const Motion::MotionTable>(std::move(points), surface->mirror),
                    *hw.geometry};
}

}  // namespace Heron::Test

#endif  // HERON_TESTS_BENCH_TABLE_H
