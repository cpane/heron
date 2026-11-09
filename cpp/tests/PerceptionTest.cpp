// File        : PerceptionTest.cpp
// Author      : Chris Pane
// Description : heron_perception: the grid's beams and rolling, addScan()'s
//               checks and epochs, and a turning robot's wall coming out
//               straight.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The N3 row of docs/navigation-cpp-design.md §9, which is design §12.1's
// list: a point marks one cell; a beam clears its line and not its end; a
// no-return clears nothing; rolling keeps obstacles in place; a skewed scan
// of a straight wall comes out straight once placed. Then addScan()'s
// checks and epochs, the worked example of design §3.3, and the wrapper's
// start and stop. The skewed wall comes from the simulator (heron_nav_sim),
// so its skew is the real LiDAR's. Same style as the other tests: no
// framework.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <thread>
#include <vector>

#include "heron/lidar/Lidar.h"
#include "heron/odometry/Odometry.h"
#include "heron/perception/ObstacleGrid.h"
#include "heron/perception/ObstacleMap.h"
#include "heron/perception/Perception.h"
#include "heron/pose/PoseHistory.h"
#include "heron/pose/PoseTracker.h"
#include "nav_sim/Base.h"
#include "nav_sim/Lidar.h"
#include "nav_sim/Otos.h"
#include "nav_sim/World.h"

namespace {

using namespace std::chrono_literals;
using namespace Heron::Perception;
using Heron::Duration;
using Heron::Scan;
using Heron::TimePoint;
using Heron::Geometry::kDegToRad;
using Heron::Geometry::Pose2;
using Heron::Geometry::Twist;
using Heron::Geometry::Vec2;
using Heron::Pose::PoseHistory;
using Heron::Pose::PoseSample;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

bool near(double a, double b, double tolerance) {
  return std::abs(a - b) <= tolerance;
}

/** How many cells hold each kind. */
struct Tally {
  std::int32_t blocked{0};
  std::int32_t free{0};
  std::int32_t changed{0};  // count not 0
};

Tally tally(const ObstacleGrid &grid) {
  Tally t;
  for (const std::int8_t c : grid.counts()) {
    t.blocked += c >= grid.config().blockedAt ? 1 : 0;
    t.free += c <= grid.config().freeAt ? 1 : 0;
    t.changed += c != 0 ? 1 : 0;
  }
  return t;
}

/** A scan by hand: healthy, fresh at `end`, one revolution of 133 ms
 *  starting behind the robot. */
Scan handScan(TimePoint end, std::vector<Heron::Point> points) {
  Scan scan;
  scan.seq = 1;
  scan.tEnd = end;
  scan.tStart = end - 133ms;
  scan.sweepStartRad = 3.0f;
  scan.rawSampleCount = 520;
  scan.worstSamplingGapRad = 8.0f * static_cast<float>(kDegToRad);
  std::sort(points.begin(), points.end(),
            [](const Heron::Point &a, const Heron::Point &b) { return a.bearingRad < b.bearingRad; });
  scan.points = std::move(points);
  return scan;
}

/** A history holding Heron still at `pose` from well before to after t. */
PoseHistory stillAt(const Pose2 &pose, TimePoint t) {
  PoseHistory history;
  history.add(PoseSample{t - 1s, pose, Twist{}});
  history.add(PoseSample{t + 10ms, pose, Twist{}});
  return history;
}

}  // namespace

int main() {
  std::printf("The grid: a point marks one cell\n");
  {
    ObstacleGrid grid;
    const Vec2 origin = grid.origin();
    check("centred on the odom origin, edges on 5 cm",
          near(origin.x, -2.0, 1e-12) && near(origin.y, -2.0, 1e-12));
    grid.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.01}, true);
    const Tally t = tally(grid);
    std::printf("        one beam of 1 m: %d blocked, %d changed\n", t.blocked, t.changed);
    check("one cell blocked, the point's", t.blocked == 1 &&
                                               grid.classify(Vec2{1.01, 0.01}) == Cell::Blocked);
    check("  it is the cell holding (1.01, 0.01): (60, 40)",
          grid.cellOf(Vec2{1.01, 0.01})->ix == 60 && grid.cellOf(Vec2{1.01, 0.01})->iy == 40 &&
              grid.count(CellIndex{60, 40}) == 3);
    check("  a neighbour is not", grid.classify(Vec2{1.06, 0.01}) == Cell::Unknown &&
                                      grid.count(CellIndex{61, 40}) == 0);
  }

  std::printf("The grid: a beam clears its line and not its end\n");
  {
    ObstacleGrid grid;
    grid.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.01}, true);
    check("after one beam, the line is -1: still Unknown",
          grid.count(CellIndex{50, 40}) == -1 && grid.classify(Vec2{0.51, 0.01}) == Cell::Unknown);
    grid.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.01}, true);
    bool lineFree = true;
    for (int ix = 40; ix < 60; ++ix) {
      lineFree = lineFree && grid.count(CellIndex{ix, 40}) == -2;
    }
    check("after two, every cell from the LiDAR's to the point's is Free", lineFree);
    check("  the end is Blocked, +6", grid.count(CellIndex{60, 40}) == 6);
    check("  nothing beyond it changed", grid.count(CellIndex{61, 40}) == 0);
    check("  nor beside the line", grid.count(CellIndex{50, 41}) == 0 &&
                                        grid.count(CellIndex{50, 39}) == 0);

    // A diagonal: Bresenham steps one cell at a time, no gaps.
    ObstacleGrid diagonal;
    diagonal.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.51}, true);
    const Tally d = tally(diagonal);
    std::printf("        a diagonal of 20 by 10 cells: %d changed\n", d.changed);
    check("a diagonal 20 cells by 10: 20 passed and 1 hit", d.changed == 21 && d.blocked == 1);

    ObstacleGrid missed;
    missed.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.01}, false);
    check("hit = false: the line passed, the end unchanged",
          missed.count(CellIndex{59, 40}) == -1 && missed.count(CellIndex{60, 40}) == 0);
  }

  std::printf("The grid: limits and edges\n");
  {
    ObstacleGrid grid;
    for (int i = 0; i < 10; ++i) {
      grid.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.01}, true);
    }
    check("ten hits saturate at +12", grid.count(CellIndex{60, 40}) == 12);
    check("ten passes saturate at -8", grid.count(CellIndex{50, 40}) == -8);
    for (int i = 0; i < 3; ++i) {
      grid.addRay(Vec2{0.01, 0.01}, Vec2{1.51, 0.01}, true);
    }
    check("a cell at +12 crossed 3 times is still Blocked (+9)",
          grid.count(CellIndex{60, 40}) == 9 && grid.classify(Vec2{1.01, 0.01}) == Cell::Blocked);

    ObstacleGrid edge;
    edge.addRay(Vec2{0.01, 0.01}, Vec2{5.01, 0.01}, true);
    check("a point beyond the edge: the line lowered up to the edge",
          edge.count(CellIndex{79, 40}) == -1 && tally(edge).changed == 40 &&
              tally(edge).blocked == 0);
    check("  and outside is Unknown", edge.classify(Vec2{5.01, 0.01}) == Cell::Unknown &&
                                          !edge.cellOf(Vec2{5.01, 0.01}));
    ObstacleGrid outside;
    outside.addRay(Vec2{-5.01, 0.01}, Vec2{-1.49, 0.01}, true);
    check("a beam from outside the grid: only the cells inside change",
          tally(outside).blocked == 1 && outside.count(CellIndex{0, 40}) == -1);
    outside.addRay(Vec2{std::nan(""), 0.0}, Vec2{1.0, 1.0}, true);
    check("a beam from nowhere (NaN): ignored", tally(outside).changed == 11);
  }

  std::printf("The grid: rolling keeps obstacles in place\n");
  {
    ObstacleGrid grid;
    grid.addRay(Vec2{0.01, 0.01}, Vec2{1.01, 0.31}, true);
    grid.addRay(Vec2{0.01, 0.01}, Vec2{-1.89, 0.01}, true);
    grid.follow(Vec2{0.49, 0.0});
    check("Heron 0.49 m from the centre: no shift", near(grid.origin().x, -2.0, 1e-12));
    grid.follow(Vec2{0.83, 0.12});
    const Vec2 o = grid.origin();
    std::printf("        Heron at (0.83, 0.12): origin (%.2f, %.2f)\n", o.x, o.y);
    check("at (0.83, 0.12): shifted to centre its cell, edges still on 5 cm",
          near(o.x, -1.2, 1e-9) && near(o.y, -1.9, 1e-9));
    check("the obstacle at (1.01, 0.31) still Blocked, where it was",
          grid.classify(Vec2{1.01, 0.31}) == Cell::Blocked);
    check("  in another cell index, the same floor cell", grid.cellOf(Vec2{1.01, 0.31})->ix == 44);
    check("the one at (-1.89, 0.01) shifted out: Unknown",
          grid.classify(Vec2{-1.89, 0.01}) == Cell::Unknown);
    grid.follow(Vec2{0.0, 0.0});
    check("back again: what went out stays forgotten",
          grid.classify(Vec2{-1.89, 0.01}) == Cell::Unknown &&
              grid.classify(Vec2{1.01, 0.31}) == Cell::Blocked);
    grid.follow(Vec2{30.0, 0.0});
    check("a jump past the grid: all Unknown", tally(grid).changed == 0);
  }

  std::printf("addScan: the worked example (design §3.3)\n");
  {
    // Heron at (2, 1) facing +y, the LiDAR 6 cm ahead, a box at 30 deg, 1 m.
    const TimePoint t0{100s};
    const Pose2 heron{2.0, 1.0, 90.0 * kDegToRad};
    ObstacleMap map;
    const Scan scan = handScan(t0, {Heron::Point{static_cast<float>(30.0 * kDegToRad), 1.0f}});
    const ScanOutcome outcome =
        addScan(map, scan, stillAt(heron, t0), 1, LidarMount{Vec2{0.06, 0.0}}, t0);
    check("first scan of epoch 1: NewFrame, and added", outcome == ScanOutcome::NewFrame);
    // x = 1.5 is a cell edge, and the float bearing puts the point a hair
    // either side of it: so the check is on the one blocked cell's centre.
    std::optional<Vec2> box;
    for (int iy = 0; iy < kGridCells; ++iy) {
      for (int ix = 0; ix < kGridCells; ++ix) {
        if (map.grid.count(CellIndex{ix, iy}) >= 3) {
          box = map.grid.centreOf(CellIndex{ix, iy});
        }
      }
    }
    std::printf("        the blocked cell's centre: (%.3f, %.3f)\n", box ? box->x : 0.0,
                box ? box->y : 0.0);
    check("one cell Blocked, holding (1.50, 1.93) on the floor",
          tally(map.grid).blocked == 1 && box && near(box->x, 1.5, 0.026) &&
              near(box->y, 1.926, 0.026));
    check("the LiDAR's cell, (2.00, 1.06), passed",
          map.grid.count(*map.grid.cellOf(Vec2{2.0, 1.06})) == -1);
    check("the map records the scan and the pose", map.scanSeq == 1 && map.scanEnd == t0 &&
                                                       map.epoch == 1 &&
                                                       near(map.poseAtScanEnd.y, 1.0, 1e-12));
    check("the grid follows Heron: centred on its cell",
          near(map.grid.centre().x, 2.0, 1e-9) && near(map.grid.centre().y, 1.0, 1e-9));
  }

  std::printf("addScan: a no-return clears nothing\n");
  {
    // 520 samples, one return straight ahead at 1 m: the room to the left,
    // where every sample came back empty, keeps what it held.
    const TimePoint t0{100s};
    ObstacleMap map;
    PoseHistory history = stillAt(Pose2{}, t0);
    map.epoch = 1;
    map.grid.addRay(Vec2{0.0, 0.5}, Vec2{0.0, 1.0}, true);  // something seen before
    const Scan scan = handScan(t0, {Heron::Point{0.0f, 1.0f}});
    check("added", addScan(map, scan, history, 1, LidarMount{}, t0) == ScanOutcome::Added);
    check("the obstacle at (0, 1), in the no-returns, is still Blocked",
          map.grid.classify(Vec2{0.0, 1.0}) == Cell::Blocked);
    check("and nothing on that side was lowered", map.grid.count(*map.grid.cellOf(Vec2{0.0, 0.5})) == -1);
  }

  std::printf("addScan: checks and epochs\n");
  {
    const TimePoint t0{100s};
    const PoseHistory history = stillAt(Pose2{}, t0);
    const Scan scan = handScan(t0, {Heron::Point{0.0f, 1.0f}});
    ObstacleMap map;
    (void)addScan(map, scan, history, 1, LidarMount{}, t0);
    const Tally before = tally(map.grid);

    ObstacleMap stale = map;
    check("300 ms old: added", addScan(stale, scan, history, 1, LidarMount{}, t0 + 300ms) ==
                                   ScanOutcome::Added);
    stale = map;
    check("301 ms old: Stale, map unchanged",
          addScan(stale, scan, history, 1, LidarMount{}, t0 + 301ms) == ScanOutcome::Stale &&
              stale.grid.counts() == map.grid.counts());

    Scan holed = scan;
    holed.worstSamplingGapRad = 20.0f * static_cast<float>(kDegToRad);
    ObstacleMap uncovered = map;
    check("a 20 deg hole: NotCovered, map unchanged",
          addScan(uncovered, holed, history, 1, LidarMount{}, t0) == ScanOutcome::NotCovered &&
              uncovered.grid.counts() == map.grid.counts());

    // History ends 60 ms before the scan does: past the 50 ms extrapolation.
    PoseHistory early;
    early.add(PoseSample{t0 - 1s, Pose2{}, Twist{}});
    early.add(PoseSample{t0 - 60ms, Pose2{}, Twist{}});
    ObstacleMap posed = map;
    check("no pose for the scan's end: NoPose, map unchanged",
          addScan(posed, scan, early, 1, LidarMount{}, t0) == ScanOutcome::NoPose &&
              posed.grid.counts() == map.grid.counts());
    // History starting mid-scan: the points before it have no pose.
    PoseHistory late;
    late.add(PoseSample{t0 - 50ms, Pose2{}, Twist{}});
    late.add(PoseSample{t0, Pose2{}, Twist{}});
    const Scan behind = handScan(t0, {Heron::Point{0.0f, 1.0f}, Heron::Point{2.9f, 1.0f}});
    ObstacleMap half = map;
    check("one point before the history: NoPose, none of the scan added",
          addScan(half, behind, late, 1, LidarMount{}, t0) == ScanOutcome::NoPose &&
              half.grid.counts() == map.grid.counts());

    ObstacleMap restarted = map;
    const ScanOutcome fresh = addScan(restarted, handScan(t0, {Heron::Point{1.5f, 0.5f}}),
                                      stillAt(Pose2{}, t0), 2, LidarMount{}, t0);
    check("epoch 2: NewFrame; the old obstacle gone, the new one in",
          fresh == ScanOutcome::NewFrame && restarted.epoch == 2 &&
              restarted.grid.classify(Vec2{0.0, 1.0}) != Cell::Blocked &&
              tally(restarted.grid).blocked == 1);
    ObstacleMap skipped = map;
    check("epoch 2 with a stale scan: Stale, but the map is still reset",
          addScan(skipped, scan, history, 2, LidarMount{}, t0 + 1s) == ScanOutcome::Stale &&
              skipped.epoch == 2 && tally(skipped.grid).changed == 0);
    std::printf("        the map before: %d blocked, %d changed\n", before.blocked, before.changed);
  }

  std::printf("addScan: a turning robot's wall comes out straight (simulator)\n");
  {
    // A wall at x = 1.2 m across the room; Heron at the origin, driving
    // forward at 0.2 m/s and turning at 1.5 rad/s: a revolution turns it
    // 11 deg. The history is the simulated OTOS, with no drift or noise, so
    // what is measured is the deskewing alone.
    using Heron::NavSim::Base;
    using Heron::NavSim::BaseConfig;
    using Heron::NavSim::Otos;
    using Heron::NavSim::OtosConfig;
    Heron::NavSim::World world;
    world.segments.push_back(Heron::NavSim::Segment{Vec2{1.2, -3.0}, Vec2{1.2, 3.0}});
    TimePoint now{100s};
    Base base(BaseConfig{}, Pose2{}, now);
    Heron::NavSim::LidarConfig lidarConfig;
    lidarConfig.noReturnFraction = 0.3;
    Heron::NavSim::Lidar lidar(world, lidarConfig, now);
    Otos otos(OtosConfig{0.0, 0.0, 1}, now);
    PoseHistory history;
    history.add(otos.read(now, base));
    base.command(Twist{0.2, 0.0, 1.5}, now);
    std::optional<Scan> scan;
    while (now < TimePoint{100s} + 1500ms || !scan) {
      now += 10ms;
      base.advance(now);
      history.add(otos.read(now, base));
      std::optional<Scan> s = lidar.advance(now, base);
      if (s && now >= TimePoint{100s} + 1500ms) {
        scan = std::move(s);
      }
    }
    std::printf("        the scan: %zu points, turning %.2f rad/s\n", scan->size(),
                base.trueVelocity().wz);

    const LidarMount mount{lidarConfig.offset};
    ObstacleMap deskewed;
    const ScanOutcome outcome = addScan(deskewed, *scan, history, 1, mount, now);

    // The same scan, every point placed with the pose at the scan's end.
    const Pose2 atEnd = history.at(scan->tEnd).pose;
    ObstacleMap atEndOnly;
    (void)addScan(atEndOnly, *scan, stillAt(atEnd, scan->tEnd), 1, mount, now);

    auto worstOffWall = [](const ObstacleGrid &grid) {
      double worst = 0.0;
      for (int iy = 0; iy < kGridCells; ++iy) {
        for (int ix = 0; ix < kGridCells; ++ix) {
          if (grid.count(CellIndex{ix, iy}) >= grid.config().blockedAt) {
            worst = std::max(worst, std::abs(grid.centreOf(CellIndex{ix, iy}).x - 1.2));
          }
        }
      }
      return worst;
    };
    const double straight = worstOffWall(deskewed.grid);
    const double bent = worstOffWall(atEndOnly.grid);
    std::printf("        blocked cells off the wall, worst: %.0f mm at each point's own time, "
                "%.0f mm at the scan's end (%d and %d cells)\n",
                1000.0 * straight, 1000.0 * bent, tally(deskewed.grid).blocked,
                tally(atEndOnly.grid).blocked);
    check("added", outcome == ScanOutcome::NewFrame);
    check("the wall seen: over 20 blocked cells", tally(deskewed.grid).blocked > 20);
    check("each point at its own time: every blocked cell within a cell of the wall",
          straight <= 0.05);
    check("all at the scan's end: off by over 10 cm", bent > 0.10);
  }

  std::printf("Perception: the thread starts and stops\n");
  {
    // Neither sensor is started: the wrapper waits, publishes nothing.
    Heron::Odometry::Odometry odometry(Heron::Odometry::OtosConfig{});
    Heron::Pose::PoseTracker tracker(odometry);
    Heron::Lidar::Lidar lidar(Heron::Lidar::LidarConfig{});
    Perception perception(lidar, tracker, PerceptionConfig{});
    check("start()", perception.start());
    check("start() again: a no-op", perception.start());
    std::this_thread::sleep_for(50ms);
    const PerceptionHealth h = perception.getHealth();
    check("no scan: no map, nothing counted",
          perception.getLatest() == nullptr && h.scansAdded == 0 && h.scansSkipped == 0);
    perception.stop();
    perception.stop();
    check("stop() twice: fine", true);
  }

  std::printf("\n%s: %" PRId32 " failure(s)\n", s_failures == 0 ? "OK" : "FAILED", s_failures);
  return s_failures == 0 ? 0 : 1;
}
