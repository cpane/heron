// File        : MotorControllerTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the motor firmware's logic -- commands, the
//               watchdog, Status -- on the host, with made-up frames and times.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// MotorController is the firmware's decision-making, compiled here from the
// same file that goes into the Arduino (firmware/motor/lib/). Frames are built
// with Protocol.h's encoders and fed through a real FrameParser, as on the
// board; time is whatever each test says it is. Same style as
// IBusParserTest.cpp -- deliberately no test framework yet.

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "MotorController.h"
#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"

namespace {

using namespace Heron::Motor;

std::int32_t s_failures = 0;

void check(const char *what, bool ok) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) {
    ++s_failures;
  }
}

// The firmware's receive path, minus the serial port: bytes through a
// parser into the controller, all stamped with one time.
struct Board {
  FrameParser parser;
  MotorController controller;

  void receive(const std::uint8_t *frame, std::size_t length, std::uint32_t nowMs) {
    parser.feed(frame, length,
                [this, nowMs](const Frame &f) { controller.onFrame(f, nowMs); });
  }

  void drive(std::uint8_t sequence, std::int16_t m1, std::int16_t m2, std::int16_t m3,
             std::int16_t m4, std::uint32_t nowMs) {
    DriveCommand cmd;
    cmd.sequence = sequence;
    cmd.wheels[0] = m1;
    cmd.wheels[1] = m2;
    cmd.wheels[2] = m3;
    cmd.wheels[3] = m4;
    std::uint8_t out[kMaxFrameLen];
    receive(out, encodeDrive(cmd, out, sizeof(out)), nowMs);
  }

  void stop(std::uint8_t sequence, std::uint32_t nowMs) {
    std::uint8_t out[kMaxFrameLen];
    receive(out, encodeStop(StopCommand{sequence}, out, sizeof(out)), nowMs);
  }

  void info(std::uint8_t sequence, std::uint32_t nowMs) {
    std::uint8_t out[kMaxFrameLen];
    receive(out, encodeInfoRequest(InfoRequest{sequence}, out, sizeof(out)), nowMs);
  }

  [[nodiscard]] Status status() const { return controller.makeStatus(parser.getStats(), 0, 0); }
};

bool appliedIs(const MotorController &c, std::int16_t m1, std::int16_t m2, std::int16_t m3,
               std::int16_t m4) {
  return c.getApplied(0) == m1 && c.getApplied(1) == m2 && c.getApplied(2) == m3 &&
         c.getApplied(3) == m4;
}

}  // namespace

int main() {
  std::printf("Start-up\n");

  {
    Board b;
    const Status s = b.status();
    check("stopped, reason boot", !b.controller.isRunning() &&
                                      s.state == MotorState::Stopped &&
                                      s.stopReason == StopReason::Boot);
    check("all applied zero", appliedIs(b.controller, 0, 0, 0, 0));
    b.controller.update(10'000);
    check("the watchdog does not fire before any Drive", b.status().watchdogStops == 0);
  }

  std::printf("Commands\n");

  {
    Board b;
    b.drive(7, 100, -200, 300, -400, 1000);
    const Status s = b.status();
    check("drive: running, no stop reason",
          b.controller.isRunning() && s.state == MotorState::Running &&
              s.stopReason == StopReason::None);
    check("drive: target and applied as commanded, in order",
          appliedIs(b.controller, 100, -200, 300, -400) && s.target[0] == 100 &&
              s.target[3] == -400 && s.applied[1] == -200);
    check("drive: sequence echoed", s.lastSequence == 7);
  }

  {
    Board b;
    b.drive(1, 500, 500, 500, 500, 1000);
    b.stop(2, 1010);
    const Status s = b.status();
    check("stop: stopped, reason Stop message",
          !b.controller.isRunning() && s.stopReason == StopReason::StopMessage);
    check("stop: target and applied zero", appliedIs(b.controller, 0, 0, 0, 0) &&
                                               s.target[0] == 0 && s.target[3] == 0);
    check("stop: sequence echoed", s.lastSequence == 2);
    b.controller.update(5000);
    check("stop: not counted as a watchdog stop", b.status().watchdogStops == 0 &&
                                                      b.status().stopReason ==
                                                          StopReason::StopMessage);
    b.drive(3, 10, 20, 30, 40, 5000);
    check("stop: the next Drive runs again", b.controller.isRunning() &&
                                                 appliedIs(b.controller, 10, 20, 30, 40));
  }

  {
    Board b;
    check("info: no Boot owed at start", !b.controller.takeBootRequest());
    b.info(9, 1000);
    check("info: a Boot is owed", b.controller.takeBootRequest());
    check("info: owed once only", !b.controller.takeBootRequest());
    check("info: sequence echoed", b.status().lastSequence == 9);
    check("info: does not start the motors", !b.controller.isRunning());
  }

  std::printf("Watchdog\n");

  {
    Board b;
    b.drive(1, 600, 600, -600, -600, 1000);
    b.controller.update(1250);
    check("still running at exactly 250 ms", b.controller.isRunning());
    b.controller.update(1251);
    const Status s = b.status();
    check("stopped just past 250 ms, reason watchdog",
          !b.controller.isRunning() && s.stopReason == StopReason::Watchdog);
    check("applied zero", appliedIs(b.controller, 0, 0, 0, 0));
    check("target keeps the last command", s.target[0] == 600 && s.target[3] == -600);
    check("counted once", s.watchdogStops == 1);
    b.controller.update(2000);
    check("not counted again while stopped", b.status().watchdogStops == 1);
    b.drive(2, 1, 1, 1, 1, 2000);
    check("the next Drive runs again", b.controller.isRunning());
  }

  {
    Board b;
    std::uint32_t t = 1000;
    bool alwaysRunning = true;
    for (std::uint8_t seq = 0; seq < 100; ++seq, t += 20) {
      b.drive(seq, 1, 1, 1, 1, t);
      b.controller.update(t + 19);
      alwaysRunning = alwaysRunning && b.controller.isRunning();
    }
    check("Drive every 20 ms keeps it running", alwaysRunning && b.status().watchdogStops == 0);
  }

  {
    Board b;
    b.drive(1, 1, 1, 1, 1, 1000);
    b.info(2, 1200);
    b.stop(3, 1210);
    b.drive(4, 1, 1, 1, 1, 1300);
    b.info(5, 1500);
    b.controller.update(1551);
    check("only Drive feeds the watchdog, not InfoRequest",
          b.status().stopReason == StopReason::Watchdog);
  }

  {
    // millis() wraps after about 49 days; the watchdog must not notice.
    Board b;
    const std::uint32_t nearWrap = 0xFFFFFF00;
    b.drive(1, 1, 1, 1, 1, nearWrap);
    b.controller.update(nearWrap + 200);  // wrapped past zero
    check("across the millis() wrap: still running at 200 ms", b.controller.isRunning());
    b.controller.update(nearWrap + 251);
    check("across the millis() wrap: stopped past 250 ms", !b.controller.isRunning());
  }

  std::printf("Status timing\n");

  {
    MotorController c;
    check("the first is due at once", c.isStatusDue(5000));
    check("not again in the same period", !c.isStatusDue(5000) && !c.isStatusDue(5099));
    check("due 100 ms later", c.isStatusDue(5100));
    check("and 100 ms after that", !c.isStatusDue(5150) && c.isStatusDue(5200));
  }

  {
    MotorController c;
    c.isStatusDue(5000);
    check("a late loop still sends", c.isStatusDue(5130));
    check("on schedule again, not drifting", !c.isStatusDue(5199) && c.isStatusDue(5200));
  }

  {
    MotorController c;
    c.isStatusDue(5000);
    check("after a 1 s stall: one Status", c.isStatusDue(6000));
    check("after a 1 s stall: not a burst of the missed ones",
          !c.isStatusDue(6000) && !c.isStatusDue(6099) && c.isStatusDue(6100));
  }

  {
    MotorController c;
    const std::uint32_t nearWrap = 0xFFFFFFC0;
    c.isStatusDue(nearWrap);
    check("across the millis() wrap: due 100 ms later",
          !c.isStatusDue(nearWrap + 99) && c.isStatusDue(nearWrap + 100));
  }

  std::printf("Status contents\n");

  {
    Board b;
    b.drive(1, 1, 1, 1, 1, 1000);
    const std::uint8_t noise[] = {0x00, 0x12, 0x34};
    b.receive(noise, sizeof(noise), 1001);
    const Status s = b.controller.makeStatus(b.parser.getStats(), 7400, 812);
    check("link counters come from the parser",
          s.framesReceived == 1 && s.bytesDiscarded == 3 && s.checksumErrors == 0);
    check("battery and loop time come from the caller",
          s.batteryMillivolts == 7400 && s.longestLoopMicros == 812);
  }

  {
    // A frame with a good checksum but an unexpected payload: ignored.
    Board b;
    const std::uint8_t payload[] = {1, 2, 3};
    std::uint8_t out[kMaxFrameLen];
    b.receive(out, encodeFrame(MessageType::Drive, payload, sizeof(payload), out, sizeof(out)),
              1000);
    b.receive(out, encodeFrame(static_cast<MessageType>(0x7F), payload, 1, out, sizeof(out)),
              1000);
    check("wrong length or unknown type: ignored",
          !b.controller.isRunning() && b.status().lastSequence == 0 &&
              b.status().framesReceived == 2);
  }

  std::printf("\n%" PRId32 " failure(s)\n", s_failures);
  return s_failures == 0 ? 0 : 1;
}
