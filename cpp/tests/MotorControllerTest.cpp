// File        : MotorControllerTest.cpp
// Author      : Chris Pane
// Description : Behaviour of the motor firmware's logic -- commands, the
//               watchdog, the ramp, Status, the output mapping -- on the host,
//               with made-up frames and times.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// MotorController is the firmware's decision-making, compiled here from the
// same file that goes into the Arduino (firmware/motor/lib/). Frames are built
// with Protocol.h's encoders and fed through a real FrameParser, as on the
// board; time is whatever each test says it is. Most tests use no ramp, so
// applied follows the target on the next update(); the ramp has its own
// section. Same style as
// IBusParserTest.cpp -- deliberately no test framework yet.

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "MotorController.h"
#include "MotorOutput.h"
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

constexpr RampConfig kNoRamp{0};

// The firmware's receive path, minus the serial port: bytes through a
// parser into the controller, all stamped with one time. drive() then runs
// update() at the same time, as the loop does after reading the port.
struct Board {
  explicit Board(const RampConfig &ramp = kNoRamp) : controller(ramp) {}

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
    controller.update(nowMs);
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
    MotorController c(kNoRamp);
    check("the first is due at once", c.isStatusDue(5000));
    check("not again in the same period", !c.isStatusDue(5000) && !c.isStatusDue(5099));
    check("due 100 ms later", c.isStatusDue(5100));
    check("and 100 ms after that", !c.isStatusDue(5150) && c.isStatusDue(5200));
  }

  {
    MotorController c(kNoRamp);
    c.isStatusDue(5000);
    check("a late loop still sends", c.isStatusDue(5130));
    check("on schedule again, not drifting", !c.isStatusDue(5199) && c.isStatusDue(5200));
  }

  {
    MotorController c(kNoRamp);
    c.isStatusDue(5000);
    check("after a 1 s stall: one Status", c.isStatusDue(6000));
    check("after a 1 s stall: not a burst of the missed ones",
          !c.isStatusDue(6000) && !c.isStatusDue(6099) && c.isStatusDue(6100));
  }

  {
    MotorController c(kNoRamp);
    const std::uint32_t nearWrap = 0xFFFFFFC0;
    c.isStatusDue(nearWrap);
    check("across the millis() wrap: due 100 ms later",
          !c.isStatusDue(nearWrap + 99) && c.isStatusDue(nearWrap + 100));
  }

  std::printf("Ramp\n");

  {
    Board b(RampConfig{4});
    b.controller.update(1000);
    b.drive(1, 1000, -1000, 400, 0, 1000);
    check("a Drive alone moves nothing in the same millisecond",
          appliedIs(b.controller, 0, 0, 0, 0) && b.status().target[0] == 1000);
    b.controller.update(1010);
    check("10 ms at 4 per ms: 40 toward each target", appliedIs(b.controller, 40, -40, 40, 0));
    b.controller.update(1100);
    check("100 ms: 400; a target of 400 is reached and held",
          appliedIs(b.controller, 400, -400, 400, 0));
    b.controller.update(1250);
    b.drive(2, 1000, -1000, 400, 0, 1250);  // keep the watchdog fed
    check("250 ms: full", appliedIs(b.controller, 1000, -1000, 400, 0));
  }

  {
    Board b(RampConfig{4});
    b.controller.update(1000);
    std::uint32_t t = 1000;
    for (std::uint8_t seq = 0; t <= 1300; ++seq, t += 20) {
      b.drive(seq, 1000, 0, 0, 0, t);
    }
    check("full forward after 300 ms", b.controller.getApplied(0) == 1000);
    bool smooth = true;
    std::int16_t previous = b.controller.getApplied(0);
    for (std::uint8_t seq = 100; t <= 1900; ++seq, t += 20) {
      b.drive(seq, -1000, 0, 0, 0, t);
      const std::int16_t now = b.controller.getApplied(0);
      smooth = smooth && now <= previous && previous - now <= 80;  // 20 ms at 4 per ms
      previous = now;
    }
    check("a reversal ramps down through zero, never jumping", smooth && previous == -1000);
  }

  {
    Board b(RampConfig{4});
    b.controller.update(1000);
    b.drive(1, 1000, 1000, 1000, 1000, 1000);
    b.controller.update(1100);
    b.stop(2, 1100);
    check("Stop releases at once, not on the ramp", appliedIs(b.controller, 0, 0, 0, 0));
    b.drive(3, 1000, 1000, 1000, 1000, 1200);
    b.controller.update(1210);
    check("after a Stop the next Drive ramps up from zero",
          appliedIs(b.controller, 40, 40, 40, 40));
  }

  {
    Board b(RampConfig{4});
    b.controller.update(1000);
    b.drive(1, 800, 800, 800, 800, 1000);
    b.controller.update(1240);
    b.controller.update(1251);
    check("the watchdog releases at once too", !b.controller.isRunning() &&
                                                  appliedIs(b.controller, 0, 0, 0, 0));
  }

  {
    Board b(RampConfig{4});
    b.controller.update(1000);
    b.drive(1, -1000, 0, 0, 0, 1000);
    b.drive(2, -1000, 0, 0, 0, 1200);  // a 200 ms stall between loop passes
    check("a stalled loop moves only as far as the ramp allows",
          b.controller.getApplied(0) == -800);
  }

  {
    Board b(RampConfig{4});
    const std::uint32_t nearWrap = 0xFFFFFFF0;
    b.controller.update(nearWrap);
    b.drive(1, 1000, 0, 0, 0, nearWrap);
    b.drive(2, 1000, 0, 0, 0, nearWrap + 0x20);  // 32 ms later, past the wrap
    check("across the millis() wrap: 32 ms is 128", b.controller.getApplied(0) == 128);
  }

  std::printf("Output: value to duty and direction\n");

  {
    const MotorOutput zero = toMotorOutput(0, 1, 70);
    check("0: released, duty 0", zero.direction == Direction::Release && zero.duty == 0);
    const MotorOutput one = toMotorOutput(1, 1, 70);
    check("1: the dead band's edge, forward", one.direction == Direction::Forward &&
                                                  one.duty == 70);
    check("500: halfway through the band", toMotorOutput(500, 1, 70).duty == 162);
    const MotorOutput full = toMotorOutput(-1000, 1, 70);
    check("-1000: full, backward", full.direction == Direction::Backward && full.duty == 255);
    check("sign -1 reverses the direction, not the duty",
          toMotorOutput(-1000, -1, 70).direction == Direction::Forward &&
              toMotorOutput(300, -1, 70).direction == Direction::Backward &&
              toMotorOutput(300, -1, 70).duty == toMotorOutput(300, 1, 70).duty);
    check("out of range clamps", toMotorOutput(30000, 1, 70).duty == 255 &&
                                     toMotorOutput(-30000, 1, 70).direction ==
                                         Direction::Backward);
    check("no dead band: 1..1000 spread over 0..255",
          toMotorOutput(1000, 1, 0).duty == 255 && toMotorOutput(500, 1, 0).duty == 127 &&
              toMotorOutput(1, 1, 0).duty == 0 &&
              toMotorOutput(1, 1, 0).direction == Direction::Forward);
    bool rising = true;
    for (std::int16_t v = 2; v <= 1000; ++v) {
      rising = rising && toMotorOutput(v, 1, 70).duty >= toMotorOutput(v - 1, 1, 70).duty;
    }
    check("duty never falls as the value rises", rising);
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
