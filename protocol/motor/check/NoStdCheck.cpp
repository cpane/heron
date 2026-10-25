// File        : NoStdCheck.cpp
// Author      : Chris Pane
// Description : Compiles the shared motor protocol headers without the C++
//               standard library, as the AVR compiler will.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Built with -nostdinc++, so a C++ standard header included by the protocol
// fails the host build rather than the firmware's. It also instantiates the
// parser's template, which a header alone would not compile.

#include "heron/motor/FrameParser.h"
#include "heron/motor/Protocol.h"

namespace Heron::Motor {

size_t noStdCheck(const uint8_t *bytes, size_t count) {
  FrameParser parser;
  size_t driven = 0;
  return parser.feed(bytes, count, [&driven](const Frame &frame) {
    DriveCommand cmd;
    if (decodeDrive(frame, cmd)) {
      ++driven;
    }
  }) + driven;
}

}  // namespace Heron::Motor
