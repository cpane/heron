// File        : Failsafe.h
// Author      : Chris Pane
// Description : Radio-loss detection for the remote control: recognises the
//               failsafe value the transmitter set on one channel.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Pure, no I/O. Why it is needed (docs/remote-ibus-protocol.md §5, measured
// on our FS-iA6B): the receiver never stops sending frames when the radio is
// lost, so the stale rule cannot see it. Channels whose failsafe is Off
// freeze; a channel with a failsafe value moves to it about 1 s after the
// loss, with no flag in the top bits. The only sign of radio loss is
// therefore a channel reading the value the transmitter was told to fail to.
//
// The channel and value belong to the robot, not to this library: they are
// the robot's arm switch and its disarmed position (docs/remote-cpp-design.md
// §4 and §5). A failsafe frame and a deliberate disarm look the same, and are
// meant to: both mean stop.

#ifndef HERON_REMOTE_FAILSAFE_H
#define HERON_REMOTE_FAILSAFE_H

#include <cstddef>
#include <cstdint>

#include "heron/remote/IBusParser.h"

namespace Heron::Remote {

/**
 * Which channel carries a failsafe value, and what that value reads as.
 */
struct FailsafeConfig {
  std::size_t channel{0};       // 0-based slot: CH7 is 6
  std::uint16_t value{1000};    // what the channel reads in failsafe, 12-bit
  std::uint16_t tolerance{50};  // value +- this, inclusive, counts as failsafe
};

/**
 * Checks a failsafe configuration.
 *
 * @param config The configuration to check
 * @throws std::invalid_argument if channel is not below kIBusChannelCount, or
 *         value does not fit in the 12 value bits (it could never match)
 */
void checkFailsafeConfig(const FailsafeConfig &config);

/**
 * Says whether a frame shows the transmitter's failsafe on the configured
 * channel.
 *
 * Compares channelValue(), the low 12 bits. A channel word with any of its
 * top 4 bits set also counts as failsafe: none was ever seen on our receiver,
 * so such a word is unexplained, and an unexplained word on the arm channel
 * must not read as "armed".
 *
 * @param frame A decoded frame
 * @param config The failsafe channel, value and tolerance
 * @return true when the channel is within tolerance of the failsafe value, or
 *         carries a non-zero high nibble
 * @throws std::invalid_argument for a bad config (see checkFailsafeConfig())
 */
[[nodiscard]] bool isFailsafe(const IBusFrame &frame, const FailsafeConfig &config);

}  // namespace Heron::Remote

#endif  // HERON_REMOTE_FAILSAFE_H
