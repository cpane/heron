// File        : TapChannel.h
// Author      : Chris Pane
// Description : A serial channel that records everything the SDK sends and
//               receives to a .rpraw.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// Below the boundary: includes SDK headers. Only the adapter, probes and
// tests may use it. The overridden methods keep the SDK's own names and
// signatures (open, waitForDataExt, ...), which are the vendor's interface
// and outside this standard.

#ifndef HERON_LIDAR_TAP_CHANNEL_H
#define HERON_LIDAR_TAP_CHANNEL_H

#include <cstddef>

#include <sl_lidar_driver.h>

#include "lidar/Rpraw.h"

namespace Heron::Lidar {

/**
 * Wraps the SDK's own serial channel and records the session.
 *
 * Every call is forwarded to the wrapped channel; each write() and each
 * non-empty read() is also written to a .rpraw as one chunk. Because the
 * chunks are exactly the calls the SDK made, ReplayChannel can later hand the
 * same bytes back in the same pieces.
 *
 * It reports CHANNEL_TYPE_SERIALPORT and forwards setDTR(), so the SDK's
 * motor control works through it unchanged: on the A1, motor on/off IS the
 * DTR line (sl_lidar_driver.cpp, setMotorSpeed()). DTR changes are not
 * recorded -- the .rpraw format has no record for them, and the Python reader
 * would reject one -- which is harmless for replay, since a recording cannot
 * be made to spin a motor anyway.
 *
 * Thread Safety: as thread-safe as the wrapped channel; the recording side is
 * serialised by RprawWriter.
 *
 * Usage:
 * @code
 * RprawWriter writer(path, header);
 * TapChannel tap(*serialPort, writer);
 * driver->connect(&tap);
 * @endcode
 */
class TapChannel final : public sl::ISerialPortChannel {
  public:
    /**
     * @param inner The real serial channel. Not owned; must outlive this.
     * @param writer Where the session is recorded. Not owned; must outlive this.
     */
    TapChannel(sl::ISerialPortChannel &inner, RprawWriter &writer);

    bool open() override;
    void close() override;
    void flush() override;
    bool waitForData(std::size_t size, sl_u32 timeoutInMs, std::size_t *actualReady) override;
    sl_result waitForDataExt(std::size_t &sizeHint, sl_u32 timeoutInMs) override;
    int write(const void *data, std::size_t size) override;
    int read(void *buffer, std::size_t size) override;
    void clearReadCache() override;
    int getChannelType() override;
    void setDTR(bool dtr) override;

  private:
    sl::ISerialPortChannel &m_inner;
    RprawWriter &m_writer;
};

}  // namespace Heron::Lidar

#endif  // HERON_LIDAR_TAP_CHANNEL_H
