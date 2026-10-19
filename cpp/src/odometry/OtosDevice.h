// File        : OtosDevice.h
// Author      : Chris Pane
// Description : The OTOS on a bus, synchronous: the start-up sequence, and
//               one snapshot at a time.
//
// Copyright (c) 2026 Chris Pane
// SPDX-License-Identifier: MIT
// Licensed under the MIT License; see LICENSE in the repository root.

// The layer between the codec and the reader thread: Odometry owns one and
// calls start() once, then readSnapshot() every period. Private, like the
// codec, because its results carry the raw status byte and register-level
// detail (docs/odometry-cpp-design.md §1).
//
// start() is the sequence docs/odometry-otos-reference.md §4 found necessary:
// without the calibration and the reset, a still board read a tilt warning
// and drifted 12 deg/min. It takes about 0.65 s, and the sensor must be
// still and level throughout (design §3).

#ifndef HERON_ODOMETRY_OTOS_DEVICE_H
#define HERON_ODOMETRY_OTOS_DEVICE_H

#include <chrono>
#include <system_error>

#include "I2cBus.h"
#include "OtosCodec.h"
#include "heron/odometry/OtosConfig.h"

namespace Heron::Odometry {

/**
 * How long start() waits, and how often it polls. Fixed in production;
 * tests shorten the timeouts.
 */
struct OtosTiming {
  std::chrono::milliseconds poll{10};
  std::chrono::milliseconds calibrationTimeout{2000};  // measured 609-619 ms
  std::chrono::milliseconds resetTimeout{500};         // measured 11 ms
};

/**
 * What start() found and did.
 */
struct OtosStartup {
  OtosVersion hardware{};
  OtosVersion firmware{};
  Duration calibrationTime{};
  Duration resetTime{};
  OtosSnapshot first{};  // the read that proved the snapshot path
};

/**
 * An OTOS on a bus.
 *
 * Thread Safety: Not thread-safe. External synchronization required.
 *
 * Usage:
 * @code
 * LinuxI2cBus bus("/dev/i2c-1", kOtosAddress);
 * OtosDevice otos(bus);
 * const OtosStartup up = otos.start(config);   // ~0.65 s; keep it still
 * OtosSnapshot s;
 * if (!otos.readSnapshot(s)) { use(s); }
 * @endcode
 */
class OtosDevice {
  public:
    /**
     * @param bus The bus the sensor is on; must outlive this object
     * @param timing Poll interval and timeouts for start()
     */
    explicit OtosDevice(I2cBus &bus, const OtosTiming &timing = OtosTiming{})
        : m_bus(bus), m_timing(timing) {}

    /**
     * Runs the start-up sequence: identify, calibrate the IMU, write and
     * verify the scalars, write the offset, reset tracking, read once.
     *
     * @param config Scalars and offset; checked first, before the bus is used
     * @return Versions, timings and the first snapshot
     * @throws std::invalid_argument for a bad config
     * @throws std::system_error if a transfer fails
     * @throws std::runtime_error if the product ID is wrong, the scalars do
     *         not read back as written, or a self-clearing register does not
     *         clear in time
     */
    OtosStartup start(const OtosConfig &config);

    /**
     * Reads and decodes one 37-byte snapshot.
     *
     * @param out The snapshot; unchanged on failure
     * @return Empty on success; the bus error otherwise
     */
    [[nodiscard]] std::error_code readSnapshot(OtosSnapshot &out);

  private:
    // Writes a self-clearing register and polls it until it reads 0.
    Duration writeAndWait(std::uint8_t reg, std::uint8_t value,
                          std::chrono::milliseconds timeout, const char *what);

    I2cBus &m_bus;
    OtosTiming m_timing;
};

}  // namespace Heron::Odometry

#endif  // HERON_ODOMETRY_OTOS_DEVICE_H
