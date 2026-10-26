# Heron C++ tree.
#
# Layering, enforced by what each target is allowed to link:
#
#   heron_lidar_types        public headers (Lidar.h, Scan.h, SectorCheck.h)
#   heron_rpraw,             below the boundary: the capture format, the
#   heron_scan_builder,      SDK-free scan builder, and the SDK channels
#   heron_lidar_channels     for record and replay
#   heron_lidar              the adapter. The only production library that
#                            links rplidar_sdk, and it does so PRIVATELY
#   probes/, tools/, tests/  programs. Probes may use the SDK directly; not
#                            production, same spirit as python/probes/
#   (later) robot code       links heron_lidar, never rplidar_sdk
#
# See docs/lidar-cpp-design.md for why the boundary is where it is, and
# docs/cpp-build.md for what each target is.

# -- the boundary type -------------------------------------------------------
#
# Header-only. An INTERFACE target rather than loose include paths, so the
# adapter links it rather than reaching into the directory, and consumers
# inherit the include path transitively.
add_library(heron_lidar_types INTERFACE)
target_include_directories(heron_lidar_types INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

# -- capture format and SDK channels ------------------------------------------
#
# Below the boundary. heron_rpraw is our own file format and needs no SDK.
# heron_lidar_channels plugs into the SDK's IChannel to record a session
# (TapChannel) or replay one in lockstep (ReplayChannel); it links the SDK
# publicly, because its headers expose SDK types. Probes, tests and -- later --
# the adapter may link it. Core code never does.
add_library(heron_rpraw STATIC src/lidar/Rpraw.cpp)
target_include_directories(heron_rpraw PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(heron_rpraw PRIVATE heron_warnings)

add_library(heron_lidar_channels STATIC
    src/lidar/TapChannel.cpp
    src/lidar/ReplayChannel.cpp)
target_link_libraries(heron_lidar_channels
    PUBLIC heron_rpraw rplidar_sdk
    PRIVATE heron_warnings)

# -- the adapter's pure parts -------------------------------------------------
#
# SDK-free: turns raw samples into a Scan (gap detection, transform, filter,
# sort). Kept apart from the SDK-facing code so it is testable on its own.
add_library(heron_scan_builder STATIC src/lidar/ScanBuilder.cpp)
target_include_directories(heron_scan_builder PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(heron_scan_builder
    PUBLIC heron_lidar_types
    PRIVATE heron_warnings)

# -- the adapter ---------------------------------------------------------------
#
# The production LiDAR library. Public surface: Lidar.h and Scan.h only. It
# links the SDK and the channels PRIVATELY, so robot code that links this gets
# no SDK include path: the boundary is enforced by the build, not by habit.
add_library(heron_lidar STATIC src/lidar/Lidar.cpp)
target_link_libraries(heron_lidar
    PUBLIC heron_lidar_types
    PRIVATE heron_scan_builder heron_lidar_channels heron_warnings)

# -- remote control: FlySky iBUS ----------------------------------------------
#
# Independent of everything above. heron_ibus is pure -- frame parser, stats,
# the stale-link rule, failsafe detection, normalization -- and does no I/O, so the tests drive it
# with bytes. heron_ibus_port is the receive-only termios port. They are
# separate so a consumer, or a test, can take the parser without the port.
# See docs/remote-ibus-protocol.md.
add_library(heron_ibus STATIC
    src/remote/Failsafe.cpp
    src/remote/IBusParser.cpp
    src/remote/Normalize.cpp)
target_include_directories(heron_ibus PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(heron_ibus PRIVATE heron_warnings)

add_library(heron_ibus_port STATIC src/remote/IBusPort.cpp)
target_include_directories(heron_ibus_port PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
# PUBLIC heron_ibus: IBusReceiver.h (header-only, here) combines the port with
# the parser and the link rule.
target_link_libraries(heron_ibus_port PUBLIC heron_ibus PRIVATE heron_warnings)

# What robot code links: RemoteControl, a reader thread publishing the newest
# frame and link state, shaped like heron_lidar. docs/remote-cpp-design.md.
find_package(Threads REQUIRED)
add_library(heron_remote STATIC src/remote/RemoteControl.cpp)
target_link_libraries(heron_remote
    PUBLIC heron_ibus_port
    PRIVATE Threads::Threads heron_warnings)

# -- odometry: SparkFun OTOS over I2C -----------------------------------------
#
# Independent of the LiDAR and the remote. heron_otos is pure: the register
# map, its scalings and the conversions to SI units, with no I/O. Its codec
# header is private (src/), so robot code never sees a register number; the
# public headers in include/heron/odometry/ speak SI units only. See
# docs/odometry-cpp-design.md.
add_library(heron_otos STATIC src/odometry/OtosCodec.cpp)
target_include_directories(heron_otos
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(heron_otos PRIVATE heron_warnings)

# The sensor on a bus: I2cBus (an interface, so tests can fake the bus on the
# host), LinuxI2cBus (i2c-dev; on other systems its constructor throws, so
# everything above still builds), and OtosDevice, the start-up sequence. Its
# headers are private too; tests and probes are given the path.
add_library(heron_otos_device STATIC
    src/odometry/LinuxI2cBus.cpp
    src/odometry/OtosDevice.cpp)
target_include_directories(heron_otos_device PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(heron_otos_device PUBLIC heron_otos PRIVATE heron_warnings)

# What robot code links: Odometry, a reader thread publishing the newest
# reading, shaped like heron_lidar and heron_remote. Its public header,
# include/heron/odometry/Odometry.h, names no register and no bus type.
add_library(heron_odometry STATIC src/odometry/Odometry.cpp)
target_include_directories(heron_odometry PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(heron_odometry
    PUBLIC heron_otos_device
    PRIVATE Threads::Threads heron_warnings)

# -- motor link: the shared protocol ------------------------------------------
#
# Header-only, in protocol/motor/ at the top of the repository, because the
# Arduino firmware compiles the same headers. heron_motor_protocol is how the
# Pi side takes them. The AVR compiler has no C++ standard library, so the
# headers must not use it: heron_motor_protocol_nostd compiles them with
# -nostdinc++, and fails the host build if one does. docs/motor-protocol.md.
add_library(heron_motor_protocol INTERFACE)
target_include_directories(heron_motor_protocol INTERFACE
    "${PROJECT_SOURCE_DIR}/protocol/motor/include")

add_library(heron_motor_protocol_nostd OBJECT
    "${PROJECT_SOURCE_DIR}/protocol/motor/check/NoStdCheck.cpp")
target_compile_options(heron_motor_protocol_nostd PRIVATE -nostdinc++)
target_link_libraries(heron_motor_protocol_nostd PRIVATE heron_motor_protocol heron_warnings)

# The firmware's logic: MotorController and MotorOutput, in firmware/motor/lib/ because
# PlatformIO compiles it into the Arduino firmware. CMake compiles the same
# file for the host, so the tests run with ctest like everything else. It is
# shared AVR code too, so it is built with -nostdinc++ here as well.
add_library(heron_motor_controller STATIC
    "${PROJECT_SOURCE_DIR}/firmware/motor/lib/MotorController/src/MotorController.cpp"
    "${PROJECT_SOURCE_DIR}/firmware/motor/lib/MotorController/src/MotorOutput.cpp")
target_include_directories(heron_motor_controller PUBLIC
    "${PROJECT_SOURCE_DIR}/firmware/motor/lib/MotorController/src")
target_compile_options(heron_motor_controller PRIVATE -nostdinc++)
target_link_libraries(heron_motor_controller
    PUBLIC heron_motor_protocol
    PRIVATE heron_warnings)

# The Pi's serial port to the Arduino: termios, read and write. POSIX, so it
# builds on Linux and macOS but never for the AVR. The motor library of
# step 4 (heron_motor) will be built on it.
add_library(heron_motor_port STATIC src/motor/MotorPort.cpp)
target_include_directories(heron_motor_port PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(heron_motor_port PRIVATE heron_warnings)

# What robot code links to drive: Motors -- the start-up handshake, a reader
# thread publishing the newest Status, drive() for the control loop -- shaped
# like heron_remote and heron_odometry. Links heron_profile publicly: its
# configuration holds the rig profile's [motor] section.
# docs/motor-cpp-design.md.
add_library(heron_motor STATIC src/motor/Motors.cpp)
target_link_libraries(heron_motor
    PUBLIC heron_motor_port heron_motor_protocol heron_profile
    PRIVATE Threads::Threads heron_warnings)

# -- rig profiles ---------------------------------------------------------------
#
# Each physical rig's measured numbers, read from profiles/<rig>.conf and
# checked, and which rig a Pi is (~/heron/rig). Shared by every workstream
# that needs per-rig values; the motor section is the first. Pure apart from
# reading files. docs/rig-profiles.md.
add_library(heron_profile STATIC src/profile/RigProfile.cpp)
target_include_directories(heron_profile PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(heron_profile PRIVATE heron_warnings)

# -- tests -------------------------------------------------------------------

add_executable(ibus_parser_test tests/IBusParserTest.cpp)
target_link_libraries(ibus_parser_test PRIVATE heron_ibus heron_warnings)
add_test(NAME ibus_parser_test COMMAND ibus_parser_test)

add_executable(normalize_test tests/NormalizeTest.cpp)
target_link_libraries(normalize_test PRIVATE heron_ibus heron_warnings)
add_test(NAME normalize_test COMMAND normalize_test)

add_executable(failsafe_test tests/FailsafeTest.cpp)
target_link_libraries(failsafe_test PRIVATE heron_ibus heron_warnings)
add_test(NAME failsafe_test COMMAND failsafe_test)

add_executable(remote_control_test tests/RemoteControlTest.cpp)
target_link_libraries(remote_control_test PRIVATE heron_remote Threads::Threads heron_warnings)
add_test(NAME remote_control_test COMMAND remote_control_test)

# The codec header is private to the library, so the test is given its path.
add_executable(otos_codec_test tests/OtosCodecTest.cpp)
target_include_directories(otos_codec_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(otos_codec_test PRIVATE heron_otos heron_warnings)
add_test(NAME otos_codec_test COMMAND otos_codec_test)

add_executable(otos_device_test tests/OtosDeviceTest.cpp)
target_include_directories(otos_device_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(otos_device_test PRIVATE heron_otos_device heron_warnings)
add_test(NAME otos_device_test COMMAND otos_device_test)

add_executable(odometry_test tests/OdometryTest.cpp)
target_include_directories(odometry_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
target_link_libraries(odometry_test PRIVATE heron_odometry Threads::Threads heron_warnings)
add_test(NAME odometry_test COMMAND odometry_test)

add_executable(motor_protocol_test tests/MotorProtocolTest.cpp)
target_link_libraries(motor_protocol_test PRIVATE heron_motor_protocol heron_warnings)
add_test(NAME motor_protocol_test COMMAND motor_protocol_test)

add_executable(motor_controller_test tests/MotorControllerTest.cpp)
target_link_libraries(motor_controller_test PRIVATE heron_motor_controller heron_warnings)
add_test(NAME motor_controller_test COMMAND motor_controller_test)

# Motors against a fake Arduino on a pty, which runs the firmware's own
# logic (heron_motor_controller) on the host.
add_executable(motors_test tests/MotorsTest.cpp)
target_link_libraries(motors_test
    PRIVATE heron_motor heron_motor_controller Threads::Threads heron_warnings)
add_test(NAME motors_test COMMAND motors_test)

add_executable(rig_profile_test tests/RigProfileTest.cpp)
target_link_libraries(rig_profile_test PRIVATE heron_profile heron_warnings)
add_test(NAME rig_profile_test COMMAND rig_profile_test)

# Every profile in the repository must load, so a broken edit fails the tests
# rather than a robot's start-up. CONFIGURE_DEPENDS: a new profile is picked
# up without re-running CMake by hand.
file(GLOB HERON_PROFILES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/profiles/*.conf")
add_test(NAME rig_profiles_check COMMAND rig_profile check ${HERON_PROFILES})

add_executable(scan_test tests/ScanTest.cpp)
target_link_libraries(scan_test PRIVATE heron_lidar_types heron_warnings)
add_test(NAME scan_test COMMAND scan_test)

add_executable(replay_test tests/ReplayTest.cpp)
target_link_libraries(replay_test PRIVATE heron_lidar_channels heron_warnings)
add_test(NAME replay_test COMMAND replay_test)

add_executable(scan_builder_test tests/ScanBuilderTest.cpp)
target_link_libraries(scan_builder_test PRIVATE heron_scan_builder heron_warnings)
add_test(NAME scan_builder_test COMMAND scan_builder_test)

add_executable(sector_check_test tests/SectorCheckTest.cpp)
target_link_libraries(sector_check_test PRIVATE heron_lidar_types heron_warnings)
add_test(NAME sector_check_test COMMAND sector_check_test)

# Not a ctest: needs recordings, which are git-ignored. Driven by
# python/tools/lidar_gapcheck.py against the Python reference.
add_executable(scan_builder_check tests/ScanBuilderCheck.cpp)
target_link_libraries(scan_builder_check PRIVATE heron_scan_builder heron_warnings)

# Not a ctest either: runs the Lidar adapter against a recording. Note it
# links only heron_lidar, as robot code would, and so sees no SDK header.
add_executable(lidar_replay_check tests/LidarReplayCheck.cpp)
target_link_libraries(lidar_replay_check PRIVATE heron_lidar heron_warnings)

# -- probes ------------------------------------------------------------------
#
# These exist before the adapter does, on purpose: the first thing worth
# proving is that the SDK builds, links and talks to the device at all.

add_executable(lidar_info probes/LidarInfo.cpp)
target_link_libraries(lidar_info PRIVATE rplidar_sdk heron_warnings)

# The probe includes SDK headers directly. That is allowed here and nowhere
# else; see the layering note above.

add_executable(lidar_record probes/LidarRecord.cpp)
target_link_libraries(lidar_record PRIVATE heron_lidar_channels heron_warnings)

# The adapter, live. Links only heron_lidar: like robot code, it sees no
# SDK header, so it also proves the boundary holds.
add_executable(lidar_adapter probes/LidarAdapter.cpp)
target_link_libraries(lidar_adapter PRIVATE heron_lidar heron_warnings)

# Nearest obstacle in a sector, live or from a recording. A playable test of
# the whole chain: adapter, Scan, and the consumer rules in SectorCheck.h.
add_executable(lidar_nearest probes/LidarNearest.cpp)
target_link_libraries(lidar_nearest PRIVATE heron_lidar heron_warnings)

# RemoteControl, live: what a 50 Hz robot loop sees. Links only heron_remote,
# like robot code, so it also proves that library stands on its own.
add_executable(remote_adapter probes/RemoteAdapter.cpp)
target_link_libraries(remote_adapter PRIVATE heron_remote heron_warnings)

# Motors, live: what a 50 Hz robot loop sees, driving the board. Links only
# heron_motor, like robot code. Loads this Pi's rig profile.
add_executable(motor_adapter probes/MotorAdapter.cpp)
target_link_libraries(motor_adapter PRIVATE heron_motor heron_warnings)

# Odometry, live: what a 50 Hz robot loop sees. Links only heron_odometry and
# includes only its public header, like robot code.
add_executable(odometry_adapter probes/OdometryAdapter.cpp)
target_link_libraries(odometry_adapter PRIVATE heron_odometry heron_warnings)

# OTOS over I2C: does the Pi 3 read the odometry sensor reliably? Uses the
# kernel's I2C interface (linux/i2c-dev.h), so it builds only for Linux --
# the Pi -- and not on a macOS host. docs/odometry-otos-reference.md §10.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(otos_bus probes/OtosBus.cpp)
    target_link_libraries(otos_bus PRIVATE heron_warnings)

    # The start-up sequence (IMU calibration, reset), then a still board's
    # status, heading drift and raw accelerometer.
    add_executable(otos_start probes/OtosStart.cpp)
    target_link_libraries(otos_start PRIVATE heron_warnings)

    # Interactive: push the sensor a measured distance, compare.
    add_executable(otos_move probes/OtosMove.cpp)
    target_link_libraries(otos_move PRIVATE heron_warnings)

    # Interactive: spin the sensor a counted number of turns, compare.
    add_executable(otos_spin probes/OtosSpin.cpp)
    target_link_libraries(otos_spin PRIVATE heron_warnings)

    # OtosDevice on the real sensor: the library's start-up, then snapshots.
    add_executable(otos_device probes/OtosDeviceLive.cpp)
    target_include_directories(otos_device PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry")
    target_link_libraries(otos_device PRIVATE heron_otos_device heron_warnings)
endif()

# -- tools -----------------------------------------------------------------------
#
# Programs for working with the LiDAR from the host. The viewer server serves
# the adapter's scans to python/tools/lidar_gui.py; it links only
# heron_lidar, like robot code.
add_executable(lidar_gui_server tools/LidarGuiServer.cpp)
target_link_libraries(lidar_gui_server PRIVATE heron_lidar heron_warnings)

# FlySky iBUS reader: learning and debugging the remote control link from the
# Pi's UART. Links the two iBUS libraries above, as robot code would.
add_executable(ibus_reader tools/IBusReader.cpp)
target_link_libraries(ibus_reader PRIVATE heron_ibus_port heron_warnings)

# The motor link's bench tool: sends commands to the Arduino and runs the
# link checks (docs/RESUME-motor.md). It configures the Arduino from the
# rig's profile, converting it with heron_motor's toMotorConfig(), so the
# tool and the library cannot convert differently. Built for the host as well as the Pi,
# so an Uno can be checked from either.
add_executable(motor_link tools/MotorLink.cpp)
target_link_libraries(motor_link
    PRIVATE heron_motor heron_motor_port heron_motor_protocol heron_profile heron_warnings)

# Checks profile files on the host, and shows a Pi's own profile there.
add_executable(rig_profile tools/RigProfileTool.cpp)
target_link_libraries(rig_profile PRIVATE heron_profile heron_warnings)
