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

# OTOS over I2C: does the Pi 3 read the odometry sensor reliably? Uses the
# kernel's I2C interface (linux/i2c-dev.h), so it builds only for Linux --
# the Pi -- and not on a macOS host. docs/odometry-otos-reference.md §10.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(otos_bus probes/OtosBus.cpp)
    target_link_libraries(otos_bus PRIVATE heron_warnings)
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
