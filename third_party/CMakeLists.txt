# The vendor RPLIDAR SDK, built as a static library.
#
# This file lives HERE rather than inside third_party/rplidar_sdk/ because
# that directory is a git submodule. Anything written inside it would be a
# local modification to upstream: it would show as dirty in `git status`,
# fight every `git submodule update`, and be lost on the first version bump.
# Keeping our build description outside the submodule means updating the SDK
# is `git -C third_party/rplidar_sdk checkout <tag>` and nothing else.
#
# The SDK ships GNU Makefiles. We compile its sources into our own target
# instead of invoking them: one build graph, one set of flags, one toolchain,
# and sanitiser or debug builds come free rather than needing a second build
# system taught about them.

set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk/sdk")

if(NOT EXISTS "${SDK_DIR}/include/sl_lidar_driver.h")
    message(FATAL_ERROR
        "The RPLIDAR SDK submodule is not checked out.\n"
        "Run:\n"
        "    git submodule update --init --recursive\n"
        "Expected to find: ${SDK_DIR}/include/sl_lidar_driver.h")
endif()

# Source list mirrors sdk/Makefile rather than globbing. Two reasons: a glob
# silently picks up whatever upstream adds on the next submodule bump, and
# upstream deliberately does NOT build every .cpp in the tree -- see below.
set(SDK_SOURCES
    "${SDK_DIR}/src/sl_lidar_driver.cpp"
    "${SDK_DIR}/src/sl_async_transceiver.cpp"
    "${SDK_DIR}/src/sl_lidarprotocol_codec.cpp"
    "${SDK_DIR}/src/sl_crc.cpp"
    "${SDK_DIR}/src/sl_serial_channel.cpp"
    "${SDK_DIR}/src/sl_tcp_channel.cpp"
    "${SDK_DIR}/src/sl_udp_channel.cpp"
    "${SDK_DIR}/src/hal/thread.cpp"
    "${SDK_DIR}/src/dataunpacker/dataunpacker.cpp"
    "${SDK_DIR}/src/dataunpacker/unpacker/handler_capsules.cpp"
    "${SDK_DIR}/src/dataunpacker/unpacker/handler_hqnode.cpp"
    "${SDK_DIR}/src/dataunpacker/unpacker/handler_normalnode.cpp")

# NOTE: src/rplidar_driver.cpp is deliberately absent. It is the deprecated
# pre-`sl::` API, and upstream's own sdk/Makefile does not build it either --
# so this matches upstream rather than diverging from it.

# -- local patch: one upstream bug, fixed at build time ------------------------
#
# AsyncTransceiver allocates each receive buffer with `new Buffer()`
# (sl_async_transceiver.cpp:331) but unbindAndClose() frees any still queued
# with `delete []` (:255). Mixing the two is undefined behaviour. It only bites
# when disconnect() runs with data still queued, which an overrun backlog
# produces: measured 2026-10-04, replaying the starved-reader capture aborted
# inside disconnect() on macOS ("pointer being freed was not allocated").
# Linux may instead corrupt the heap silently.
#
# The submodule stays untouched. The file is copied into the build tree, the
# one statement is replaced, and the copy is compiled in place of the
# original; its quoted includes still resolve through the src/ include path
# below. The replacement must match exactly once, so an SDK bump that changes
# or fixes the line stops configure and gets the patch reviewed, rather than
# silently skipping it.
set(SDK_TRANSCEIVER_SRC "${SDK_DIR}/src/sl_async_transceiver.cpp")
set(SDK_TRANSCEIVER_BAD "delete [] *itr;")
set(SDK_TRANSCEIVER_FIX "delete *itr;  /* makerbot patch: allocated with new, not new[] */")
file(READ "${SDK_TRANSCEIVER_SRC}" SDK_TRANSCEIVER_TEXT)
string(FIND "${SDK_TRANSCEIVER_TEXT}" "${SDK_TRANSCEIVER_BAD}" SDK_BAD_FIRST)
string(FIND "${SDK_TRANSCEIVER_TEXT}" "${SDK_TRANSCEIVER_BAD}" SDK_BAD_LAST REVERSE)
if(SDK_BAD_FIRST EQUAL -1 OR NOT SDK_BAD_FIRST EQUAL SDK_BAD_LAST)
    message(FATAL_ERROR
        "The RPLIDAR SDK patch no longer applies: expected exactly one\n"
        "    ${SDK_TRANSCEIVER_BAD}\n"
        "in ${SDK_TRANSCEIVER_SRC}.\n"
        "The SDK has changed. Check whether upstream fixed the delete/new\n"
        "mismatch in AsyncTransceiver::unbindAndClose(), then update or remove\n"
        "this patch in third_party/CMakeLists.txt.")
endif()
string(REPLACE "${SDK_TRANSCEIVER_BAD}" "${SDK_TRANSCEIVER_FIX}"
       SDK_TRANSCEIVER_TEXT "${SDK_TRANSCEIVER_TEXT}")
# Write via configure_file so the copy's timestamp only moves when its content
# does; a plain file(WRITE) would force a rebuild on every configure.
set(SDK_PATCH_DIR "${CMAKE_CURRENT_BINARY_DIR}/sdk-patched")
file(WRITE "${SDK_PATCH_DIR}/sl_async_transceiver.cpp.in" "${SDK_TRANSCEIVER_TEXT}")
configure_file("${SDK_PATCH_DIR}/sl_async_transceiver.cpp.in"
               "${SDK_PATCH_DIR}/sl_async_transceiver.cpp" COPYONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SDK_TRANSCEIVER_SRC}")
list(REMOVE_ITEM SDK_SOURCES "${SDK_TRANSCEIVER_SRC}")
list(APPEND SDK_SOURCES "${SDK_PATCH_DIR}/sl_async_transceiver.cpp")

# Platform layer. The SDK ships linux, macOS and win32; we build exactly one.
# Selecting macOS here is what allows the adapter and its tests to build and
# run natively on the development host, with no container and no hardware.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(SDK_ARCH_DIR "${SDK_DIR}/src/arch/linux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(SDK_ARCH_DIR "${SDK_DIR}/src/arch/macOS")
    # Required, not optional. sdk/src/hal/ guards pthread_mutex_timedlock and
    # pthread_condattr_setclock behind #ifdef _MACOS because macOS has
    # neither; without the define the SDK does not compile here at all.
    # Upstream sets it the same way (mak_def.inc: CDEFS += -D_MACOS).
    set(SDK_PLATFORM_DEFINES _MACOS)
else()
    message(FATAL_ERROR
        "No RPLIDAR SDK platform layer for ${CMAKE_SYSTEM_NAME}. "
        "The SDK ships linux, macOS and win32; this project supports the "
        "first two.")
endif()

list(APPEND SDK_SOURCES
    "${SDK_ARCH_DIR}/net_serial.cpp"
    "${SDK_ARCH_DIR}/net_socket.cpp"
    "${SDK_ARCH_DIR}/timer.cpp")

add_library(rplidar_sdk STATIC ${SDK_SOURCES})

# Include paths taken from sdk/Makefile. The dataunpacker sources include
# their neighbours unqualified, so those two directories are not optional.
target_include_directories(rplidar_sdk SYSTEM PUBLIC
    "${SDK_DIR}/include")
target_include_directories(rplidar_sdk SYSTEM PRIVATE
    "${SDK_DIR}/src"
    "${SDK_DIR}/src/dataunpacker"
    "${SDK_DIR}/src/dataunpacker/unpacker")

# Upstream builds at -std=c++11 (mak_def.inc). Hold it there rather than
# dragging third-party code forward into C++20, where removed features and
# tightened rules turn into other people's compile errors.
set_target_properties(rplidar_sdk PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS ON          # the SDK uses GNU extensions
    POSITION_INDEPENDENT_CODE ON)

# Deliberately NOT linked against makerbot_warnings. Third-party warnings are
# noise we cannot action, and the temptation to silence them by lowering our
# own warning level is exactly what SYSTEM includes and this omission avoid.
target_compile_options(rplidar_sdk PRIVATE -w)
target_compile_definitions(rplidar_sdk PRIVATE ${SDK_PLATFORM_DEFINES})

find_package(Threads REQUIRED)
target_link_libraries(rplidar_sdk PUBLIC Threads::Threads)

# Record what we pinned, visible in the configure output.
find_package(Git QUIET)
if(GIT_FOUND)
    execute_process(
        COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rplidar_sdk"
        OUTPUT_VARIABLE SDK_COMMIT
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET)
endif()
message(STATUS "  rplidar_sdk  ${SDK_COMMIT} (${CMAKE_SYSTEM_NAME} platform layer)")
message(STATUS "  rplidar_sdk  1 local patch: unbindAndClose() delete/new mismatch")
