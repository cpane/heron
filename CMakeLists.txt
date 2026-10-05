# Heron -- C++ build.
#
# 3.20 rather than something newer: the Pi runs CMake 3.31.6 and this host
# runs 4.1.1, and CMake 4 removed compatibility with `cmake_minimum_required`
# below 3.5. 3.20 is comfortably supported by both and by any Debian trixie
# container, so the same tree configures everywhere without version pinning
# games.
cmake_minimum_required(VERSION 3.20)

project(heron
    VERSION 0.1.0
    DESCRIPTION "Heron robot software"
    LANGUAGES CXX)

# -- which machine is this build for? ---------------------------------------
#
# Two separate things, and conflating them was a bug:
#
#   HERON_DEPLOYABLE  what the toolchain ACTUALLY produces. Derived from
#                        the compiler, so it cannot be wrong.
#   HERON_TARGET      what the caller SAYS they want. A guard only.
#
# The banner reports the first. An earlier version reported the second, which
# meant a CLion profile using the Docker toolchain but not setting
# HERON_TARGET built a genuine aarch64 Pi binary and labelled it
# "NOT deployable" -- the exact confusion the banner exists to prevent, just
# inverted. Anything the IDE can forget to set is not a safe source of truth.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux"
   AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(HERON_DEPLOYABLE TRUE)
else()
    set(HERON_DEPLOYABLE FALSE)
endif()

set(HERON_TARGET "host" CACHE STRING "Build for: host | pi")
set_property(CACHE HERON_TARGET PROPERTY STRINGS host pi)

if(HERON_TARGET STREQUAL "pi" AND NOT HERON_DEPLOYABLE)
    # Refuse rather than silently produce something undeployable: the "pi"
    # preset on macOS would otherwise yield a Mach-O binary in build-pi/bin
    # under a reassuring name, and the mistake would surface only as
    # "cannot execute binary file" on the target.
    message(FATAL_ERROR
        "HERON_TARGET=pi needs an aarch64 Linux toolchain, but this is "
        "${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}.\n"
        "\n"
        "  From a terminal:  scripts/build_cpp.sh\n"
        "  In CLion:         attach a Docker toolchain using the image\n"
        "                    heron-build:trixie to this profile\n"
        "\n"
        "To build for this machine instead, use the 'host' preset.")
elseif(NOT HERON_TARGET STREQUAL "pi" AND NOT HERON_TARGET STREQUAL "host")
    message(FATAL_ERROR "HERON_TARGET must be 'host' or 'pi', got '${HERON_TARGET}'")
endif()

# -- toolchain defaults ------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generated for editors and clang-tidy. Costs nothing, saves an argument later.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# Everything lands in one place so deploy_cpp.sh has a single directory to
# mirror to the target.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# -- our warning policy ------------------------------------------------------
#
# Applied to our targets only, never to the vendor SDK. An interface library
# rather than a global flag, precisely so third_party/ can opt out.
add_library(heron_warnings INTERFACE)
target_compile_options(heron_warnings INTERFACE
    -Wall -Wextra -Wpedantic
    -Wshadow
    -Wconversion          # the Q14/Q6 and float/double seams live here
    -Wsign-conversion
    -Wnon-virtual-dtor
    -Wold-style-cast)

enable_testing()

add_subdirectory(third_party)
add_subdirectory(cpp)

if(HERON_DEPLOYABLE)
    set(HERON_TARGET_BANNER "pi-aarch64  --  DEPLOYABLE to the Raspberry Pi")
else()
    set(HERON_TARGET_BANNER
        "host-native  --  runs on THIS machine, NOT deployable to the Pi")
endif()

# Says so out loud when the two disagree. Harmless, but it means a profile
# that was set up by hand rather than from a preset still tells the truth.
if(HERON_DEPLOYABLE AND NOT HERON_TARGET STREQUAL "pi")
    message(STATUS "")
    message(STATUS "  NOTE: this toolchain produces deployable aarch64 binaries,")
    message(STATUS "        but HERON_TARGET is '${HERON_TARGET}'. Harmless")
    message(STATUS "        -- the banner below reports what is actually built --")
    message(STATUS "        but setting HERON_TARGET=pi (or using the 'pi'")
    message(STATUS "        preset) keeps the two in agreement.")
endif()

message(STATUS "")
message(STATUS "  ============================================================")
message(STATUS "   heron ${PROJECT_VERSION}")
message(STATUS "   TARGET:     ${HERON_TARGET_BANNER}")
message(STATUS "  ============================================================")
message(STATUS "   build type  ${CMAKE_BUILD_TYPE}")
message(STATUS "   system      ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "   compiler    ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "   output      ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "")
