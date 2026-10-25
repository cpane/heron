#!/usr/bin/env bash

# Build the Arduino motor firmware, inside a container.
#
# The firmware counterpart of build_cpp.sh. The AVR compiler and PlatformIO
# live in the image (docker/Dockerfile.firmware), so the host needs only
# Docker. Versions are pinned in the image and in firmware/motor/platformio.ini.
#
# Unlike build_cpp.sh, the container runs at the host's own architecture:
# the output is AVR code either way, and PlatformIO ships its compiler for
# both arm64 and x86_64 Linux, so there is nothing to emulate.
#
# PlatformIO downloads the compiler and the Arduino core on first use, about
# 290 MB. They go to build-firmware/platformio/ in the repository (git-ignored),
# so they survive between runs; the firmware lands in build-firmware/motor/.
#
# Runs on macOS and Linux hosts. Windows is not supported and is not a goal.
#
#   scripts/build_firmware.sh            build every platform's firmware
#   scripts/build_firmware.sh --clean    wipe the firmware build first
#   scripts/build_firmware.sh --shell    drop into the container instead
#
# Then: scripts/flash_firmware.sh <user>@<pi>

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

IMAGE_TAG="heron-firmware:trixie"
# Must match build_dir in platformio.ini.
BUILD_DIR="build-firmware"
PROJECT_DIR="firmware/motor"

CLEAN=0
SHELL_MODE=0
for arg in "$@"; do
    case "${arg}" in
        --clean) CLEAN=1 ;;
        --shell) SHELL_MODE=1 ;;
        *)
            echo "ERROR: unknown argument: ${arg}"
            echo "Usage: $0 [--clean] [--shell]"
            exit 1
            ;;
    esac
done

echo "===================================================="
echo " Heron firmware build (target: Arduino Uno, AVR)"
echo "===================================================="
echo

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker is not installed."
    echo "  macOS: brew install --cask docker    then start Docker Desktop"
    echo "  Linux: install docker.io or podman and alias docker=podman"
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "ERROR: the docker daemon is not running."
    echo "  macOS: start Docker Desktop (or: colima start)"
    echo "  Linux: sudo systemctl start docker"
    exit 1
fi

echo "Host:        $(uname -s) $(uname -m)"
echo "Repository:  ${REPO_ROOT}"
echo "Build dir:   ${REPO_ROOT}/${BUILD_DIR}"
echo

echo "Building the image..."
docker build \
    -t "${IMAGE_TAG}" \
    -f "${REPO_ROOT}/docker/Dockerfile.firmware" \
    "${REPO_ROOT}/docker"

if [[ "${CLEAN}" -eq 1 ]]; then
    echo
    echo "Removing ${BUILD_DIR}/motor..."
    # The firmware only: the downloaded toolchain in platformio/ stays.
    rm -rf "${REPO_ROOT:?}/${BUILD_DIR}/motor"
fi

# --user keeps generated files owned by the invoking user rather than root.
# HOME is then set to somewhere writable, since that user has no home in the
# image and PlatformIO writes a little there.
DOCKER_USER=""
if [[ "$(uname -s)" == "Linux" ]]; then
    DOCKER_USER="--user $(id -u):$(id -g) -e HOME=/tmp"
fi

# PLATFORMIO_CORE_DIR: where PlatformIO keeps the compiler and the Arduino
# core. In the repository, not in the image, so they are fetched once.
DOCKER_ARGS="--rm ${DOCKER_USER} -v ${REPO_ROOT}:/src"
DOCKER_ARGS="${DOCKER_ARGS} -e PLATFORMIO_CORE_DIR=/src/${BUILD_DIR}/platformio"
DOCKER_ARGS="${DOCKER_ARGS} -w /src/${PROJECT_DIR}"

if [[ "${SHELL_MODE}" -eq 1 ]]; then
    echo
    echo "Entering the firmware container. The repository is at /src."
    # shellcheck disable=SC2086
    exec docker run -it ${DOCKER_ARGS} "${IMAGE_TAG}" /bin/bash
fi

echo
echo "Compiling..."
# shellcheck disable=SC2086
docker run ${DOCKER_ARGS} "${IMAGE_TAG}" pio run

echo
echo "===================================================="
echo " Build complete"
echo "===================================================="
ls -la "${REPO_ROOT}/${BUILD_DIR}/motor"/*/firmware.hex
echo
echo "Flash it through the Pi:"
echo "  scripts/flash_firmware.sh <user>@<pi>"
