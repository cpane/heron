#!/usr/bin/env bash

# Build the C++ tree for the Pi, inside a Debian trixie arm64 container.
#
# The container exists so the build environment matches the target exactly:
# same distro, same glibc, same gcc major. Building on the host directly would
# produce a macOS binary; cross-compiling would mean sourcing a toolchain whose
# glibc must be no newer than the Pi's.
#
# Runs on macOS and Linux hosts. Windows is not supported and is not a goal.
#
#   scripts/build_cpp.sh                 configure + build
#   scripts/build_cpp.sh --clean         wipe the build directory first
#   scripts/build_cpp.sh --shell         drop into the container instead

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

IMAGE_TAG="heron-build:trixie"
# Must match the "pi" preset's binaryDir in CMakePresets.json. The preset is
# the single definition of how a deployable build is configured; this script
# just arranges for it to run inside an aarch64 Linux toolchain.
BUILD_DIR="build-pi"
# The Pi is aarch64, so the container must be too, whatever the host is.
TARGET_PLATFORM="linux/arm64"

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
echo " Heron C++ build (target: Raspberry Pi, aarch64)"
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

# The submodule is the one prerequisite a fresh clone will not have, and the
# CMake error for it fires deep inside the container where it is easy to
# misread. Check here, where the fix is obvious.
if [[ ! -f "${REPO_ROOT}/third_party/rplidar_sdk/sdk/include/sl_lidar_driver.h" ]]; then
    echo "ERROR: the RPLIDAR SDK submodule is not checked out."
    echo "  git submodule update --init --recursive"
    exit 1
fi

HOST_ARCH="$(uname -m)"
echo "Host:        $(uname -s) ${HOST_ARCH}"
echo "Container:   ${TARGET_PLATFORM} (debian trixie)"
echo "Repository:  ${REPO_ROOT}"
echo "Build dir:   ${REPO_ROOT}/${BUILD_DIR}"
echo

# arm64 hosts run the container natively; anything else goes through emulation,
# which works but is slow enough to be worth warning about rather than leaving
# someone to wonder why a small C++ tree takes minutes.
case "${HOST_ARCH}" in
    arm64|aarch64)
        echo "Native arm64 host: the container runs without emulation."
        ;;
    *)
        echo "NOTE: ${HOST_ARCH} host building for arm64, so the container is"
        echo "      emulated (qemu). Correct, but expect it to be slow."
        echo "      Docker Desktop enables this by default; on Linux you may"
        echo "      need: docker run --privileged --rm tonistiigi/binfmt --install arm64"
        ;;
esac
echo

echo "Building the image..."
docker build \
    --platform "${TARGET_PLATFORM}" \
    -t "${IMAGE_TAG}" \
    -f "${REPO_ROOT}/docker/Dockerfile.build" \
    "${REPO_ROOT}/docker"

if [[ "${CLEAN}" -eq 1 ]]; then
    echo
    echo "Removing ${BUILD_DIR}..."
    rm -rf "${REPO_ROOT:?}/${BUILD_DIR}"
fi

# --user keeps generated files owned by the invoking user rather than root.
# Without it, a later `rm -rf build-pi` from the host needs sudo on Linux.
DOCKER_USER=""
if [[ "$(uname -s)" == "Linux" ]]; then
    DOCKER_USER="--user $(id -u):$(id -g)"
fi

if [[ "${SHELL_MODE}" -eq 1 ]]; then
    echo
    echo "Entering the build container. The repository is at /src."
    # shellcheck disable=SC2086
    exec docker run --rm -it \
        --platform "${TARGET_PLATFORM}" \
        ${DOCKER_USER} \
        -v "${REPO_ROOT}:/src" \
        -w /src \
        "${IMAGE_TAG}" \
        /bin/bash
fi

echo
echo "Configuring..."
# shellcheck disable=SC2086
docker run --rm \
    --platform "${TARGET_PLATFORM}" \
    ${DOCKER_USER} \
    -v "${REPO_ROOT}:/src" \
    -w /src \
    "${IMAGE_TAG}" \
    cmake --preset pi

echo
echo "Compiling..."
# shellcheck disable=SC2086
docker run --rm \
    --platform "${TARGET_PLATFORM}" \
    ${DOCKER_USER} \
    -v "${REPO_ROOT}:/src" \
    -w /src \
    "${IMAGE_TAG}" \
    cmake --build --preset pi

echo
echo "===================================================="
echo " Build complete"
echo "===================================================="
ls -la "${REPO_ROOT}/${BUILD_DIR}/bin"
echo
echo "Deploy and run on the Pi:"
echo "  scripts/deploy_cpp.sh <user>@<pi> lidar_info"
