#!/usr/bin/env bash

# Flash the motor firmware onto the Arduino, through the Pi.
#
# The firmware counterpart of deploy_cpp.sh. The Arduino is plugged into the
# Pi (USB port 1-1.2, named /dev/arduino by the udev rule), on the bench as on
# the robot, so flashing goes the same way as deploying: copy the build
# output to the Pi over SSH, then run the tool there. The tool is avrdude,
# installed by provisioning; the Pi never builds anything.
#
#   scripts/build_firmware.sh                       first
#   scripts/flash_firmware.sh <user>@<pi>               the uno-l293d firmware
#   scripts/flash_firmware.sh <user>@<pi> uno-l293d     the same, by name
#
# How avrdude reaches the chip: the Uno's bootloader (Optiboot) listens on
# the serial port for a moment after each reset. avrdude opens the port,
# which resets the board, and talks to the bootloader at 115200 baud ("-c
# arduino"). Nothing else may have /dev/arduino open meanwhile: stop any
# motor_link or robot program first.

set -Eeuo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 <user@host> [platform]"
    echo "Example: $0 <user>@<pi>"
    echo "Platforms: the environments in firmware/motor/platformio.ini (default uno-l293d)"
    exit 1
fi

TARGET="$1"
PLATFORM="${2:-uno-l293d}"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

LOCAL_HEX="${REPO_ROOT}/build-firmware/motor/${PLATFORM}/firmware.hex"
REMOTE_DIR="heron/firmware"
REMOTE_HEX="${REMOTE_DIR}/motor-${PLATFORM}.hex"
DEVICE="/dev/arduino"
# The ATmega328P, through its serial bootloader.
AVRDUDE_ARGS="-p atmega328p -c arduino -b 115200"

if [[ ! -f "${LOCAL_HEX}" ]]; then
    echo "ERROR: no firmware found at:"
    echo "  ${LOCAL_HEX}"
    echo "Build first:"
    echo "  scripts/build_firmware.sh"
    exit 1
fi

echo "========================================"
echo " Heron firmware flash"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Platform:    ${PLATFORM}"
echo "Firmware:    ${LOCAL_HEX}"
echo "Device:      ${DEVICE} on the Pi"
echo

echo "Checking SSH connectivity..."
if ssh -o BatchMode=yes -o ConnectTimeout=5 "${TARGET}" true 2>/dev/null; then
    echo "Key-based SSH OK."
else
    echo
    echo "NOTE: Key-based SSH is not configured for ${TARGET}."
    echo "Every connection below will prompt for a password."
    echo "To fix this once:"
    echo "  ssh-copy-id ${TARGET}"
    echo
fi

# Checked before copying anything, so a missing tool or board is reported
# plainly rather than as an avrdude error.
echo
echo "Checking the Pi..."
if ! ssh "${TARGET}" "command -v avrdude >/dev/null"; then
    echo "ERROR: avrdude is not installed on the Pi."
    echo "  Re-run provisioning, which installs it:"
    echo "  scripts/provision_pi.sh ${TARGET}"
    exit 1
fi
if ! ssh "${TARGET}" "test -e ${DEVICE}"; then
    echo "ERROR: ${DEVICE} does not exist on the Pi."
    echo "  Is the Arduino plugged into USB port 1-1.2? Only that port gets the name"
    echo "  (target/udev/99-heron-arduino.rules)."
    exit 1
fi

echo
echo "Copying the firmware..."
ssh "${TARGET}" "mkdir -p '${REMOTE_DIR}'"
# -rlptvz rather than -a: macOS ships openrsync (see deploy_cpp.sh).
rsync -rlptvz -e ssh "${LOCAL_HEX}" "${TARGET}:${REMOTE_HEX}"

echo
echo "Flashing..."
# -D: do not erase first. The bootloader cannot erase the whole chip, and
# overwrites each page as it writes it anyway; this is what the Arduino IDE
# does. ":i" marks the file as Intel hex. avrdude verifies after writing.
ssh "${TARGET}" "avrdude ${AVRDUDE_ARGS} -P ${DEVICE} -D -U flash:w:\${HOME}/${REMOTE_HEX}:i"

echo
echo "========================================"
echo " Flashed"
echo "========================================"
echo
echo "Check it is running:"
echo "  scripts/deploy_cpp.sh ${TARGET} motor_link info"
