#!/usr/bin/env bash

set -Eeuo pipefail

trap 'echo "ERROR: Provisioning failed at line ${LINENO}."' ERR

if [[ "${EUID}" -ne 0 ]]; then
    echo "ERROR: This script must run as root."
    exit 1
fi

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <target-user>"
    exit 1
fi

TARGET_USER="$1"

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
    echo "ERROR: Target user '${TARGET_USER}' does not exist."
    exit 1
fi

# base.sh is staged on the target together with the rest of target/, because
# provisioning installs data files (udev rules) that live beside it in the
# repository rather than being embedded in this script.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TARGET_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
UDEV_RULES_DIR="${TARGET_ROOT}/udev"

# Heron network model:
#
#   eth0  = local development / SSH network
#   wlan0 = Internet/default route
#
# These can be overridden if a future target uses different interface names.
ETH_DEV="${HERON_ETH_DEV:-eth0}"
WIFI_DEV="${HERON_WIFI_DEV:-wlan0}"

echo "========================================"
echo " Heron Raspberry Pi Provisioning"
echo "========================================"

echo
echo "Host:"
hostname

echo
echo "OS:"
cat /etc/os-release

echo
echo "Architecture:"
uname -m

echo
echo "Kernel:"
uname -r

echo
echo "Target user:"
echo "${TARGET_USER}"

echo
echo "Current network interfaces:"
ip -br addr

echo
echo "Current routing table:"
ip route

#
# Network configuration
#

echo
echo "Configuring Heron network policy..."

if ! command -v nmcli >/dev/null 2>&1; then
    echo "ERROR: NetworkManager/nmcli is required but was not found."
    exit 1
fi

# If Wi-Fi already has a valid default route, remove the Ethernet
# default route from the current runtime configuration.
#
# The directly-connected Ethernet subnet route remains in place,
# so an SSH connection over eth0 is not disturbed.
if ip route show default dev "${WIFI_DEV}" | grep -q '^default '; then
    if ip route show default dev "${ETH_DEV}" | grep -q '^default '; then
        echo "Removing runtime default route from ${ETH_DEV}..."
        while ip route show default dev "${ETH_DEV}" | grep -q '^default '; do
            ip route del default dev "${ETH_DEV}"
        done
    fi
else
    echo "ERROR: ${WIFI_DEV} does not currently have a default route."
    echo
    echo "Heron expects:"
    echo "  ${ETH_DEV}  -> local development/SSH network"
    echo "  ${WIFI_DEV} -> Internet/default route"
    echo
    echo "Connect Wi-Fi before provisioning."
    exit 1
fi

echo
echo "Routing table after runtime adjustment:"
ip route

#
# Verify network connectivity before changing persistent configuration
# or attempting apt operations.
#

echo
echo "Checking Internet connectivity..."

if ! ping -c 1 -W 3 1.1.1.1 >/dev/null 2>&1; then
    echo "ERROR: Internet connectivity check failed."
    echo "Unable to reach 1.1.1.1 through ${WIFI_DEV}."
    exit 1
fi

echo "Internet connectivity OK."

echo
echo "Checking DNS..."

if ! getent ahostsv4 deb.debian.org >/dev/null 2>&1; then
    echo "ERROR: DNS resolution failed."
    echo "Unable to resolve deb.debian.org."
    exit 1
fi

echo "DNS resolution OK."

#
# Persist the network configuration.
#

ETH_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "${ETH_DEV}" \
    2>/dev/null || true)"

if [[ -n "${ETH_CONNECTION}" && "${ETH_CONNECTION}" != "--" ]]; then
    echo
    echo "Configuring Ethernet profile '${ETH_CONNECTION}' as local-only..."

    nmcli connection modify "${ETH_CONNECTION}" \
        ipv4.never-default yes \
        ipv6.never-default yes \
        ipv4.ignore-auto-dns yes \
        ipv6.ignore-auto-dns yes
else
    echo
    echo "WARNING: No active NetworkManager profile found for ${ETH_DEV}."
fi

WIFI_CONNECTION="$(nmcli -g GENERAL.CONNECTION device show "${WIFI_DEV}" \
    2>/dev/null || true)"

if [[ -n "${WIFI_CONNECTION}" && "${WIFI_CONNECTION}" != "--" ]]; then
    echo
    echo "Ensuring Wi-Fi profile '${WIFI_CONNECTION}' auto-connects..."

    nmcli connection modify "${WIFI_CONNECTION}" \
        connection.autoconnect yes
else
    echo
    echo "WARNING: No active NetworkManager profile found for ${WIFI_DEV}."
fi

#
# Package installation
#

echo
echo "Updating package database..."

# Unlike a plain apt-get update, this makes repository download failures
# fatal instead of silently continuing with stale package indexes.
apt-get update \
    -o APT::Update::Error-Mode=any

echo
echo "Installing Heron baseline packages..."

# python3-serial comes from apt rather than pip because Debian marks the
# system interpreter EXTERNALLY-MANAGED (PEP 668), which blocks pip installs
# into it by design. Trixie ships pyserial 3.5, which is the current upstream
# release, so there is no version penalty and no venv is needed on the target.
#
# If a Python dependency is ever needed that Debian does not package, create a
# venv that still inherits these apt packages rather than replacing them:
#
#   python3 -m venv --system-site-packages ~/heron/.venv
apt-get install -y \
    python3 \
    python3-venv \
    python3-pip \
    python3-serial \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    rsync \
    git

#
# Hostname
#

TARGET_HOSTNAME="${HERON_HOSTNAME:-heron}"

echo
echo "Setting hostname to '${TARGET_HOSTNAME}'..."

# The image is set up on first boot by cloud-init, from the user-data that
# Raspberry Pi Imager writes to the boot partition. On every later boot
# cloud-init still regenerates /etc/hosts (manage_etc_hosts), but from its
# cached copy of the first-boot user-data, not the boot partition: editing
# that file was measured to have no effect. So a rename would be undone in
# /etc/hosts on each boot. Its first-boot work is done by the time this
# script runs, and the network profiles it created are persistent
# NetworkManager keyfiles, so it is disabled and the hostname is owned here.
if [[ -d /etc/cloud ]]; then
    touch /etc/cloud/cloud-init.disabled
fi

# /etc/hosts is updated before the hostname itself, so that sudo and other
# tools never see a hostname that does not resolve.
if grep -q '^127\.0\.1\.1[[:space:]]' /etc/hosts; then
    sed -i "s/^127\.0\.1\.1[[:space:]].*/127.0.1.1 ${TARGET_HOSTNAME} ${TARGET_HOSTNAME}/" \
        /etc/hosts
else
    echo "127.0.1.1 ${TARGET_HOSTNAME} ${TARGET_HOSTNAME}" >> /etc/hosts
fi

hostnamectl set-hostname "${TARGET_HOSTNAME}"

#
# Device access and stable device names
#

echo
echo "Configuring serial-device access for ${TARGET_USER}..."

usermod -aG dialout "${TARGET_USER}"

echo
echo "Installing udev rules..."

if [[ ! -d "${UDEV_RULES_DIR}" ]]; then
    echo "ERROR: udev rules directory not found:"
    echo "  ${UDEV_RULES_DIR}"
    echo
    echo "Provisioning must stage the whole target/ directory, not just this"
    echo "script. Use scripts/provision_pi.sh rather than copying base.sh."
    exit 1
fi

# install overwrites in place, so re-provisioning is a no-op when the rules
# have not changed.
install -m 0644 "${UDEV_RULES_DIR}"/*.rules /etc/udev/rules.d/

udevadm control --reload-rules

# Re-trigger tty devices so the symlinks appear without unplugging anything.
udevadm trigger --subsystem-match=tty --action=add

#
# GPIO UART for the remote control receiver
#

echo
echo "Configuring the GPIO UART (/dev/serial0) for the iBUS receiver..."

BOOT_DIR="/boot/firmware"
BOOT_CONFIG="${BOOT_DIR}/config.txt"
BOOT_CMDLINE="${BOOT_DIR}/cmdline.txt"
UART_MARKER="# heron: GPIO UART for the iBUS receiver"
REBOOT_NEEDED=0

# On a stock image the GPIO UART is unusable: enable_uart is unset, so the
# mini UART is off (8250.nr_uarts=0), and the Pi 3's PL011 is claimed by the
# Bluetooth chip through serdev, which exposes no tty at all. /dev/serial0
# did not exist (measured 2026-10-10). Heron has no use for Bluetooth, so
# disable-bt gives the PL011 to GPIO14/15 as ttyAMA0, and serial0 points at
# it. That is the better UART: miniuart-bt would keep Bluetooth, but the mini
# UART's baud rate follows the core clock.
#
# [all] is repeated because the stock file ends in a model filter section,
# and anything appended after one applies to that model only.
if ! grep -qF "${UART_MARKER}" "${BOOT_CONFIG}"; then
    {
        echo
        echo "${UART_MARKER}"
        echo "[all]"
        echo "enable_uart=1"
        echo "dtoverlay=disable-bt"
    } >> "${BOOT_CONFIG}"
    REBOOT_NEEDED=1
fi

# The image puts the kernel console on serial0, which once enabled would put
# boot messages on the receiver's line and a login prompt behind it. Only the
# serial consoles are removed; console=tty1 stays.
if grep -qE 'console=(serial0|ttyAMA0|ttyS0),[0-9]+' "${BOOT_CMDLINE}"; then
    sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' "${BOOT_CMDLINE}"
    REBOOT_NEEDED=1
fi

# With the controller's UART taken away, the Bluetooth service has nothing
# to drive.
if systemctl is-enabled --quiet bluetooth.service 2>/dev/null; then
    systemctl disable --now bluetooth.service
fi

echo
echo "Final routing table:"
ip route

echo
echo "Target user groups:"
id "${TARGET_USER}"

echo
echo "Serial devices by id:"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  (none attached)"

echo
echo "Heron device symlinks:"
ls -l /dev/rplidar 2>/dev/null \
    || echo "  /dev/rplidar not present (LiDAR not attached?)"

echo
echo "Power/throttling status:"

# The RPLIDAR motor is a meaningful load on a Pi 3, and under-voltage corrupts
# SD cards long before it produces an obvious symptom. Surface it here rather
# than letting it be rediscovered later as flaky scan data.
THROTTLED="$(vcgencmd get_throttled 2>/dev/null || true)"
echo "  ${THROTTLED:-unavailable}"

if [[ -n "${THROTTLED}" && "${THROTTLED}" != "throttled=0x0" ]]; then
    THROTTLE_BITS="$(( ${THROTTLED#throttled=} ))"

    echo
    echo "WARNING: power/throttling flags are set."

    # Only the bits actually set are reported. The low bits describe the
    # current state; bits 16-19 are sticky and only say it happened at some
    # point since boot, which is a much weaker claim.
    #
    # An if/fi is used rather than `(( bit )) && echo`, because a false
    # arithmetic test leaves a non-zero status that would abort the script
    # under `set -e` if it ever ended up as the last statement in a block.
    report_bit() {
        if (( THROTTLE_BITS & (1 << $1) )); then
            echo "  $2"
        fi
    }

    report_bit 0  "under-voltage detected RIGHT NOW"
    report_bit 1  "ARM frequency capped RIGHT NOW"
    report_bit 2  "currently throttled"
    report_bit 3  "soft temperature limit active"
    report_bit 16 "under-voltage has occurred since boot"
    report_bit 17 "ARM frequency capping has occurred since boot"
    report_bit 18 "throttling has occurred since boot"
    report_bit 19 "soft temperature limit has occurred since boot"

    echo
    echo "The RPLIDAR motor draws from the same 5 V rail. Use a supply rated"
    echo "for at least 2.5 A, and consider a powered USB hub for the LiDAR."
fi

echo
echo "========================================"
echo " Provisioning complete"
echo "========================================"
echo
echo "NOTE:"
echo "Group membership changes take effect on the user's next login."

if [[ "${REBOOT_NEEDED}" -eq 1 ]]; then
    echo
    echo "REBOOT REQUIRED: the boot configuration changed (GPIO UART)."
    echo "  sudo reboot"
fi

