#!/usr/bin/env bash

# Start heron_robot at every boot, or stop doing so.
#
# For driving the robot with the remote and no laptop, in all three SwC
# modes: power on, keep it still while it starts (the OTOS calibrates),
# arm the remote. The service is target/systemd/heron-robot@.service, run
# as the deploy user from the binaries and rig profile deploy_cpp.sh and
# set_rig.sh put in ~/heron.
#
#   scripts/robot_autostart.sh <user>@<pi> on       # install, enable, start now
#   scripts/robot_autostart.sh <user>@<pi> off      # stop, and not at boot
#   scripts/robot_autostart.sh <user>@<pi> status   # state and recent output
#
# heron_robot and heron_teleop both own /dev/arduino and /dev/serial0, so
# on turns heron_teleop's service off (scripts/teleop_autostart.sh) and the
# unit conflicts with it. off leaves both off: turn heron_teleop back on
# with teleop_autostart.sh if wanted. Turn it off before bench work or a
# heron_robot run from a terminal. on and off need sudo on the Pi, so they
# prompt for its password; status does not.

set -Eeuo pipefail

usage() {
    echo "Usage: $0 <user@host> on|off|status"
    echo "Example: $0 <user>@<pi> on"
    exit 1
}

if [[ $# -ne 2 ]]; then
    usage
fi

TARGET="$1"
ACTION="$2"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    usage
fi

case "${ACTION}" in
    on|off|status) ;;
    *)
        echo "ERROR: unknown action '${ACTION}'."
        usage
        ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

TARGET_USER="${TARGET%@*}"
UNIT_TEMPLATE="heron-robot@.service"
UNIT="heron-robot@${TARGET_USER}.service"
TELEOP_UNIT="heron-teleop@${TARGET_USER}.service"
LOCAL_UNIT="${REPO_ROOT}/target/systemd/${UNIT_TEMPLATE}"
REMOTE_STAGE="/tmp/${UNIT_TEMPLATE}"

echo "========================================"
echo " Heron robot at boot: ${ACTION}"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Service:     ${UNIT}"
echo

if [[ "${ACTION}" == "status" ]]; then
    # systemctl exits non-zero for a stopped unit; that is an answer here,
    # not an error.
    ssh "${TARGET}" "systemctl is-enabled '${UNIT}' 2>/dev/null | sed 's/^/At boot:     /' || echo 'At boot:     not installed'; \
        systemctl is-active '${UNIT}' | sed 's/^/Now:         /' || true; \
        systemctl is-active '${TELEOP_UNIT}' | sed 's/^/Teleop now:  /' || true; \
        echo; echo 'Recent output:'; \
        journalctl -u '${UNIT}' -n 15 --no-pager -o cat 2>/dev/null || true"
    exit 0
fi

if [[ "${ACTION}" == "on" ]]; then
    # Checked before sudo, so a Pi that cannot run it is refused with the
    # reason rather than left with a service restarting every 3 seconds.
    echo "Checking the Pi has heron_robot and a rig..."
    if ! ssh "${TARGET}" "test -x heron/bin/heron_robot"; then
        echo "ERROR: ~/heron/bin/heron_robot is not on the Pi. Deploy first:"
        echo "  scripts/build_cpp.sh && scripts/deploy_cpp.sh ${TARGET}"
        exit 1
    fi
    if ! ssh "${TARGET}" "test -s heron/rig"; then
        echo "ERROR: the Pi has no rig set (~/heron/rig). Set it first:"
        echo "  scripts/set_rig.sh ${TARGET} <rig>"
        exit 1
    fi
    echo "OK: rig '$(ssh "${TARGET}" "cat heron/rig")'."
    echo

    # heron_teleop off first (it would hold the Arduino), then enable and
    # restart rather than enable --now: restart starts it if it is stopped
    # and picks up a newly deployed binary if it is running.
    scp -q "${LOCAL_UNIT}" "${TARGET}:${REMOTE_STAGE}"
    # -t so sudo can prompt for the password.
    ssh -t "${TARGET}" "sudo install -m 0644 '${REMOTE_STAGE}' '/etc/systemd/system/${UNIT_TEMPLATE}' \
        && rm -f '${REMOTE_STAGE}' \
        && sudo systemctl daemon-reload \
        && { sudo systemctl disable --now '${TELEOP_UNIT}' 2>/dev/null || true; } \
        && sudo systemctl enable '${UNIT}' \
        && sudo systemctl restart '${UNIT}'"
    echo
    echo "On: heron_robot starts at every boot, and has started now;"
    echo "heron_teleop is off. Keep the robot still for a few seconds as it starts."
else
    ssh -t "${TARGET}" "sudo systemctl disable --now '${UNIT}'"
    echo
    echo "Off: stopped, and not started at boot. heron_teleop is not turned"
    echo "back on; for that: scripts/teleop_autostart.sh ${TARGET} on"
fi

echo
echo "Watch it:"
echo "  ssh ${TARGET} journalctl -u ${UNIT} -f"
echo "  $0 ${TARGET} status"
