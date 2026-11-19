#!/usr/bin/env bash

# Tell a Pi which floor its robot is on.
#
# A rig's profile names the floor it usually runs on ([rig] surface in
# profiles/<rig>.conf); each floor's numbers are a surface, most of them
# in profiles/surfaces/<drive>/<name>.conf (docs/rig-profiles.md). Taken
# to another floor, the Maker Faire's rubber and then home to the hardwood
# again, the robot is told so here: the name goes to ~/heron/surface on the
# Pi, and heron_robot uses it until told otherwise, the boot service
# included. No change to the profile, no deploy. heron_robot --surface
# still wins for one run.
#
#   scripts/set_surface.sh <user>@<pi> rubber-1    # on the rubber from now on
#   scripts/set_surface.sh <user>@<pi> --default   # back to the profile's floor
#   scripts/set_surface.sh <user>@<pi>             # which floor it is on now
#
# The name must be a surface this Pi's rig knows, deployed on the Pi, so a
# Pi can never be told a floor it has no numbers for. Takes effect the next
# time heron_robot starts. No sudo: the file is the deploy user's.

set -Eeuo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 <user@host> [<surface> | --default]"
    echo "Example: $0 <user>@<pi> rubber-1"
    exit 1
fi

TARGET="$1"
SURFACE="${2:-}"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi> rubber-1"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

REMOTE_RIG_FILE="heron/rig"
REMOTE_SURFACE_FILE="heron/surface"

# The same rule heron_profile applies (isValidRigName), checked here so a
# bad name is refused before anything is written.
if [[ -n "${SURFACE}" && "${SURFACE}" != "--default" && ! "${SURFACE}" =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]]; then
    echo "ERROR: '${SURFACE}' is not a valid surface name: up to 32 of a-z, 0-9 and '-'."
    exit 1
fi

# Not answering at all is an error; answering without a key only means
# password prompts below.
if ! PROBE="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "${TARGET}" true 2>&1)"; then
    if [[ "${PROBE}" != *"Permission denied"* ]]; then
        echo "ERROR: ${TARGET} is not answering: ${PROBE}"
        echo "Is the Pi on, and on this network?"
        exit 1
    fi
    echo "NOTE: Key-based SSH is not configured for ${TARGET}."
    echo "Every connection below will prompt for a password. To fix this once:"
    echo "  ssh-copy-id ${TARGET}"
    echo
fi

# The rig, its profile and drive: which surfaces this Pi can be told.
RIG="$(ssh "${TARGET}" "cat '${REMOTE_RIG_FILE}' 2>/dev/null || true")"
if [[ -z "${RIG}" ]]; then
    echo "ERROR: the Pi has no rig set, so its floors are not known."
    echo "Set it first: scripts/set_rig.sh ${TARGET} <rig>"
    exit 1
fi
PROFILE="${REPO_ROOT}/profiles/${RIG}.conf"
if [[ ! -f "${PROFILE}" ]]; then
    echo "ERROR: the Pi is rig '${RIG}', which has no profile here: ${PROFILE}"
    exit 1
fi
DRIVE="$(sed -n 's/^drive[[:space:]]*=[[:space:]]*\([a-z0-9-]*\).*/\1/p' "${PROFILE}" | head -n 1)"
DEFAULT="$(sed -n 's/^surface[[:space:]]*=[[:space:]]*\([a-z0-9-]*\).*/\1/p' "${PROFILE}" | head -n 1)"
PREVIOUS="$(ssh "${TARGET}" "cat '${REMOTE_SURFACE_FILE}' 2>/dev/null || true")"

# Every surface the rig knows: its own [surface.<name>] sections, and its
# drive's library.
KNOWN="$(
    sed -n 's/^\[surface\.\([a-z0-9-]*\)\][[:space:]]*$/\1/p' "${PROFILE}"
    if [[ -n "${DRIVE}" ]]; then
        for f in "${REPO_ROOT}/profiles/surfaces/${DRIVE}"/*.conf; do
            [[ -e "${f}" ]] && basename "${f}" .conf
        done
    fi
)"

echo "========================================"
echo " Heron floor"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Rig:         ${RIG} (drive ${DRIVE:-none})"
echo "Default:     ${DEFAULT:-none} (the profile's [rig] surface)"
echo "Set on Pi:   ${PREVIOUS:-nothing, so the default}"
echo "Known:       $(echo ${KNOWN})"
echo

if [[ -z "${SURFACE}" ]]; then
    echo "heron_robot starts on: ${PREVIOUS:-${DEFAULT:-none}}"
    exit 0
fi

if [[ "${SURFACE}" == "--default" ]]; then
    ssh "${TARGET}" "rm -f '${REMOTE_SURFACE_FILE}'"
    echo "========================================"
    echo " Cleared: heron_robot starts on the default, ${DEFAULT:-none}"
    echo "========================================"
    exit 0
fi

if ! grep -qx "${SURFACE}" <<< "${KNOWN}"; then
    echo "ERROR: rig '${RIG}' has no surface '${SURFACE}'."
    echo "Known: $(echo ${KNOWN})"
    echo "A new floor is profiles/surfaces/${DRIVE}/${SURFACE}.conf (docs/rig-profiles.md), deployed."
    exit 1
fi

# The Pi must hold its numbers too: a surface added on the host since the
# last deploy is not there yet. A rig's own section travels with its profile.
LIBRARY_FILE="heron/profiles/surfaces/${DRIVE}/${SURFACE}.conf"
if ! grep -q "^\[surface\.${SURFACE}\][[:space:]]*$" "${PROFILE}" &&
    ! ssh "${TARGET}" "test -f '${LIBRARY_FILE}'"; then
    echo "ERROR: the Pi does not have ~/${LIBRARY_FILE} yet."
    echo "Deploy it first: scripts/deploy_cpp.sh ${TARGET}"
    exit 1
fi

ssh "${TARGET}" "mkdir -p heron && printf '%s\n' '${SURFACE}' > '${REMOTE_SURFACE_FILE}'"

echo "========================================"
echo " Set: ~/${REMOTE_SURFACE_FILE} = ${SURFACE}"
echo "========================================"
echo
echo "heron_robot starts on ${SURFACE} from its next start; its first lines say so:"
echo "  Surface: ${SURFACE} (this Pi's, ~/heron/surface)"
echo "A running heron_robot, or the boot service, keeps its floor until restarted."
