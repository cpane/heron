#!/usr/bin/env bash

# Tell a Pi which rig it is.
#
# Each physical Heron (the bench rig, the mini robot, ...) has its own Pi and
# its own profile, profiles/<rig>.conf: the numbers measured on that rig
# (docs/rig-profiles.md). A Pi holds its rig's name in ~/heron/rig, one line,
# and programs load ~/heron/profiles/<that name>.conf, which deploy_cpp.sh
# copies there. Run this once per Pi, and again only if a Pi moves to a
# different rig.
#
#   scripts/set_rig.sh <user>@<pi> bench
#
# The name must have a profile in this repository, so a Pi can never be set
# to a rig that has no numbers. No sudo: the file is the deploy user's.

set -Eeuo pipefail

if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <user@host> <rig>"
    echo "Example: $0 <user>@<pi> bench"
    exit 1
fi

TARGET="$1"
RIG="$2"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi> bench"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PROFILE="${REPO_ROOT}/profiles/${RIG}.conf"
REMOTE_RIG_FILE="heron/rig"

# The same rule heron_profile applies (isValidRigName), checked here so a bad
# name is refused before anything is written.
if [[ ! "${RIG}" =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]]; then
    echo "ERROR: '${RIG}' is not a valid rig name: up to 32 of a-z, 0-9 and '-'."
    exit 1
fi

if [[ ! -f "${PROFILE}" ]]; then
    echo "ERROR: no profile for rig '${RIG}': ${PROFILE} does not exist."
    echo "Rigs with a profile:"
    for f in "${REPO_ROOT}"/profiles/*.conf; do
        [[ -e "${f}" ]] && echo "  $(basename "${f}" .conf)"
    done
    exit 1
fi

echo "========================================"
echo " Heron rig name"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Rig:         ${RIG}"
echo "Profile:     profiles/${RIG}.conf"
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

echo
PREVIOUS="$(ssh "${TARGET}" "cat '${REMOTE_RIG_FILE}' 2>/dev/null || true")"
if [[ -z "${PREVIOUS}" ]]; then
    echo "The Pi had no rig set."
elif [[ "${PREVIOUS}" == "${RIG}" ]]; then
    echo "The Pi was already rig '${RIG}'."
else
    echo "The Pi was rig '${PREVIOUS}'; it becomes '${RIG}'."
fi

ssh "${TARGET}" "mkdir -p heron && printf '%s\n' '${RIG}' > '${REMOTE_RIG_FILE}'"

echo
echo "========================================"
echo " Set: ~/${REMOTE_RIG_FILE} = ${RIG}"
echo "========================================"
echo
echo "Deploy the profiles with the binaries, and check what the Pi reads:"
echo "  scripts/deploy_cpp.sh ${TARGET} rig_profile show"
