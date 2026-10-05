#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <user@host>"
    echo "Example: $0 cpane@<pi>"
    exit 1
fi

TARGET="$1"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 cpane@<pi>"
    exit 1
fi

TARGET_USER="${TARGET%%@*}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

LOCAL_TARGET_DIR="${REPO_ROOT}/target"
LOCAL_SCRIPT="${LOCAL_TARGET_DIR}/provision/base.sh"

# The whole target/ tree is staged rather than base.sh alone, because
# provisioning installs data files (udev rules) that live beside it.
REMOTE_STAGE_DIR="/tmp/heron-provision"
REMOTE_SCRIPT="${REMOTE_STAGE_DIR}/provision/base.sh"

if [[ ! -f "${LOCAL_SCRIPT}" ]]; then
    echo "ERROR: Provisioning script not found:"
    echo "  ${LOCAL_SCRIPT}"
    exit 1
fi

cleanup() {
    ssh "${TARGET}" "rm -rf '${REMOTE_STAGE_DIR}'" >/dev/null 2>&1 || true
}

trap cleanup EXIT

echo "========================================"
echo " Heron Target Provisioning"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Target user: ${TARGET_USER}"
echo

echo "Checking SSH connectivity..."
ssh "${TARGET}" "echo 'Connected to:' \$(hostname)"

echo
echo "Copying target configuration..."

# Removed first so that a stale rule file from an earlier revision of the
# repository cannot be installed alongside the current ones.
ssh "${TARGET}" "rm -rf '${REMOTE_STAGE_DIR}' && mkdir -p '${REMOTE_STAGE_DIR}'"
scp -r "${LOCAL_TARGET_DIR}/." "${TARGET}:${REMOTE_STAGE_DIR}/"

echo
echo "Running target provisioning..."
ssh -t "${TARGET}" \
    "chmod +x '${REMOTE_SCRIPT}' && sudo '${REMOTE_SCRIPT}' '${TARGET_USER}'"

echo
echo "Provisioning complete: ${TARGET}"
echo
echo "Next step:"
echo "  ${SCRIPT_DIR}/deploy_pi.sh ${TARGET}"
