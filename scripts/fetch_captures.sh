#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 <user@host>"
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

TARGET="$1"

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

LOCAL_CAPTURE_DIR="${REPO_ROOT}/captures"
REMOTE_CAPTURE_DIR="heron/captures"

echo "========================================"
echo " Heron Capture Retrieval"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Destination: ${LOCAL_CAPTURE_DIR}"
echo

if ! ssh "${TARGET}" "test -d '${REMOTE_CAPTURE_DIR}'"; then
    echo "ERROR: No capture directory on the target:"
    echo "  ${TARGET}:~/${REMOTE_CAPTURE_DIR}"
    echo "Run a probe that records data first."
    exit 1
fi

mkdir -p "${LOCAL_CAPTURE_DIR}"

echo "Fetching captures..."

# No --delete in this direction: the host is the archive. Captures accumulate
# here and are pruned from the Pi by hand, so a failed analysis can never
# destroy the data that would explain the failure.
rsync -rlptvz \
    -e ssh \
    "${TARGET}:${REMOTE_CAPTURE_DIR}/" \
    "${LOCAL_CAPTURE_DIR}/"

echo
echo "Captures retrieved: ${LOCAL_CAPTURE_DIR}"
