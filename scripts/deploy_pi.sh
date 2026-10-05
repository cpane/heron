#!/usr/bin/env bash

set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <user@host> [python-file [args...]]"
    echo "Example: $0 <user>@<pi>"
    echo "Example: $0 <user>@<pi> probes/lidar_01_port.py"
    exit 1
fi

TARGET="$1"
shift

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 <user>@<pi>"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

LOCAL_PYTHON_DIR="${REPO_ROOT}/python"

# Remote paths are relative to the target user's home, so deployment never
# requires root and the script is not tied to one username.
#
# Captures live beside the deployed tree rather than inside it: the sync below
# deletes anything on the target that is not in the repository, and recorded
# data is the one thing on the Pi that cannot be reproduced.
REMOTE_ROOT="heron"
REMOTE_PYTHON_DIR="${REMOTE_ROOT}/python"
REMOTE_CAPTURE_DIR="${REMOTE_ROOT}/captures"

if [[ ! -d "${LOCAL_PYTHON_DIR}" ]]; then
    echo "ERROR: Python source directory not found:"
    echo "  ${LOCAL_PYTHON_DIR}"
    exit 1
fi

echo "========================================"
echo " Heron Python Deployment"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Source:      ${LOCAL_PYTHON_DIR}"
echo "Destination: ~/${REMOTE_PYTHON_DIR}"
echo

echo "Checking SSH connectivity..."

# BatchMode disables the password prompt, so this succeeds only when key-based
# authentication is configured. Each deploy opens two or three connections,
# which is two or three password prompts per edit-run cycle without a key.
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
echo "Creating remote directories..."

# rsync does not create missing parent directories, and the capture directory
# must exist before the first probe that records data runs.
ssh "${TARGET}" "mkdir -p '${REMOTE_PYTHON_DIR}' '${REMOTE_CAPTURE_DIR}'"

echo
echo "Syncing Python sources..."

# -a is avoided deliberately: macOS ships openrsync, and -a implies owner and
# group preservation that a non-root receiver cannot honour. -rlptvz is the
# useful subset that openrsync supports.
#
# --delete makes the target an exact mirror of the repository, so a renamed or
# deleted probe cannot be run by accident.
rsync -rlptvz --delete \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude '.venv/' \
    --exclude '.DS_Store' \
    -e ssh \
    "${LOCAL_PYTHON_DIR}/" \
    "${TARGET}:${REMOTE_PYTHON_DIR}/"

if [[ $# -eq 0 ]]; then
    echo
    echo "Deployment complete: ${TARGET}:~/${REMOTE_PYTHON_DIR}"
    echo
    echo "To deploy and run a probe:"
    echo "  $0 ${TARGET} probes/<name>.py [args...]"
    exit 0
fi

echo
echo "Running: $*"
echo

# PYTHONPATH points at the deployed python/ directory so scripts in
# python/probes can import the heron package with no packaging step and no
# sys.path manipulation inside the probes themselves.
#
# printf %q quotes each argument for the remote shell, which re-parses
# everything ssh hands it.
REMOTE_COMMAND="cd \"\${HOME}/${REMOTE_PYTHON_DIR}\""
REMOTE_COMMAND="${REMOTE_COMMAND} && PYTHONPATH=\"\${HOME}/${REMOTE_PYTHON_DIR}\""
REMOTE_COMMAND="${REMOTE_COMMAND} HERON_CAPTURE_DIR=\"\${HOME}/${REMOTE_CAPTURE_DIR}\""
REMOTE_COMMAND="${REMOTE_COMMAND} exec python3 -u $(printf '%q ' "$@")"

# -t allocates a TTY so Ctrl-C reaches the Python process rather than only the
# ssh client. That matters here: an orphaned process leaves the LiDAR motor
# spinning, and this board already reports under-voltage.
ssh -t "${TARGET}" "${REMOTE_COMMAND}"
