#!/usr/bin/env bash

# Push the compiled C++ binaries to the target, and optionally run one.
#
# The C++ counterpart of deploy_pi.sh. Kept separate rather than folded into
# it: deploy_pi.sh mirrors python/ with --delete, and pointing that at a
# directory of build artifacts would be a different and more destructive
# operation than it looks.
#
#   scripts/deploy_cpp.sh cpane@<pi>
#   scripts/deploy_cpp.sh cpane@<pi> lidar_info
#   scripts/deploy_cpp.sh cpane@<pi> lidar_info /dev/rplidar

set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <user@host> [binary [args...]]"
    echo "Example: $0 cpane@<pi>"
    echo "Example: $0 cpane@<pi> lidar_info"
    exit 1
fi

TARGET="$1"
shift

if [[ "${TARGET}" != *@* ]]; then
    echo "ERROR: Target must be specified as user@host."
    echo "Example: $0 cpane@<pi>"
    exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

LOCAL_BIN_DIR="${REPO_ROOT}/build-pi/bin"

# Beside the Python tree, not inside it: deploy_pi.sh mirrors python/ with
# --delete and would remove anything it finds there that is not in the repo.
REMOTE_ROOT="heron"
REMOTE_BIN_DIR="${REMOTE_ROOT}/bin"

if [[ ! -d "${LOCAL_BIN_DIR}" ]]; then
    echo "ERROR: no build output found at:"
    echo "  ${LOCAL_BIN_DIR}"
    echo "Build first:"
    echo "  scripts/build_cpp.sh"
    exit 1
fi

echo "========================================"
echo " Heron C++ Deployment"
echo "========================================"
echo
echo "Target:      ${TARGET}"
echo "Source:      ${LOCAL_BIN_DIR}"
echo "Destination: ~/${REMOTE_BIN_DIR}"
echo

# A binary built for the wrong architecture deploys fine and then fails with
# a confusing exec format error on the Pi. Catch it on the host, where the
# cause is obvious. `file` is not installed everywhere, so this is advisory.
if command -v file >/dev/null 2>&1; then
    FIRST_BIN="$(ls "${LOCAL_BIN_DIR}" | head -1)"
    if [[ -n "${FIRST_BIN}" ]]; then
        ARCH_DESC="$(file -b "${LOCAL_BIN_DIR}/${FIRST_BIN}")"
        echo "Binary:      ${ARCH_DESC}"
        case "${ARCH_DESC}" in
            *ARM\ aarch64*|*aarch64*)
                ;;
            *)
                echo
                echo "ERROR: these binaries are not aarch64 ELF."
                echo "  The Pi is aarch64 Debian; this looks like a host build."
                echo "  Use scripts/build_cpp.sh, which builds in a container."
                exit 1
                ;;
        esac
        echo
    fi
fi

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
echo "Creating remote directories..."
ssh "${TARGET}" "mkdir -p '${REMOTE_BIN_DIR}'"

echo
echo "Syncing binaries..."

# -rlptvz rather than -a: macOS ships openrsync, whose -a implies owner and
# group preservation that a non-root receiver cannot honour.
#
# --delete so a renamed or removed target cannot linger and be run by accident,
# which is the same guarantee deploy_pi.sh gives for the Python tree.
rsync -rlptvz --delete \
    -e ssh \
    "${LOCAL_BIN_DIR}/" \
    "${TARGET}:${REMOTE_BIN_DIR}/"

if [[ $# -eq 0 ]]; then
    echo
    echo "Deployment complete: ${TARGET}:~/${REMOTE_BIN_DIR}"
    echo
    echo "To deploy and run:"
    echo "  $0 ${TARGET} lidar_info"
    exit 0
fi

BINARY="$1"
shift

echo
echo "Running: ${BINARY} $*"
echo

REMOTE_COMMAND="cd \"\${HOME}/${REMOTE_ROOT}\""
REMOTE_COMMAND="${REMOTE_COMMAND} && exec \"\${HOME}/${REMOTE_BIN_DIR}/${BINARY}\""
if [[ $# -gt 0 ]]; then
    REMOTE_COMMAND="${REMOTE_COMMAND} $(printf '%q ' "$@")"
fi

# -t allocates a TTY so Ctrl-C reaches the process rather than only the ssh
# client. That matters more here than for the probes: a C++ program killed
# without running its handlers leaves the LiDAR motor spinning.
ssh -t "${TARGET}" "${REMOTE_COMMAND}"
