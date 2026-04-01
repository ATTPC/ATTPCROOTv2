#!/bin/bash
# Run the ELossComparison integration macro.
# Must be called after sourcing build/config.sh (sets VMCWORKDIR, LD_LIBRARY_PATH, etc.)

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

if [ -z "${VMCWORKDIR}" ]; then
    echo "ERROR: VMCWORKDIR is not set.  Source build/config.sh first." >&2
    exit 1
fi

echo "Running ELossComparison..."
root -l -b -q ELossComparison.C

echo "Done.  Output: ${SCRIPT_DIR}/ELossComparison.pdf"
