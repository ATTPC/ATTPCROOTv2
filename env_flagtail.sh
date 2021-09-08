#!/bin/bash
module purge
module load fairroot/18.6.3

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source $SCRIPT_DIR/build/config.sh -p
