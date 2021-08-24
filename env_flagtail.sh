#!/bin/bash
module purge
module load fairroot/18.6.3
#module load xerces/3.2.3
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source $SCRIPT_DIR/build/config.sh -p

#export PATH=/mnt/analysis/e12014/clangInstall/install/bin/:$PATH
#export LD_LIBRARY_PATH=/mnt/analysis/e12014/clangInstall/install/lib:$LD_LIBRARY_PATH
