#!/bin/bash
module purge
module load gnu/gcc/9.3
module load xerces/3.2.3

export SIMPATH=/mnt/simulations/attpcroot/fair_install_18.6/FairSoft
export FAIRROOTPATH=/mnt/simulations/attpcroot/fair_install_18.6/FairRoot
export GENFIT=/mnt/simulations/attpcroot/fair_install_18.6/GenFit

# Add cmake bin to the front of path
if [[ $PATH != *"cmake/bin"* ]]; then
    export PATH=$SIMPATH/../cmake/bin:$PATH
fi

# Add linters to end of path
if [[ $PATH != *"clang"* ]]; then
    export PATH=$PATH:/mnt/analysis/e12014/clangInstall/install/bin
fi
if [[ $PATH != *"iwyu"* ]]; then
    export PATH=$PATH:/mnt/simulations/attpcroot/fair_install_18.6/iwyu/bin/
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

if [ -f $SCRIPT_DIR/build/config.sh ]; then
    source $SCRIPT_DIR/build/config.sh
fi
