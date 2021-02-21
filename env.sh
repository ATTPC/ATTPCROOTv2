#!/bin/bash
module purge

source build/config.sh


module load gnu/gcc/6.4
module load fairroot/18.00

export INDENT_PROFILE=$VMCWORKDIR/.indent.pro
export PATH=/mnt/analysis/e12014/clangInstall/install/bin/:$PATH
export LD_LIBRARY_PATH=/mnt/analysis/e12014/clangInstall/install/lib:$LD_LIBRARY_PATH
