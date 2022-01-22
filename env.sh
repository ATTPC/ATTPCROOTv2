#!/bin/bash

#### The following enviroment variables must be set to build.

#export SIMPATH=/mnt/misc/sw/x86_64/Debian/10/fairroot/18.6.3/fairsoft
#export FAIRROOTPATH=/mnt/misc/sw/x86_64/Debian/10/fairroot/18.6.3/fairroot
#export CMAKE_PREFIX_PATH=/mnt/misc/sw/x86_64/Debian/10/fairroot/18.6.3:${CMAKE_PREFIX_PATH}
#export GENFIT=/mnt/misc/sw/x86_64/Debian/10/fairroot/18.6.3/genfit

if [ -z $ROOTSYS ]; then
echo "ROOTSYS is not set. Check your ROOT installation."
else
export GENFIT=/mnt/simulations/attpcroot/fair_install_2020/GenFit_Inst
if [ `root-config --arch` = macosx ]; then
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$GENFIT/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GENFIT/lib
else
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GENFIT/lib
fi
fi

