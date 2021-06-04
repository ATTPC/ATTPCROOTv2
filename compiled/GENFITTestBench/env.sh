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
