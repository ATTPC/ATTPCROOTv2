#!/bin/bash

dir=$(pwd)
simFile="runsim_d2He.C"

if [ -z "$1" ]
then
 echo "Please set the number of events :"
 read nEvents
else
 nEvents=$1
fi

cd $VMCWORKDIR/macro/Simulation/d2He && sed -i "/void runsim_d2He(Int_t nEvents/c\void runsim_d2He(Int_t nEvents = $nEvents, TString mcEngine = \"TGeant4\")" $simFile && root -b -l -q runsim_d2He.C && root -b -l -q rundigi_d2He.C && cd outputFiles/ && cd $dir

