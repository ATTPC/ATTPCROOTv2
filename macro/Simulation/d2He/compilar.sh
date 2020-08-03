#!/bin/bash

here=$PWD
cd  ../../../build
#cmake ../
make -j3
source config.sh
#export G4ENSDFSTATEDATA=$ROOTSYS/share/Geant4/data/G4ENSDFSTATE
#export G4REALSURFACEDATA=$ROOTSYS/share/Geant4/data/RealSurface
cd $here

