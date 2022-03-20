#!/bin/bash

geometryList=("ATTPC_He1bar.C" "ATTPC_v1_1.C" "SpecMAT_He1Bar.C" "GADGET_II_lp.C" "GADGET_II.C")

cd ${VMCWORKDIR}/geometry/

for geo in ${geometryList[@]}; do

    echo "Generating geometry: $geo"
    root -l -q $geo

done
