#!/bin/bash

cd  /home/jcz/FairRoor_workdir/ATTPCROOTv2/build
cmake ../
make 
source config.sh
source incluir_datos.sh 
cd /home/jcz/FairRoor_workdir/ATTPCROOTv2/macro/Simulation

