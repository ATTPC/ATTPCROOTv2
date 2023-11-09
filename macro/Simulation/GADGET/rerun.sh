#!/bin/bash
cd ../../../build
make install -j 8
cd -
root -x rundigi_sim_Ari.C
root -l run_eve_new.C
