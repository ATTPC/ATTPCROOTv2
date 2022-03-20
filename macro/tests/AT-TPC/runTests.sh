#!/bin/bash

# Ordered list of tests to run
tests=("run_sim_attpc.C" "run_digi_attpc.C" "run_eve_sim.C" "run_unpack_attpc.C" "run_eve.C") 

for i in ${!tests[@]}; do
    echo "Starting test $i: ${tests[$i]}"
    if (( $i == 0 )); then
	root -l -q ${tests[$i]} &> test.log
    else
	root -l -q ${tests[$i]} &>> test.log
    fi
    echo "${tests[$i]} returned code $? "

done
