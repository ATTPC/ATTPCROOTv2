#!/bin/bash
RED="\e[31m"
GREEN="\e[32m"
ENDCOLOR="\e[0m"

# Ordered list of tests to run
tests=("run_sim_attpc.C" "run_digi_attpc.C" "run_eve_sim.C" "run_unpack_attpc.C" "run_unpack_graw.C" "run_eve.C") 

./symLink.sh 

for i in ${!tests[@]}; do
    echo "Test $i: running ${tests[$i]}"
    if (( $i == 0 )); then
	root -l -q ${tests[$i]} &> test.log
    else
	root -l -q ${tests[$i]} &>> test.log
    fi

    retCode=$?
    color=${RED}
    if (( $retCode == 0 )); then
	color=${GREEN}
    fi
    echo -e "${color}Test $i: returned code $retCode ${ENDCOLOR}"

done
