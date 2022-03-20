#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Loop through every directory and run tests
for d in */; do
    echo $d
    cd ${SCRIPT_DIR}/$d
    runTests.sh
    echo
done
