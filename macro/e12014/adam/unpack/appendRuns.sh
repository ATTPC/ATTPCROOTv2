#!/bin/bash

# Will append runs $1 to $2 to file $3


for run in `seq $1 $2`
do
    echo "root -b -l -q \"unpack_linked.C($run)\"" >> $3
done
