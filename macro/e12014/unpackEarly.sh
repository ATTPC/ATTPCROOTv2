#!/bin/bash

MINRUN=0
MAXRUN=130

LOGFILE=unpackEarly.log

rm -f $LOGFILE
for RUN in $(seq $MINRUN $MAXRUN)
do
    echo "Unpacking run $RUN"
    root -l -b -q "unpack.C($RUN)" >> $LOGFILE 2>&1
done
