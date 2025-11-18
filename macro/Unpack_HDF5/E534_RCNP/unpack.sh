#!/bin/bash

logDir="./logUnpack"

if [ $# -lt 1 ]; then
	echo "Usage: $0 runNum1 [runNum2, ...]"
	exit 1
fi

for runNum in "$@"; do
	logFile="$logDir/run$runNum.log"
	echo "Processing run: $runNum "

	root -b -q -l "unpack_rcnp.C($runNum)" > >(tail -n 1000 > $logFile) 2>&1
done

echo "\(>.<)/ Unpacking is completed (>=<)"
