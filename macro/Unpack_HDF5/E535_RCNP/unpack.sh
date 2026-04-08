#!/bin/bash

logDir="./logUnpack_veto"

if [ $# -lt 1 ]; then
	echo "Usage: $0 runNum1 [runNum2, ...]"
	exit 1
fi

for runNum in "$@"; do
	logFile="$logDir/run$runNum.log"
	echo "Processing run: $runNum (>'-')>.h5 "

	root -b -q -l "unpack_rcnp.C($runNum)" > >(tail -n 10000 > $logFile) 2>&1
	#root -b -q -l "unpack_rcnp.C($runNum)" 
	#root -b -q -l "unpack_rcnp.C($runNum)" > > $logFile 2>&1
	#./unpacker $runNum > >(tail -n 10000 > $logFile) 2>&1
done

echo ".root<('-'<)\(>.<)/ Unpacking is completed ~('-'~) (~'-')~"
