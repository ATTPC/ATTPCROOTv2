#!/bin/bash

logDir="./logUnpack"

if [ $# -lt 1 ]; then
	echo "Usage: $0 runNum1 [runNum2, ...]"
	exit 1
fi

for runNum in "$@"; do
	logFile="$logDir/run$runNum.log"
	echo "Processing run: $runNum "

	root -b -q -l "unpack_rcnp.C($runNum)" > >(tail -n 10000 > $logFile) 2>&1
    echo "\(>.<)/ Unpacking is completed (>=<)"

    root -b -q -l "kine_ana_transfer_reunpack.C($runNum)"
    echo "transfer done!"

    #cd /data/sustech/user/ghy/frib-decode
    /data/sustech/user/ghy/frib-decode/decode $runNum
    /data/sustech/user/ghy/frib-decode/fill_hist $runNum
    
    echo "decode frib done!"

    echo "now check run result!"
    root "check_run.C($runNum)"
    #cd /home/attpc/fair_install/ATTPCROOTv2-RCNPAnalysisBranch/macro/Unpack_HDF5/E581_RCNP
done


