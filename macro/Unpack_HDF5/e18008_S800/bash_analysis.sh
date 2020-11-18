#!/bin/bash

#runss800=(2089  2090  2091  2092  2093  2094  2095  2096  2097  2098 2100  2101  2102  2103  2104)
#runss800=(2082 2085 2086 2087)
runss800=(2060 2061 2062 2063 2064 2065 2066 2067 2068 2069 2070)

#runstpc=(89  90  91  92  93  94  95  96  97  98  100  101  102  103  104)
#runstpc=(82 85 86 87)
runstpc=(60 61 62 63 64 65 66 67 68 69 70)


PID_LIST=""
nbJobs="0"
irun="0"
len=${#runss800[@]}
remain=$len
initi="3"
if [[ $len -lt 4 ]]; then
	#let "initi = $len"
	let "initi = $len - 1"
fi

while [ $remain -gt 0 ]
do
if [[ irun -eq 0 ]]; then
	#for i in {0..$initi}; do
	for (( i = 0; i <= $initi; i++ ))
	do
   		echo "Run" ${runss800[($i)]} " " ${runstpc[($i)]}
  		root -b -q "analysis.C(${runss800[($i)]},${runstpc[($i)]})" >& "logAna/ana_run${runstpc[($i)]}.log" &
		PID="$!"
		echo "PID " "$PID"
  		PID_LIST+="$PID "
		let nbJobs++
		let irun++
		let remain--
	done
fi

if [[ nbJobs -lt 4 ]]; then
	echo "Run" ${runss800[($i)]} " " ${runstpc[($i)]}
  	root -b -q "analysis.C(${runss800[$irun]},${runstpc[$irun]})" >& "logAna/ana_run${runstpc[$irun]}.log" &
	PID="$!"
	echo "PID " "$PID"
  	PID_LIST+="$PID "
	let nbJobs++
	let irun++
	let remain--
fi

for process in ${PID_LIST[@]};do

	if pgrep -x "$process" </dev/null
	then
		echo "$process stopped"
	fi

	echo "Process " $process
	wait $process
	echo "PID " "${PID_LIST[@]}"
	let nbJobs--
	#let remain--
	break
  # exit_status=$?
  #script_name=`egrep $process $tmp_file | awk -F ":" '{print $2}' | rev | awk -F "/" '{print $2}' | rev`
done

sleep 3

done
