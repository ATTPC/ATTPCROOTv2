#!/bin/bash

#runss800=(2089  2090  2091  2092  2093  2094  2095  2096  2097  2098 2100  2101  2102  2103  2104)
#runss800=(2082 2085 2086 2087)
runss800=(2079 2085 2086 2087 2088)

#runstpc=(89  90  91  92  93  94  95  96  97  98  100  101  102  103  104)
#runstpc=(82 85 86 87)
#runstpc=(19 20 21 22 23 24 25 26 27 28 29 30 31 32 33)
runstpc=(79 85 86 87 88)


PID_LIST=""
nbJobs="0"
irun="0"
len=${#runss800[@]}
remain=$len
initi="4"
if [[ $len -lt 5 ]]; then
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

if [[ nbJobs -lt 5 ]]; then
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
