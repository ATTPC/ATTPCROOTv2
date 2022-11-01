if [ -z "$1" ];
then
  echo "== Input the first and last events and the step followed by boolean for interactive mode. Followed by run number, fit direction and simulation convention"
  echo "==   ex) 0 10000 100 0 run_0118 0 0"

  exit 1
fi

FIRSTEVE=$1
LASTEVE=$2
STEP=$3
BOOLINT=$4
RUN=$5
DIREC=$6
SIM=$7
TOTEV=$(($LASTEVE-$FIRSTEVE))
printf "Number of events  $TOTEV \n"
OUTPUT=runFit_${STEP}step_${TOTEV}ev_${RUN}_${DIREC}dir_${SIM}sim.txt
NUMCOM=$(( ($TOTEV/$STEP)-2  )) 
> $OUTPUT

FIRSTBUFF=$(($FIRSTEV+$STEP))

printf "./build/eFitterSM $FIRSTEVE $FIRSTBUFF 0 $RUN $DIREC $SIM\n" >> $OUTPUT


for (( i = 0; i <= $NUMCOM; i++ )) 
do    
    SECONDBUFF=$(($FIRSTBUFF+$STEP))
    printf "./build/eFitterSM $FIRSTBUFF $SECONDBUFF 0 $RUN $DIREC $SIM\n" >> $OUTPUT
    FIRSTBUFF=$(($FIRSTBUFF+$STEP))
    
	     done
