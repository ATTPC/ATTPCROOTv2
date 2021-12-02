if [ -z "$1" ];
then
  echo "== Input the first and last events and the step followed by boolean for interactive mode."
  echo "==   ex) 0 10000 100 false"

  exit 1
fi

FIRSTEVE=$1
LASTEVE=$2
STEP=$3
BOOLINT=$4
TOTEV=$(($LASTEVE-$FIRSTEVE))
printf "Number of events  $TOTEV \n"
OUTPUT=runFit_${STEP}step_${TOTEV}ev.txt
NUMCOM=$(( ($TOTEV/$STEP)-2  )) 
> $OUTPUT

FIRSTBUFF=$(($FIRSTEV+$STEP))

printf "./build/genfitSAexe $FIRSTEVE $FIRSTBUFF false\n" >> $OUTPUT


for (( i = 0; i <= $NUMCOM; i++ )) 
do    
    SECONDBUFF=$(($FIRSTBUFF+$STEP))
    printf "./build/genfitSAexe $FIRSTBUFF $SECONDBUFF false\n" >> $OUTPUT
    FIRSTBUFF=$(($FIRSTBUFF+$STEP))
    
	     done
