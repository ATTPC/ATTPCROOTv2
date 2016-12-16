#!/bin/bash

if [ -z "$1" ];
then
  echo "==  Please provide the name of the run file "
  echo "==   i.e: ./create_runfile.sh ar46_run_0128 /data/ar46/run_0128/"

  return;
fi

RUN=$1

OUTPUT=$1\.txt
BUFF=buffer.txt
> $PWD/$OUTPUT

readlink -f $2/*.* >> $BUFF
sed '/lookup/d' $BUFF >> $OUTPUT
rm -rf $BUFF
#sort -t "o" -k1n,1 $OUTPUT
#sort -t"." -k2n,2  $OUTPUT
#sort -n $OUTPUT >> $OUTPUT
