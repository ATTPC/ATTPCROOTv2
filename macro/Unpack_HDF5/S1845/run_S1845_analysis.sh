echo "Bash version ${BASH_VERSION}..."

if [ $# -eq 0 ]; then
    echo "No arguments provided"
    return;
fi

if [ $# -eq 1 ]; then
    echo "Only one argument provided!"
    return;
fi


END=$2
i=$1

while [[ $i -le $END ]]
do

  if [ ! -f  run_00$i.root ]; then
    echo "File run_00$i.root not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "analysis_v2.C(\"run_00$i\")"
  mv experiment_bragg_events.txt run_00$i.txt
  ((i=i+1));
done