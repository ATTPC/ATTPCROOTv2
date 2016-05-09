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

  if [ ! -f  runfiles/ar46_run_0$i.txt ]; then
    echo "File ar46_run_0$i.txt not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "run_unpack2.C(\"runfiles/ar46_run_0$i.txt\",\"ATTPC.e15503b.par\",\"/data/ar46/run_0085/\")"
  mv output.root run_0$i.root
  ((i=i+1));
done
