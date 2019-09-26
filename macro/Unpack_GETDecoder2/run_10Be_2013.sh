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

  if [ ! -f  runfiles/ND/10Be_2013/10Be_2013_run_000$i.txt ]; then
    echo "File 10Be_2013_run_000$i.txt not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "run_unpack_proto_10Be.C(\"runfiles/ND/10Be_2013/10Be_2013_run_000$i.txt\",\"pATTPC.TRIUMF2015.par\")"
  mv output_proto.root 10Be_2013_run_000$i.root
  ((i=i+1));
done
