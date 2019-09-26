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

  if [ ! -f  runfiles/TRIUMF/8He_TRIUMF_run_0$i.txt ]; then
    echo "File 8He_TRIUMF_run_0$i.txt not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "run_unpack_proto_8He.C(\"runfiles/TRIUMF/8He_TRIUMF_run_0$i.txt\",\"pATTPC.TRIUMF2015.par\")"
  mv output_proto.root 8He_TRIUMF_run_0$i.root
  ((i=i+1));
done
