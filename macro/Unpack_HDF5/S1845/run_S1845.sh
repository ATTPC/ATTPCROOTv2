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

  if [ ! -f  /home/ayyadlim/Desktop/get/files/run_00$i.h5 ]; then
    echo "File run_00$i.h5 not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "run_unpack_HC.C(\"/home/ayyadlim/Desktop/get/files/run_00$i.h5\",\"pATTPC.S1845.par\",\"/data/ar46/\")"
  mv output_proto.root run_00$i.root
  ((i=i+1));
done