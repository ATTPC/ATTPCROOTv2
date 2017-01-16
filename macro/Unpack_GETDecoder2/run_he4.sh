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

  if [ ! -f  runfiles/NSCL/alphas/alpha_run_00$i.txt ]; then
    echo "File runfiles/NSCL/alphas/alpha_run_00$i.txt not found!"
    ((i=i+1));
    #return;
  fi
  root -b -q -l "run_unpack_alpha_RANSAC_Ana.C(\"runfiles/NSCL/alphas/alpha_run_00$i.txt\",\"ATTPC.alpha.par\",\"/data/ar46/run_0085/\")"
  mv output.root run_00$i.root
  ((i=i+1));
done
