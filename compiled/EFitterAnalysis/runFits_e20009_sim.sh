echo "Bash version ${BASH_VERSION}..."

if [ $# -eq 0 ]; then
    echo "No arguments provided"
    return;
fi

if [ $# -eq 1 ]; then
    echo "Only one argument provided!"
    return;
fi

FOLDER=data_sim_$1_$2

mkdir $FOLDER

END=$2
i=$1

while [[ $i -le $END ]]
do

   list=$(find ./runFiles_e20009_sim -name run*run_sim_0_0_0dir*.txt)
   if [ -z "$list" ]
   then
      echo "Not found"
   else
       echo $list
       ./parallel --verbose --timeout 200% < $list
   fi

   mv *.root $FOLDER

   ((i=i+1));

done
