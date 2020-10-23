#!/bin/bash


#runss800=(2089  2090  2091  2092  2093  2094  2095  2096  2097  2098 2100  2101  2102  2103  2104)
#runss800=(2082 2085 2086 2087)
runss800=(2064  2085  2086  2087)

#runstpc=(89  90  91  92  93  94  95  96  97  98  100  101  102  103  104)
#runstpc=(82 85 86 87)
runstpc=(64  85  86  87)



N=4 ## max number of parallel processes
(
for i in {0..4}; do
 
  
  ((k=k%N)); ((k++==0)) && wait

  echo "Run" $i

 # echo "run_unpack_Juan.C(${runss800[($i)]}, ${runstpc[($i)]})" &
  root  -q -b -l  "analysis.C(${runss800[($i)]},${runstpc[($i)]})" &
 
done
)
