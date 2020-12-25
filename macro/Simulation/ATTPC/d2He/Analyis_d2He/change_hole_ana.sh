#!/bin/bash


hole="elhueco=0  elhueco=10  elhueco=20  elhueco=30  elhueco=40"
#oldhole="elhueco=0"

 #       for k in $hole; do
  #               sed  -i "s/$oldhole/$k/g"  d2He_ana_12N_1atm.C
   #             nohup root.exe -b -q  d2He_ana_12N_1atm.C&
    #            
     #           sed  -i "s/$oldhole/$k/g"  d2He_ana_12N_07atm.C
      #          nohup root.exe -b -q  d2He_ana_12N_07atm.C&
       #        
        #        sed  -i "s/$oldhole/$k/g"  d2He_ana_12N_05atm.C
         #       nohup root.exe -b -q  d2He_ana_12N_05atm.C&
          #      
           #     sed  -i "s/$oldhole/$k/g"  d2He_ana_12N_03atm.C
            #    nohup root.exe -b -q  d2He_ana_12N_03atm.C&
             # 
              #  wait
               # oldhole=$k
       # done

wait        

oldhole="elhueco=0"

        for k in $hole; do
                 sed  -i "s/$oldhole/$k/g"  d2He_ana_56Ni_1atm.C
                nohup root.exe -b -q  d2He_ana_56Ni_1atm.C&
                
                sed  -i "s/$oldhole/$k/g"  d2He_ana_56Ni_07atm.C
                nohup root.exe -b -q  d2He_ana_56Ni_07atm.C&
               
                sed  -i "s/$oldhole/$k/g"  d2He_ana_56Ni_05atm.C
                nohup root.exe -b -q  d2He_ana_56Ni_05atm.C&
                
                sed  -i "s/$oldhole/$k/g"  d2He_ana_56Ni_03atm.C
                nohup root.exe -b -q  d2He_ana_56Ni_03atm.C&
              
                wait
                oldhole=$k
        done

