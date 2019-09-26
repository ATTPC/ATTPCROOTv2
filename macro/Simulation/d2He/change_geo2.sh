#!/bin/bash


geometries="_1atm  _07atm  _05atm  _03atm"
old="_1atm"

for i in $geometries; do
        sed  -i "s/$old/$i/g"  d_2He_sim_12N.C
        root.exe -b -q d_2He_sim_12N.C
        old=$i
        wait
done
