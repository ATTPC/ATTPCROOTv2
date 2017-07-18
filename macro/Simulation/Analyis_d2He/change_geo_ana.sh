#!/bin/bash


geometries="_1atm  _07atm  _05atm  _03atm"
old="_1atm"
hole="elhueco=0  elhueco=10  elhueco=20  elhueco=30  elhueco=40"
oldhole="elhueco=0"

for i in $geometries; do
        sed  -i "0,/$old/  s/$old/$i/g"  d2He_ana_12N.C
        for k in $hole; do
                sed  -i "s/$oldhole/$k/g"  d2He_ana_12N.C
                root.exe -b -q  d2He_ana_12N.C                
                wait
                oldhole=$k
        done
        old=$i
done

