#!/bin/bash
i=0;
rm ./data/pulser-files.txt
for filename in /mnt/analysis/e12014/TPC/pulserTest220926/graw/run_0036/*.graw; do
    ln -sf $filename "./data/file${i}_0.graw";
    echo "./data/file${i}_0.graw" >> ./data/pulser-files.txt
    ((i=i+1));
done
