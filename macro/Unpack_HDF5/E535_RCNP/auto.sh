#!/bin/bash

for i in {3001..3094}; do
        root -l -q "kine_ana_transfer.C($i)" 
done

