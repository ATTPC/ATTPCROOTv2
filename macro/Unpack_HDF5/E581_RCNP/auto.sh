#!/bin/bash

for i in {4081..4082}; do
        root -l -q "kine_ana_transfer_reunpack.C($i)" 
done

