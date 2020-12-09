#!/bin/bash

here=$PWD
cd  ../../../build
cmake ../
make -j
source config.sh
cd $here

