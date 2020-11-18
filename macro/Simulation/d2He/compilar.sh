#!/bin/bash

here=$PWD
cd  ../../../build
cmake ../
make -j3
source config.sh
cd $here
