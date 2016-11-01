#!/bin/bash

if [ ! -e $EIGEN_VERSION.tar.bz2 ];
then
wget $EIGEN_LOCATION
fi

//if [ ! -d "~/" ];
//then

tar -vxjf $EIGEN_VERSION.tar.bz2 -C ~/
cd ~/eigen*/
mkdir build && cd build
cmake ../
make -j4
sudo make install
