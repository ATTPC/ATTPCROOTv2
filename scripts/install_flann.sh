#!/bin/bash
if [ ! -e flann-$FLANN_VERSION-src.zip ];
then
wget $FLANN_LOCATION
fi

if [ ! -d "~/flann-$FLANN_VERSION-src" ];
then
unzip flann-$FLANN_VERSION-src.zip
mv flann-$FLANN_VERSION-src ~/Downloads
cd ~/Downloads/flann-$FLANN_VERSION-src
mkdir build
cd build
cmake ../
make 
sudo make install
fi
