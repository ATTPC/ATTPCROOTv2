#!/bin/bash
if [ ! -d "~/VTK" ];
then
git clone $VTK_LOCATION ~/VTK
fi
install_prefix=/usr/local

cd ~/VTK
mkdir build
cmake -DQT_QMAKE_EXECUTABLE:PATH=$QT_BIN/qmake  -DVTK_Group_Qt:BOOL=ON  -DBUILD_SHARED_LIBRARIES:BOOL=ON ~/VTK
make -j4
sudo make install
