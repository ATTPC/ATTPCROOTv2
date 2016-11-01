if [ ! -e qt-everywhere-opensource-src-$QT_VERSION.tar.gz ];
then
wget $QT_LOCATION
fi
if [ ! -d "~/qt-everywhere-opensource-src-$QT_VERSION" ];
then
tar xzf qt-everywhere-opensource-src-$QT_VERSION.tar.gz
mv qt-everywhere-opensource-src-$QT_VERSION ~/
cd ~/qt-everywhere-opensource-src-$QT_VERSION
./configure
make -j4
make install
fi
