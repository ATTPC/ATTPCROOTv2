if [ ! -e pcl-$PCL_VERSION.tar.gz ];
then
wget $PCL_LOCATION
fi

if [ ! -d "~/pcl-$PCL_VERSION" ];
then
tar xzf pcl-$PCL_VERSION.tar.gz
mv pcl-pcl-$PCL_VERSION ~/Downloads && cd ~/Downloads/pcl-pcl-$PCL_VERSION
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_visualization=ON  ..
make 
sudo make install
fi
