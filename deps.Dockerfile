FROM rootproject/root:6.26.10-ubuntu22.04

# Install git
RUN apt-get update && apt-get install -y git && apt-get -y clean

WORKDIR /

# Clone and build minimalist version of FairSoft repository (nov22)
RUN git clone -b nov22_patches https://github.com/anthoak13/FairSoft.git && cd FairSoft && \
    cmake -S /FairSoft -B /FairSoft/build -C /FairSoft/FairSoftConfig.cmake -DCMAKE_INSTALL_PREFIX=/opt/FairSoft && \
    cmake --build /FairSoft/build -j8 && \
    rm -rf /FairSoft

ENV SIMPATH=/opt/FairSoft
WORKDIR /

# Clone and build FairRoot repository
RUN git clone -b v18.8_patches https://github.com/FairRootGroup/FairRoot.git && cd FairRoot && \
    mkdir build && cd build && \
    cmake -DBUILD_BASEMQ=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX=/opt/FairRoot .. && \
    make -j8 && make install && \
    rm -rf /FairRoot

ENV FAIRROOTPATH=/opt/FairRoot
WORKDIR /

# Install HDF5
RUN wget https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.10/hdf5-1.10.4/src/hdf5-1.10.4.tar.gz && \
    tar -xvf hdf5-1.10.4.tar.gz && mkdir hdf5-1.10.4/build && cd hdf5-1.10.4/build && \
    cmake -DCMAKE_INSTALL_PREFIX=/opt/hdf5 .. && \
    make -j 8 && make install && \
    rm -rf /hdf5-1.10.4 && rm -rf /hdf5-1.10.4.tar.gz

CMD [ "bash" ]