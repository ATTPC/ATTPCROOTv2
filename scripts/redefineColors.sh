#!/bin/bash

#Break if something errors
set -e
echo "Changing #define cCOLOR to constexpr string cCOLOR"

#Make sure $VMCWORKDIR is set. 
#if [ -z "$VMCWORKDIR" ]
#then
#    echo "VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting..."
#    return
#fi

for file in `find -L $VMCWORKDIR -type f | grep -w cxx$`
#for file in `find -L $VMCWORKDIR -type f | grep -w h$`
do
    echo Updating $file
    sed -i -E 's/#define (c[A-Z]+) (\"\\.+\")/constexpr auto \1 = \2;/' $file
    #sed -i -E 's/constexpr std::string (.+);/constexpr auto \1;/' $file
    #sed -i -E 's/#include \"(T.+\.h)\"/#include <\1>/' $file
    #sed -i -E 's/#include \"(R.+\.h)\"/#include <\1>/' $file
    
done

#find -L $VMCWORKDIR -type f | grep -w h$ | xargs -I fname clang-format -i --verbose fname
#find -L $VMCWORKDIR -type f | grep -w cxx$ | xargs -I fname clang-format -i --verbose fname

echo "Success! Indented all source and header files"
