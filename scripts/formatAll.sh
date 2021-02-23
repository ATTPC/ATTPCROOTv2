#!/bin/bash

#Break if something errors
set -e
echo "Indenting all source (.cxx) and header files (.h)"

#Make sure $VMCWORKDIR is set. 
if [ -z "$VMCWORKDIR" ]
then
    echo "VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting..."
    return
fi

find -L $VMCWORKDIR -type f | grep -w h$ | xargs -I fname clang-format -i --verbose fname
find -L $VMCWORKDIR -type f | grep -w cxx$ | xargs -I fname clang-format -i --verbose fname

echo "Success! Indented all source and header files"
