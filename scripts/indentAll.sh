#!/bin/bash

#Break if something errors
set -e
echo "Indenting all source (.cc) and header files (.hh)"

#Make sure $VMCWORKDIR is set. 
if [ -z "$VMCWORKDIR" ]
then
    echo "VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting..."
    return
else
    echo "VMCWORKDIR is set to '$VMCWORKDIR'. Using indent profile $VMCWORKDIR/.indent.pro"
fi

#Make sure there is an intent profile there
if [ ! -f $VMCWORKDIR/.indent.pro ]
then
    echo "Could not find profile $VMCWORKDIR/.indent.pro!"
    return
fi
export INDENT_PROFILE=$VMCWORKDIR/.indent.pro

find -L $VMCWORKDIR -type f | grep -w h$ | xargs -i 'echo {}'
find -L $VMCWORKDIR -type f | grep -w cxx$ | xargs -i 'echo {}'

echo "Success! Indented all source and header files"
