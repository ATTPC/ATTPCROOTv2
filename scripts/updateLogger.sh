#!/bin/bash

#Break if something errors
set -e
echo "Indenting all source (.cxx) and header files (.h)"

#Make sure $VMCWORKDIR is set. 
#if [ -z "$VMCWORKDIR" ]
#then
#    echo "VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting..."
#    return
#fi

for file in `find "$PWD" -type f | grep -w cxx$`
do
    echo Updating $file
    
    sed -i 's/fLogger->Error(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(error) << "\1";/' $file
    sed -i 's/fLogger->Fatal(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(fatal) << "\1";/' $file
    sed -i 's/fLogger->Info(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(info) << "\1";/' $file
    sed -i 's/fLogger->Debug(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(debug) << "\1";/' $file
    sed -i 's/fLogger->Warning(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(warning) << "\1";/' $file

    sed -i 's/ << FairLogger::endl;/;/' $file
done

#find -L $VMCWORKDIR -type f | grep -w h$ | xargs -I fname clang-format -i --verbose fname
#find -L $VMCWORKDIR -type f | grep -w cxx$ | xargs -I fname clang-format -i --verbose fname

echo "Success! Indented all source and header files"
