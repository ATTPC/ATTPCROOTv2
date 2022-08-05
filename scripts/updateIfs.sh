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

#for file in `find "$PWD" -type f | grep -w cxx$`
#do
file=$1
echo Updating $file


#sed -i 's/if (\(.*\) < \(.*\) <= \(.*\))/if ((\1 < \2) \&\& (\2 <= \3))/' $file
#sed -i 's/if (\(.*\) = \(.*\))/if (\1 == \2)/' $file
sed -i 's/h[0-9]\+/h1/' $file
sed -i 's/TH1F \*h1 = new TH1F/h1 = std::make_unique<TH1F>/' $file
sed -i 's/delete h1;//' $file
sed -i 's/f[0-9]\+/f1/' $file
sed -i 's/TF1 \*f1 = new TF1/f1 = std::make_unique<TF1>/' $file
sed -i 's/"f[0-9]\+"/"f1"/' $file

#    sed -i 's/fLogger->Fatal(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(fatal) << "\1";/' $file
 #   sed -i 's/fLogger->Info(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(info) << "\1";/' $file
 #   sed -i 's/fLogger->Debug(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(debug) << "\1";/' $file
 #   sed -i 's/fLogger->Warning(MESSAGE_ORIGIN, "\([^"]*\)");/LOG(warning) << "\1";/' $file

#    sed -i -E 's/#define (c[A-Z]+) (\"\\.+\")/constexpr auto \1 = \2;/' $file
#    sed -i 's/ << FairLogger::endl;/;/' $file
#done

#find -L $VMCWORKDIR -type f | grep -w h$ | xargs -I fname clang-format -i --verbose fname
#find -L $VMCWORKDIR -type f | grep -w cxx$ | xargs -I fname clang-format -i --verbose fname

echo "Success! Indented all source and header files"
