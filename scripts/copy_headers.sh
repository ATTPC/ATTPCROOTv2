#!/bin/bash
RED="\033[0;31m"
GREEN="\033[0;32m"
NC='\033[0m' # No Color
set -e
printf "${GREEN}Copying project headers into include folder.${NC}\n"
if [ -z "$VMCWORKDIR" ];
then printf "${RED}VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting...\n${NC} "; return;
else printf "${GREEN}VMCWORKDIR is set to '$VMCWORKDIR'${NC}\n";
fi
rm -rf $VMCWORKDIR/include/
mkdir $VMCWORKDIR/include/
find $VMCWORKDIR -type f | grep -w h$ | xargs -i cp {} $VMCWORKDIR/include/
find $VMCWORKDIR -type f | grep -w hh$ | xargs -i cp {} $VMCWORKDIR/include/
printf "${GREEN}Success! Project headers copied into '$VMCWORKDIR/include/'${NC}\n"
