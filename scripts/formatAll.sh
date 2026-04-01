#!/bin/bash

#Break if something errors
set -e
echo "Indenting all source (.cxx, .C) and header files (.h)"

#Make sure $VMCWORKDIR is set.
if [ -z "$VMCWORKDIR" ]
then
    echo "VMCWORKDIR is unset. Please run config.sh on build directory first. Aborting..."
    return
fi

# Directories excluded from formatting (matches .github/workflows/main.yml)
EXCLUDE_DIRS="macro scripts compiled resources include geometry build .venv lib bin"

# Build a prune expression for find
PRUNE_EXPR=""
for dir in $EXCLUDE_DIRS; do
    PRUNE_EXPR="$PRUNE_EXPR -path $VMCWORKDIR/$dir -prune -o"
done

find -L $VMCWORKDIR $PRUNE_EXPR -type f \( -name "*.h" -o -name "*.cxx" -o -name "*.C" \) -print \
    | xargs -I fname clang-format-17 -i --verbose fname

echo "Success! Indented all source and header files"
