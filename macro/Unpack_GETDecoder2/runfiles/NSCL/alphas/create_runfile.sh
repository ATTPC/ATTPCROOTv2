#!/bin/bash

if [ -z "$1" ];
then
  echo "== Please provide the output filename"
  echo "==  i.e: ./create_runfile.sh ar46_run_0128 /data/ar46/run_0128"

  return;
fi

if [ -z "$2" ];
then
  echo "== Please provide a directory to read files from"
  echo "==  i.e: ./create_runfile.sh ar46_run_0128 /data/ar46/run_0128"

  return;
fi

OUTPUT=$1\.txt

# get all files and folders ending on *.graw in the target directory (one per line)
ls --color=never -1 $2/*.graw |
# get the number following "CoBo" and (eventually) the number immediately before ".graw"
# and write them like this: (\t beeing the tab-character)
# {CoBoNumber}\t{Before.grawNumber}\t{original line}
  perl -pe "s|(.*CoBo([0-9]+).*?(?:\.([0-9]*))?\.graw)|\2\t\3\t\1|" |
# sort numerically by the first 2 columns
  sort -n -k1,1 -k2,2 |
# drop everything - except the third column
# and write result into output file
  cut -f3 > $OUTPUT
