#!/bin/bash
# run two processes in the background and wait for them to finish

nohup sleep 3 &
nohup sleep 10 &

echo "This will wait until both are done"
date
wait
date
echo "Done"
