# Unpacking many runs

You can unpack many runs at once like:
```
./parallel --jobs 5 --joblog ./log.150 < 150torr.txt
```

where `--jobs` sets the maximum number of jobs to run at once. `--joblog ./fileName` sets the log file which will save the command, runtime, and return/error code. The file piped in contains each command to run (one per line).
