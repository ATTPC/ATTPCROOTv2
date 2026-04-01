# Installation (First-Time Setup)

This page covers the one-time steps to configure and build ATTPCROOT on a new machine. After completing these steps, day-to-day work uses only `source build/config.sh` and `cmake --build build` — the environment variables below are never needed again.

## Prerequisites

FairSoft and FairRoot must already be installed on the machine. FairSoft provides ROOT, Geant4, and VMC. FairRoot provides the task/run framework.

## Step 1 — Set Configure-Time Variables

These two variables tell CMake where FairRoot and FairSoft are installed. They only need to be set in the shell session where you run `cmake` for the first time:

```bash
export FAIRROOTPATH=/path/to/your/fairroot/install
export SIMPATH=/path/to/your/fairsoft/install
```

These are machine-specific. Ask your sysadmin or check the FairRoot/FairSoft install documentation for the correct paths on your system.

## Step 2 — Configure

Run CMake out-of-source from the repo root:

```bash
cmake -S . -B build
```

If HDF5 is not bundled with your FairSoft installation, add its prefix:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/hdf5
```

A successful configure generates `build/config.sh`. This file captures the full runtime environment and is sourced before any subsequent build or analysis session.

## Step 3 — Build

```bash
cmake --build build -j10
```

Adjust `-j` to your CPU count.

## After Installation

From this point on:

- **Before any session and to rebuild:** see [daily-use.md](daily-use.md)

The `FAIRROOTPATH` and `SIMPATH` variables from Step 1 are baked into `build/config.sh` and do not need to be set again.
