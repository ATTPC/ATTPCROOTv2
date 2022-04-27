ATTPCROOT is a ROOT-based (root.cern.ch) framework to analyze data of active target detectors including the ATTPC (Active Target Time Projection Chamber) detector and the p-ATTPC (Prototype). For reference http://ribf.riken.jp/ARIS2014/slide/files/Jun2/Par2B06Bazin-final.pdf. The detector is based at the NSCL but experiments are performed at other facilities as well (Notre Dame, TRIUMF, Argonne...).
 
The framework allows the end user to unpack and analyze the data, as well as perform realistic simulations based on a Virtual Monte Carlo (VMC) package. The framework needs external libraries (FairSoft and FairRoot https://fairroot.gsi.de/) to be compiled and executed, which are developed by other groups and not directly supported by the AT-TPC group. Please refer to their forums for installation issues of these packages.

Doxygen documentation for the most recent release can be found [here](http://attpc.github.io/ATTPCROOTv2/).
Doxygen documentation tracking the develop branch can be found [here](https://anthoak13.github.io/ATTPCROOTv2/)

There was also some documentation written as part of Alexander Carl's 2019 thesis which can be found [here](https://publications.nscl.msu.edu/thesis/%20Carls_2019_6009.pdf)

# Table of contents

- [For developers](#for-developers)
  - [Coding conventions](#coding-conventions)
    - [Object Ownership](#object-ownership)
    - [Formatting code for PRs](#formatting-code-for-pull-requests)
    - [Adding a class](#adding-a-class)
    - [Adding a task](#adding-a-task)
    - [Adding a library](#adding-a-library)
- [Installation](#installation)
  - [On NSCL/FRIB cluster](#installation-on-frib-cluster)
  - [Building from scratch](#installation-from-scratch)
  - [Linking against ATTPCROOT](#linking-against-attpcroot)

# Installation

## Installation on FRIB cluster

ROOT and FairROOT are already installed on the system. As of 1/14/2021 the IT department has decided they no longer want to maintain the installation, the the process has changed. The prequisites for installation can now bew found in the directory `/mnt/simulation/attpcroot/fair_install_18.6/`. It is recomended to use the `env_fishtank.sh` script to set the required modules and enviroment variables that used to be handled just through the fishtank module system.

To install ATTPCROOT checkout the repository from github and source the proper enviroment script. This code requires an out of source build, so create a folder in the repository to build the code and cd into it. From this directory you can call CMake to configure the build, setting the CMAKE_PREFIX_PATH so cmake can find all of the prequisites. Then you can build the source. 

```
git clone https://github.com/ATTPC/ATTPCROOTv2.git
cd ATTPCROOTv2
source env_fishtank.sh
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=/mnt/simulations/attpcroot/fair_install_18.6/ ../
make install -j 4 
source config.sh
```

After the source builds, you should be good to go. The CMake script will output a configuration file in the build directory called `config.sh`. This script should be sourced before building or running any macros. For those working on the FRIB fishtank computers, the enviroment script `env_fishtank.sh` can be sourced instead.

## Installation from scratch

The ATTPCROOT code depends on the following external packages, with tested version numbers:

- Required dependencies
  - Compiler with support for C++14 standard
  - [Xerces](https://xerces.apache.org/) (3.2.3)
  - [FairSoft](https://github.com/FairRootGroup/FairSoft/releases/tag/apr21p2) (apr21 with ROOT fftw3 module)
  - [FairRoot](https://github.com/FairRootGroup/FairRoot/releases/tag/v18.6.5) (18.6.5)
  - [HDF5](https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.10/hdf5-1.10.4/src/) (1.10.4)
    - [PCL](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.10.1) (1.10.1)
    - [FLANN](https://github.com/flann-lib/flann/releases/tag/1.9.1) (1.9.1)
    - [Eigen](https://gitlab.com/libeigen/eigen/-/releases/3.3.9) (3.3.9)

- Optional dependencies 
  - [GenFit](https://github.com/Yassid/GenFit)
  - [HiRAEVT](https://github.com/nscl-hira/HiRAEVT)
  - [IWYU](https://github.com/include-what-you-use/include-what-you-use) for static analysis


### Installation of prequisites

Coppied below are the instructions for building and installing the prequisites on FRIB fishtank. Hopefully they will be of use installing the software on other systems:

#### FairSoft

Run `module purge`, `module load gnu/gcc/9.3`, `module load xerces/3.2.3`
run `export XercesC_ROOT=/mnt/misc/sw/x86_64/Debian/10/xerces/3.2.3/`

Follow instructions here https://github.com/FairRootGroup/FairSoft/blob/master/legacy/README.md, for the release apr21_patches. You will probably have to bootstrap cmake to get the correct verison.

It will fail when it gets to installing root, because it can't build XROOTD (it is missing the header files from openssl, on a personal computer this could probably be fixed by installing libssl-dev). On fishtank, we are just going to disable this module (we expect all of out data to be accessed locally) and at the same time **enable the builtin_fftw3 module**.

I decided to do this by modifying the cmake cache file for root, found at Build/root/CMakeCache.txt by changin the lines:

1. `xrootd:BOOL=ON` to `xrootd:BOOL=OFF`
2. `builtin_xrootd:BOOL=ON` to `builtin_xrootd:BOOL=OFF`
3. `builtin_fftw3:BOOL=OFF` to `builtin_fftw3:BOOL=ON`
4. `fftw3:BOOL=OFF` to `fftw3:BOOL=ON`


#### FairRoot 18.6.5

Modified directions from https://github.com/FairRootGroup/FairRoot

1. `export SIMPATH=FairSoftInstallDirectory`
2. Clone the FairRoot repository `git clone -b v18.6_patches https://github.com/FairRootGroup/FairRoot.git`
3. Create the build directory `mkdir /mnt/analysis/e12014/fair_install/FairRoot/`
4. Run cmake in the build directory `cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/FairRoot /mnt/simulations/attpcroot/fair_install_18.6/src/FairRoot/`
5. Build and install the code in the build directory `make` `make install`

#### HDF5 1.10.4

1. Grab source code `wget https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.10/hdf5-1.10.4/src/hdf5-1.10.4.tar.gz`
2. Untar source `tar -xvf hdf5-1.10.4.tar.gz`
3. Make build directory `mkdir /mnt/analysis/e12014/fair_install/hdf5`
4. Run cmake `cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/hdf5 /mnt/simulations/attpcroot/fair_install_18.6/src/hdf5-1.10.4`
5. Make and install `make && make install`

#### FLANN 1.9.1 (Required for PCL)

1. Grab the source `wget https://github.com/flann-lib/flann/archive/refs/tags/1.9.1.tar.gz`
2. Unpack the source `tar -xzf 1.9.1.tar.gz`
3. Create a build directory `mkdir /mnt/analysis/e12014/fair_install/flann/`
4. Modify flann per issue [#369](https://github.com/flann-lib/flann/issues/369) to build with newer versions of cmake:
```
touch src/cpp/empty.cpp
sed -e '/add_library(flann_cpp SHARED/ s/""/empty.cpp/' \
-e '/add_library(flann SHARED/ s/""/empty.cpp/' \
-i src/cpp/CMakeLists.txt
```
5. In the build directory, run cmake `cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/flann /mnt/simulations/attpcroot/fair_install_18.6/src/flann`
6. `make && make install` (might yell about pyFlann but idc)

#### Eigen3 (Required for PCL)
1. Grab the source `wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz`
2. Untar file `tar -xzf eigen-3.3.9.tar.gz`
3. Make build directory `mkdir /mnt/analysis/e12014/fair_install/eigen3`
4. `cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/eigen /mnt/simulations/attpcroot/fair_install_18.6/src/eigen-3.3.9/`
5. Install `make install`

#### PCL 1.10.1 (Optional dependency)

0. Make sure FLANN, and Eigen are all installed
1. Grab the source code `https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.10.1.tar.gz`
2. Untar source `tar -xvf pcl-1.10.1.tar.gz`
3. Create build directory `mkdir /mnt/analysis/e12014/fair_install/pcl`
4. In build directory, run cmake set to use the boost libraries from fairsoft
```
cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/pcl -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/mnt/simulations/attpcroot/fair_install_18.6/flann -DBOOST_INCLUDEDIR=/mnt/simulations/attpcroot/fair_install_18.6/FairSoft/include -DBOOST_LIBRARYDIR=/mnt/simulations/attpcroot/fair_install_18.6/FairSoft/lib/ -DBoost_NO_SYSTEM_PATHS=ON /mnt/simulations/attpcroot/fair_install_18.6/src/pcl-pcl-1.10.1/
```
5. Build and install the code `make` and `make install`


#### GenFit (Optional dependency)

1. Get the source `git clone https://github.com/Yassid/GenFit.git`
2. Make build directory `mkdir /mnt/analysis/e12014/fair_install/genfit`
3. Run cmake from build directory, making sure it can find our version of root 
```
cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/GenFit -DCMAKE_PREFIX_PATH=/mnt/simulations/attpcroot/fair_install_18.6/FairSoft/ -DBUILD_TESTING=OFF /mnt/simulations/attpcroot/fair_install_18.6/src/GenFit/
```
4. Build and install `make` and `make install`

### HiRAEVT (Optional dependency)

1. Get the source `git clone https://github.com/nscl-hira/HiRAEVT.git`
2. Create build directory `mkdir /mnt/analysis/e12014/fair_install/HiRAEVT`
3. Run cmake in build directory, disabling unpacking modules 
```
cmake -DCMAKE_INSTALL_PREFIX=/mnt/simulations/attpcroot/fair_install_18.6/HiRAEVT -DCMAKE_PREFIX_PATH=/mnt/simulations/attpcroot/fair_install_18.6/FairSoft/ -DBUILD_UNPACKERS=OFF /mnt/simulations/attpcroot/fair_install_18.6/src/HiRAEVT/
```
4. Build and install `make` and `make install`

## Linking against ATTPCROOT
ATTPCROOT can be included in an external project using CMake. When installed (`make install`) ATTPCROOT will export information on its targets (libraries) and dependencies for others to use. By default, the installation path is `build/install` but this can be changed by setting the `CMAKE_INSTALL_PREFIX` variable when building the ATTPCROOT code.

The simplest use involves making sure the install directory sits on your project's `CMAKE_PREFIX_PATH` and using CMake's build in `find_package(ATTPCROOT)` command, and then adding the libraries required with `target_link_libraries()`. A minimally fuctioning CMakeLists.txt for an external project might look something like this:

```
list(APPEND CMAKE_PREFIX_PATH /path/to/ATTPCROOT/install/dir)

find_package(ATTPCROOT REQUIRED)
add_library(yourLib SHARED yourLib.cpp)
target_link_libraries(yourLib PUBLIC ATTPCROOT::AtData)
```
This will build a shared library called `libyourLib.so` and link it against `libAtData.so` and all its dependencies.

# For developers

## Coding conventions

Pull requests submitted should have the code formated in the [ROOT style](https://root.cern/contribute/coding_conventions/). Never include a header file when a forward decleration is sufficient. Only include header files for base classes or classes that are used by value in the class definition. The static analyzer IWYU helps with this, but will ocassionally get things wrong, so use your judgment.

All data members of a class should be private or protected and begin with the letter `f`, followed by a capital letter. All public member functions should begin with a capital letter. Following ROOT's style guide, private data members should be declared first, followed by the private static members, the private methods and the private static methods. Then the protected members and methods, and finally the public methods. Add trivial get or setters directly in the class definition. Add more complex inlines (longer than one line) at the bottom of the .h file. 

Avoid raw c types for anything that might be written to disk (any memeber variable), instead use ROOT defined types like `Int_t` defined in `<Rtypes.h>`. If any changes are made to the memory layout of a class, the version number in the ROOT macro `ClassDef` needs to be incremented. If the class overrides any virtual function, the macro `ClassDefOverride` should be used instead.

An object should always be created in a valid state. Instance variables that are set to the same value in all constructors should be instantiated in the header file. Variables whose initial value can change depending on the constructor called, should be intialized in the constructor's braced-init-list. Prefer delegating constructors to repeating code.

Header files refrencing this code base should be included using quotes i.e. `#include "header.h"`. Header files from dependencies should be included using angle brackets i.e. `#include <Rtypes.h>`. This is the syntax assumed by the custom mapping file we use for running IWYU and will lead to false errors running the linter if deviated from. 

### Object ownership

When we talk about object ownership we mean the right to free the associated resources (in otherwords `delete` an object). ROOT has some weird quirks of ownership, where you may not own the things you think you do. This is particularly true of histograms, TTrees, and TEventLists. Details of ROOT ownership practices can be found [here](https://root.cern.ch/root/htmldoc/guides/users-guide/ObjectOwnership.html). For everything not discussed here we use the following conventions (modified from the [C++ Core Guidlines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-resource):

* Express ownership explicitly through use of smart pointers `std::unique_ptr` and `std::shared_ptr`. This means there should never be an explicit call to `new` or `delete`. The exception to this is ROOT cannot yet create streamer for writing `std::shared_ptr` to disk. In that case, a raw pointer (`T*`) with a comment explaining the intended ownership is acceptible until ROOT fixes that problem.
* Prefer `std::unique_ptr` over `std::shared_ptr` unless ownership must be shared.
* Raw pointers (`T*`) and raw references are non-owning (`T&`). They should never be deleted. 
* Take smart pointers as parameters only to express lifetime semantics (transfer ownership)
* In interfaces (headers), use raw pointers to denote individual objects. Arrays should be represented by a containter (unless it is c-style string).

### Formatting code for pull requests

Before submitting a pull request, reformat the code using `clang-format`. If run from within the repository it will pick up the `.clang-format` file detail the style. This file is also reproduced on the page detailing ROOT the ROOT coding style. If for some reason you need a section of code to not be formatted, you can turn off formatting using comment flags. Formatting of code is turned off with the comment `// clang-format off` and re-enabled with the comment `// clang-format on`. This process can be simplified using the command `git-clang-format` which when given no options will tun `clang-format` on all lines of code that differ between the working directory and HEAD. Detailed documentation is [here](https://github.com/llvm-mirror/clang/blob/master/tools/clang-format/git-clang-format).

Before submitting a pull request run the tests as described [below](#running-tests).

## Adding a class

Classes, for example a new generator, can be added by creating the header and source files. In the library directory the `CMakeLists.txt` file must be edited to add any include directories needed as well as add the source file so CMake knows to compile it. In addition the class should be added the the local `LinkDef.h` file, if needed. Classes should respect the naming convention of `AtName.cxx` for source, and `AtName.h` for headers or the build system might fail. The macros used to generate the ROOT dictionaries for each library, assumes these file extensions.

In general, any class added or modified should try to respect backwards compatibilty. In addition, it should avoid modifying important base classes unless there is a good reason. That is, if you're adding a feature only used in a certain experiment to a certain class you should extend that class rather then modify it.

## Adding a task

Each task class (something that inherits from FairTask) should be primarily responsible for setting up the input and output. The logic of the task should be handled by another class, an instance of which is a member of the task class. The goal here is to seperate the IO from the actualy loogic of the task. This makes it easier for someone linking against ATTPCROOT to use the logic without spinning up an entire instance of FairRoot. 

## Adding a library

Each folder in the root directory names `AtName` will build into a shared object with the name `libAtName` as defined by a `CMakelists.txt` file in each folder. Other folders, (eg macro, icon, parameters, etc), are not built and do not contain a CMakeLists.txt folder. In order for the new library to be built, it must be added as a subdirectory to the global CMakeList.txt in the root directory.

## Running tests

There is a suite of tests located in `macro/tests` to check the unpacking of AT-TPC, SpecMAT, and GADGET data. It also tests the simulation of AT-TPC data. To run the tests, the following pre-requites must be met:

* Generate the required geometry files by running the script `macro/tests/generateGeometry.sh`
* GADGETII tests must be run on fishtank (the data is to large to package with the repository)
* SpecMAT tests require the ROOT file TTreesGETrun_9993.root be placed in the `macro/tests/SpecMAT/data` folder.

To run all tests run the bash script `macro/tests/runAllTest.sh`. To the screen it will print a summary of each test run and the return value. The output of the test scripts run are saved in a text file in each test directory (test.log).

### Coverage of tests

- [ ] Simulation Coverage
  * Generators
    - [ ] AtTPC20MgDecay
	- [ ] AtTPC2Body
	- [ ] AtTPC_Background
	- [ ] AtTPC_d2He
	- [ ] AtTPCFissionGenerator
	- [ ] AtTPCFissionGeneratorv2
	- [x] AtTPCFissionGeneratorv3
	- [ ] AtTPCGammaDummyGenerator
	- [ ] AtTPCIonDecay
	- [x] AtTPCIonGenerator
	- [ ] AtTPCXSManager
	- [ ] AtTPCXSReader
  - [ ] AtAvalanchTask
  - [x] AtClusterizeLineTask
  - [ ] AtClusterizeTask
  - [x] AtPulseLineTask
  - [ ] AtPulseTask
  - [ ] AtSpaceChargeTask
  - [ ] AtTriggerTask
- [x] Unpacking Coverage
  - [x] AtUnpackTask
    - [x] AtHDFUnpacker
    - [x] AtROOTUnpacker
    - [x] AtGRAWUnpacker
- [ ] Reconstruction Coverage
  - [ ] AtAuxFilterTask
  - [x] AtDataReductionTask
  - [x] AtFilterTask
    - [x] AtFilterSubtraction
	- [ ] AtFilterCalibrate
	- [ ] AtTrapezoidFilter
  - [ ] AtFitterTask
  - [ ] AtLinkDAQtask
  - [ ] AtPRAtask 
    - [ ] AtLmedsMod
	- [ ] AtMlesacMod
	- [ ] AtPRA
    - [ ] AtRansacMod
	- [ ] AtTrackFinderHC
  - [x] AtPSAtask
    - [ ] AtCalibration
	- [ ] AtPSAFilter
	- [ ] AtPSAFull
	- [ ] AtPSAProto
	- [ ] AtPSAProtoFull
    - [x] AtPSASimple2
	- [ ] AtPSASimple
  - [x] AtRansacTask
    - [x] AtRansac
  - [ ] AtTrackFinderHCTask
  
# Creating geometry files

### Adding materials

Each material that can be used in a detector is defined in the global file `geometry/media.geo`. If you would like to add any new materials, it can be done in that file. The header has detailed instructions for adding new materials.

### Creating detectors

There are a number of scripts in geometry/ for building detector geometries. Each of these output two root files that are then passed to the simulation code. In these script files, all of the materials of the detector can be changed including the gas in the attpc, and window material. If the ion chamber or some other detector is present those can also be added.

The file ATTPC_v1_2.C has the basic geometry for the full sized TPC. The file ATTPC_Proto_v1_0.C ha the basic geometry for the pATTPC. Both of these files are pretty self explanitory if opened.

To generate the root files that will be used for the physics simulation just run the macro `root ATTPC_v1_2.C`. The simulation macros will look in the `geometry/` folder for these files.

# Running simulations

### Generating physics

Each "event" is split into two phases, the beam phase and the reaction phase. Internally these are different events. What that means is that events 0 and 1 in the code constitute a full, physical beam induced event. Likewise events 2 and 3 are the next beam induced event, etc.

Particles are tracked by an internal counter fTrackID. fTrackID = 0 is the beam. The value these have determine how the TPC detector handles adding the track.

The folder `macro/Simulation` holds all of the ROOT macros for physics simulations. These produce two ROOT files, one is the actual output file that holds track information. The other is a parameter file generated by the process. These simulations can be run using either the Geant4 or Geant3 monte-carlo engine.

For every simulation, the detector must be created, and a geometry file set. A `FairPrimaryGenerator` must also exist. *This is responsible for managing all of the other generators.(True?)*

Typically, an `ATTPCIonGenerator` is used to create the beam. It creates an ion within some beamspot and adds a track to the primary generator. It also generates a random number between 0 and the nominal beam energy (or some other specified value) that is stored in the global instance of `ATVertexPropagator` (gATVP). When the beam particle (TrackID = 0) looses energy equal to this random number, the simulation stops the track. Due to the tracking of the TrackID and `aATVP`'s tracking of the current track number, tracks from the beam/reaction generator are only added when it is the right type of event (even = beam, odd = reaction).

#### ATFissionGeneratorV3

This class takes two input files. One is a space seperated file that contains a list of every ion used by that fission simulation so they can be registered for the run. The other is root file containing a tree with the simulation data for the fission fragments int he CM frame. It has the structure:

trEvents ->
 - nTracks /I
 - Aout[nTracks] /I
 - Zout[nTracks] /I
 - pX[nTracks] /D in units MeV/c
 - pY[nTracks] /D in units MeV/c
 - pZ[nTracks] /D in units MeV/c
 - pT[nTracks] /D in units MeV/c


During a run, the generator will boost the fragments before generating the particles to the beam momentum. It pulls the beam momentum from the global vertex propogator. 

There is a macro for generating these input files in [macro/Simulation/E12014](macro/Simulation/E12014).

# Running Analysis

When running analysis in a macro, an instance of FairRunAna is created and tasks added to it. Each task will take in an event, process it, and then pass the event to the next task. Each task can save the intermediary step if desired by setting the boolean flag kIsPersistant.

### Unpacking Tasks
These tasks are responsible for taking raw data in and saving it in an ATRawEvent class.

#### HFD5ParserTask
TODO

### Pulse Shape Analysis
These tasks are responsible for taking an ATRawEvent and saving it as an ATEvent. That is, it takes the waveforms from the pads and process it into an array of ATHits which are the location of charge deposition in the TPC in space.

All the different PSA methods are handled by the same task, ATPSAtask. This task can also handle gain and jitter calibration through the class ATCalibration. When constructing the task, a pointer must be passed to a class that derives from the virtual class ATPSA. This derived class handles the logic of the task and the managment of whatever parameters are necessary. Below contains documentation on the various implemented PSA methods, including discriptions of their parameters.

#### ATPSASimple
TODO
#### ATPSASimple2
TODO
#### ATPSAFull
TODO
#### ATPSASFilter
TODO
#### ATPSAProtoFull
TODO
#### ATPSAProto
TODO

# Notes on design
ATTPCROOT is task based and there are two main input streams of data. One is from simulations, the other experimental data. The two streams come together at the level of waveforms on the pads, either simulated or real. From then on the analysis is identiacal.

## Simulation Tasks

### FairRunSim
Physics (Primary Generators/physics lists) -> MC Tracks

### FairRunAna

#### ATClusterize task
MC Tracks -> e- locations along track (TClonesArray of ATSimulationPoints)

#### ATPulseTask
e- locations along track (TClonesArray of ATSimulationPoints) -> waveform on pads (TClonesArray of ATRawEvent)

## Data Tasks

### FairRunAna

#### HDFParserTask
HDF5 file -> waveform on pads (TClonesArray of ATRawEvent)
Output is named: "ATRawEvent"
## Analysis Tasks

### FairRunAna

#### ATPSATask
There are many diffrent options for this task depending on how you want to treat the wavefrom (the standard ATTPC version is ATPSASimple2), but in general:

Waveform on pads (TClonesArray of ATRawEvent) -> record of hits, 3D position and charge in tpc, (ATHit) on pad (TClonesArray of ATEvent)
Output is named: "ATEventH"

This task uses the parameters in the ATTPC .par files. In particular this loads (bolded are used by PSASimple):
* PadPlaneX (?)
* PadSizeX (?)
* PadPlaneZ (?)
* PadSizeZ (?)
* PadRows (?)
* PadLayers (?)
* **NumTbs (Number of time buckets)**
* **TBTTime (Sampling rate in ns)**
* **DriftVelocity (cm/us)**
* MaxDriftLength (Length of TPC)
* BField
* EField
* TiltAng
* TB0 (Depricated: time bucket refrence for micromegas)
* EntTB (position in time buckets for detector entrance)


#### ATPRATask
Has options for RANSAC, Hierarchical Clustering, and Houghs algorithms for track finding.

record of hits (TClonesArray of AtEvent) -> record of reconstructed tracks (ATTrack) (TClonesArray of ATPatternEvent)

