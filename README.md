ATTPCROOT is a ROOT-based (root.cern.ch) framework to analyze data of the ATTPC detector (Active Target Time Projection Chamber) and the p-ATTPC (Prototype). For reference http://ribf.riken.jp/ARIS2014/slide/files/Jun2/Par2B06Bazin-final.pdf. The detector is based at the NSCL but experiments are performed at other facilities as well (Notre Dame, TRIUMF, Argonne...).
 
The framework allows the end user to unpack and analyze the data, as well as perform realistic simulations based on a Virtual Monte Carlo (VMC) package. The framework needs external libraries (FairSoft and FairRoot https://fairroot.gsi.de/) to be compiled and executed, which are developed by other groups and not directly supported by the AT-TPC group. Please refer to their forums for installation issues of these packages.

There was also some documentation written as part of Alexander Carl's 2019 thesis which can be found [here](https://publications.nscl.msu.edu/thesis/%20Carls_2019_6009.pdf)

# Installation on NSCL/FRIB cluster

ROOT and FairROOT are already installed on the system. The modules and their prerequisites just need to be loaded with the commands:
```
module purge
module load fairroot/18.6.3
module load xerces/3.2.3
```

To install ATTPCROOT checkout the repository from github and then create a folder in the repository to build the code and cd into it. From this directory you can call CMake to configure the build. Then you can build the source. You will need to tell CMake where you installed things besides FairRoot and FairSoft by setting the DCMAKE_PREFIX_PATH:
```
git clone https://github.com/ATTPC/ATTPCROOTv2.git
cd ATTPCROOTv2
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=/mnt/misc/sw/x86_64/Debian/10/fairroot/18.6.3/ ../
make -j 4
```

After the source builds, you should be good to go. The CMake script will output a configuration file in the build directory called `config.sh`. This script should be sourced before building or running any macros. For those working on the NSCL fishtank computers, the script `env.sh` can be sourced instead. This will also add to your `$PATH` the executables needed to format the code using `clang-format` described below.

# For developers

### Formatting code for pull requests
Pull requests submitted should have the code formated in the [ROOT style](https://root.cern/contribute/coding_conventions/). Never include a header file when a forward decleration is sufficient. Only include header files for base classes or classes that are used by value in the class definition.

Before submitting a pull request, reformat the code using `clang-format`. If run from within the repository it will pick up the `.clang-format` file detail the style. This file is also reproduced on the page detailing ROOT the ROOT coding style. If for some reason you need a section of code to not be formatted, you can turn off formatting using comment flags. Formatting of code is turned off with the comment `// clang-format off` and re-enabled with the comment `// clang-format on`. This process can be simplified using the command `git-clang-format` which when given no options will tun `clang-format` on all lines of code that differ between the working directory and HEAD. Detailed documentation is [here](https://github.com/llvm-mirror/clang/blob/master/tools/clang-format/git-clang-format).

All data members of a class should be private or protected and begin with the letter `f`, followed by a capital letter. All member functions should begin with a capital letter. Private data members should be declared first, followed by the private static members, the private methods and the private static methods. Then the protected members and methods and finally the public methods. 

Avoid raw c types for anything that might be written to disk (any memeber variable), instead use ROOT defined types like `Int_t` defined in `Rtypes.h`. If any changes are made to the memory layout of a class, the version number in the ROOT macro `ClassDef` needs to be incremented. If the class overrides any virtual function, the macro `ClassDefOverride` should be used instead.

### Adding a class

Classes, for example a new generator, can be added by created the header and source files. In that directory the `CMakeLists.txt` file must be edited to add any include directories needed as well as add the source file so the make file knows to compile it. In addition the class should be added the the local `GenLinkDef.h` file, if needed. In addition, a symbolic link should be created to the new header in the `include` directory. Classes should respect the naming convention of `AtName.cxx` for source, and `AtName.h` for headers. The FairRoot macro used to generate the ROOT dictionaries for each library, assumes these file extensions.

In general, any class added or modified should try to respect backwards compatibilty. In addition, it should avoid modifying important base classes unless there is a good reason. That is, if you're adding a feature only used in a certain experiment to a certain class you should extend that class rather then modify it.

### Adding a task

Each task class (something that inherits from FairTask) should be primarily responsible for setting up the input and output. The logic of the task should be handled by another class, an instance of which is a member of the task class. When adding new features or options to a task the base logic class should be extended rather then modified.

### Adding a library

Each folder in the root directory names `AtName` will build into a shared object with the name `libAtName` as defined by a `CMakelists.txt` file in each folder. Other folders, (eg macro, icon, parameters, etc), are not built and do not contain a CMakeLists.txt folder. In order for the new library to be built, it must be added as a subdirectory to the global CMakeList.txt in the root directory.



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

