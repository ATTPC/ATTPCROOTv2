ATTPCROOT is a ROOT-based (root.cern.ch) framework to analyze data of the ATTPC detector (Active Target Time Projection Chamber) and the p-ATTPC (Prototype). For reference http://ribf.riken.jp/ARIS2014/slide/files/Jun2/Par2B06Bazin-final.pdf. The detector is based at the NSCL but experiments are performed at other facilities as well (Notre Dame, TRIUMF, Argonne...).
 
The framework allows the end user to unpack and analyze the data, as well as perform realistic simulations based on a Virtual Monte Carlo (VMC) package. The framework needs external libraries (FairSoft and FairRoot https://fairroot.gsi.de/) to be compiled and executed, which are developed by other groups and not directly supported by the AT-TPC group. Please refer to their forums for installation issues of these packages.

There was also some documentation written as part of Alexander Carl's 2019 thesis which can be found [here](https://publications.nscl.msu.edu/thesis/%20Carls_2019_6009.pdf)

# Installation on NSCL/FRIB cluster

ROOT and FairROOT are already installed on the system. The modules and their prerequisites just need to be loaded with the commands:
```
module load gnu/gcc/6.4
module load fairroot
```

To install ATTPCROOT checkout the repository from github and then create a folder in the repository to build the code and cd into it. From this directory you can call CMake to configure the build. Then you can build the source
```
git clone https://github.com/ATTPC/ATTPCROOTv2.git
cd ATTPCROOTv2
mkdir build
cd build
cmake ../
make -j 4
```

After the source builds, you should be good to go.

# For developers

### Formatting code for pull requests
Pull requests submitted should have the code formated in the linux stlye with tabs as 4 spaces instead of 8. The GNU program `indent` can be used to easily reformat code before commiting changes. If called from the root directory as `indent [files]`, it will read the local profile `.indent.pro`. If called elsewhere, the proper command line arguments need to be passed `indent -linux -i4 -pmt [files]`. If there is a region of code you want to make sure is not reformatted by `indent` you can use control comments. Formatting of code is turned off with the comment `// *INDENT-OFF*` and re-enabled with the comment `// *INDENT-ON*`.

To the extent possible, keep `#include` statements in source files. In a header file, a forward decleration is always preferable to a `#include`, if possible.

If any changes are made to the memory layout of a class, the version number in the ROOT macro `ClassDef` needs to be incremented. For members of a class, prefer ROOT types (ie Double_t) over native types.

### Adding a class

Classes, for example a new generator, can be added by created the header and source files. In that directory the `CMakeLists.txt` file must be edited to add any include directories needed as well as add the source file so the make file knows to compile it. In addition the class should be added the the local `GenLinkDef.h` file, if needed. In addition, a symbolic link should be created to the new header in the `include` directory.

In general, any class added or modified should try to respect backwards compatibilty. In addition, it should avoid modifying important base classes unless there is a good reason. That is, if you're adding a feature only used in a certain experiment to a certain class you should extend that class rather then modify it.

### Expanding on tasks

Each task class (something that inherits from FairTask) should be primarily responsible for setting up the input and output. The logic of the task should be handled by another class, and instance of which is a member of the task class. When adding new features or options to a task the base logic class should be extended instead of modified.

For example, the task responsible for pulse shape analysis is `ATPSATask`. It contains a member `fPSA` of type `ATPSA`.Each PSA method extends `ATPSA` and when ATPSATask is created, it should be passed a pointer to an instance of a class derived from `ATPSA`. This sets the behavior of the task and holds all of the necessary parameters and flags.


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

