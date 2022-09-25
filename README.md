ATTPCROOT is a ROOT-based (root.cern.ch) framework to analyze data of active target detectors including the ATTPC (Active Target Time Projection Chamber) detector and the p-ATTPC (Prototype). For reference http://ribf.riken.jp/ARIS2014/slide/files/Jun2/Par2B06Bazin-final.pdf. The detector is based at the NSCL but experiments are performed at other facilities as well (Notre Dame, TRIUMF, Argonne...).
 
The framework allows the end user to unpack and analyze the data, as well as perform realistic simulations based on a Virtual Monte Carlo (VMC) package. The framework needs external libraries (FairSoft and FairRoot https://fairroot.gsi.de/) to be compiled and executed, which are developed by other groups and not directly supported by the AT-TPC group. Please refer to their forums for installation issues of these packages.

# Installation and Use

For installation directions see the wiki: https://github.com/ATTPC/ATTPCROOTv2/wiki/Installation

For documentation on the use of the code see the wiki: https://github.com/ATTPC/ATTPCROOTv2/wiki/Accessing-documentation

# Old documentation to be ported to wiki and updated

## Running Analysis

When running analysis in a macro, an instance of FairRunAna is created and tasks added to it. Each task will take in an event, process it, and then pass the event to the next task. Each task can save the intermediary step if desired by setting the boolean flag kIsPersistant.

### Unpacking Tasks
These tasks are responsible for taking raw data in and saving it in an ATRawEvent class.

### Pulse Shape Analysis
These tasks are responsible for taking an ATRawEvent and saving it as an ATEvent. That is, it takes the waveforms from the pads and process it into an array of ATHits which are the location of charge deposition in the TPC in space.

All the different PSA methods are handled by the same task, ATPSAtask. This task can also handle gain and jitter calibration through the class ATCalibration. When constructing the task, a pointer must be passed to a class that derives from the virtual class ATPSA. This derived class handles the logic of the task and the managment of whatever parameters are necessary. Below contains documentation on the various implemented PSA methods, including discriptions of their parameters.


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
* **NumTbs (Number of time buckets)**
* **TBTTime (Sampling rate in ns)**
* **DriftVelocity (cm/us)**
* BField
* EField
* EntTB (position in time buckets for detector entrance)


#### ATPRATask
Has options for RANSAC, Hierarchical Clustering, and Houghs algorithms for track finding.

record of hits (TClonesArray of AtEvent) -> record of reconstructed tracks (ATTrack) (TClonesArray of ATPatternEvent)

