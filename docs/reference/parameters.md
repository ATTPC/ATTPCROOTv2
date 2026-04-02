# Parameter Files

ATTPCROOT uses FairRoot's parameter system to manage experiment-specific detector configuration. The main container class is `AtDigiPar`, defined in `AtParameter/AtDigiPar.h`.

## Loading Parameters in a Macro

```cpp
FairRuntimeDb *rtdb = run->GetRuntimeDb();
FairParAsciiFileIo *parIo = new FairParAsciiFileIo();
parIo->open("parameters/ATTPC.myexperiment.par", "in");
rtdb->setFirstInput(parIo);
rtdb->getContainer("AtDigiPar");
```

Parameter files live in `parameters/`. Each file typically contains an `[AtDigiPar]` section followed by key-value pairs. Pick the file that matches the experiment or create a new one by copying a current experiment file such as `parameters/ATTPC.e20020.par`.

## AtDigiPar Parameters

`AtDigiPar::getParams()` requires the following keys:

### Electromagnetic Fields

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `EField` | Double_t | V/m | Longitudinal drift electric field |
| `BField` | Double_t | T | Longitudinal magnetic field |

### Detector Geometry

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `TBEntrance` | Int_t | time buckets | Beam entrance position at detector entrance |
| `ZPadPlane` | Double_t | mm | Position of the micromegas pad plane (detector length) |

### Gas Physics

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `DriftVelocity` | Double_t | cm/µs | Electron drift velocity |
| `EIonize` | Double_t | eV | Effective ionization energy of the fill gas |
| `Fano` | Double_t | — | Fano factor of the fill gas |
| `CoefL` | Double_t | cm²/µs | Longitudinal diffusion coefficient |
| `CoefT` | Double_t | cm²/µs | Transverse diffusion coefficient |
| `GasPressure` | Double_t | torr | Gas pressure |
| `Density` | Double_t | kg/m³ | Gas density |

### Electronics

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `SamplingRate` | Int_t | MHz | GET electronics sampling frequency |
| `GETGain` | Double_t | fC | Gain from GET electronics |
| `PeakingTime` | Int_t | ns | Electronic response peaking time |
| `Gain` | Double_t | — | Average micromegas amplification factor |

## Accessing Parameters in Code

Tasks retrieve `AtDigiPar` from `FairRuntimeDb` in their `Init()`:

```cpp
auto *rtdb = FairRuntimeDb::instance();
fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
```

Then in `Exec()`:

```cpp
double driftVel = fPar->GetDriftVelocity(); // cm/µs
double bField   = fPar->GetBField();         // T
```

## Example `AtDigiPar` Block

```text
[AtDigiPar]
BField:Double_t         2.0
EField:Double_t         5000
TBEntrance:Int_t        445
ZPadPlane:Double_t      1000.0
EIonize:Double_t        15.603
Fano:Double_t           0.24
CoefL:Double_t          0.0003
CoefT:Double_t          0.0003
GasPressure:Double_t    600
Density:Double_t        0.197
DriftVelocity:Double_t  0.87
Gain:Double_t           1000
SamplingRate:Int_t      3
GETGain:Double_t        1000
PeakingTime:Int_t       720
```

Copy a complete file from `parameters/` and edit the values for the target experiment.
