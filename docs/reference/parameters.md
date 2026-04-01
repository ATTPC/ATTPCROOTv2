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

Parameter files live in `parameters/`. Each file typically contains an `[AtDigiPar]` section followed by key-value pairs. Pick the file that matches the experiment or create a new one by copying an existing one.

## AtDigiPar Parameters

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
| `DriftLength` | Double_t | mm | Full drift length (AT-TPC: 1000, prototype: 500) |

### Gas Physics

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `DriftVelocity` | Double_t | cm/µs | Electron drift velocity |
| `EIonize` | Double_t | eV | Effective ionization energy of the fill gas |
| `Fano` | Double_t | — | Fano factor of the fill gas |
| `CoefL` | Double_t | cm^−½ | Longitudinal diffusion coefficient |
| `CoefT` | Double_t | cm^−½ | Transverse diffusion coefficient |
| `GasPressure` | Double_t | torr | Gas pressure |
| `Density` | Double_t | kg/m³ | Gas density |

### Electronics

| Parameter | Type | Unit | Description |
|-----------|------|------|-------------|
| `SamplingRate` | Int_t | MHz | GET electronics sampling frequency |
| `GETGain` | Double_t | fC | Gain from GET electronics |
| `PeakingTime` | Int_t | ns | Electronic response peaking time |
| `Gain` | Double_t | — | Average micromegas amplification factor |
| `NumTbs` | Int_t | — | Number of time buckets |

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

## Example Parameter File Entry

```
[AtDigiPar]
EField:Double_t         5000    # V/m
BField:Double_t            2    # Tesla
TBEntrance:Int_t         280    # time bucket at detector entrance
ZPadPlane:Double_t      1000    # mm
DriftVelocity:Double_t  5.00    # cm/us
EIonize:Double_t       15.603   # eV (He+CO2 example)
Fano:Double_t           0.22
CoefL:Double_t          0.025   # cm^-0.5
CoefT:Double_t          0.010   # cm^-0.5
GasPressure:Double_t     100    # torr
Density:Double_t       0.0738   # kg/m3
SamplingRate:Int_t      12.5    # MHz
Gain:Double_t          100.0
NumTbs:Int_t             512
PeakingTime:Int_t        117    # ns
```

See `parameters/AT.parameters.par` for a complete reference example and `parameters/` for all experiment-specific files.
