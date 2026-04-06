# Plan: AtSimpleSimulation Validation — Elastic Scatter with B-Field

## Context

`AtSimpleSimulation` / `AtPropagator` (Lorentz-force RK4) have never been validated against full Geant4 physics. The validation scenario is proton-on-He elastic scattering with a 2 T solenoid, which gives helical tracks with analytically-known Larmor radii and is straightforward to generate with the existing `AtTPC2Body` + `AtTPCIonGenerator` pair.

The macros live in `macro/Simulation/protonBragg/` (to be **renamed** to `macro/Simulation/AtSimValidation/`).

**Why the single-generator approach failed:**
`AtTPCIonGenerator` alone produces identical-momentum particles every event with no vertex variation — no statistical content. The simulation pipeline alternates beam events (even) and reaction events (odd). On beam events `AtTPCIonGenerator` fires; on reaction events `AtTPC2Body` fires. Without a reaction generator registered, reaction-phase events corrupt `AtVertexPropagator` state and can crash VMC initialisation.

**Why two macros per scenario:**
Geant4 uses `FairRunSim`; `AtSimpleSimulation` uses `FairRunAna` + `AtTestSimulation`. They cannot share a run. Each scenario therefore has a Geant4 macro and a mirrored SimpleSim macro that uses identical generator parameters and the same `gRandom` seed, so both process the same sequence of random CMS angles and beam-depth variations.

---

## Physics Being Simulated

**Reaction:** p + ⁴He → p + ⁴He (elastic)  
**Beam:** proton, 1 MeV kinetic energy (p = 43.33 MeV/c = 0.04333 GeV/c along Z)  
**Gas / target:** He-1bar, density 1.664 × 10⁻⁴ g/cm³, serves as both medium and target nuclei  
**Geometry:** ATTPC cylinder, r = 25 cm, z = 0–100 cm  
**B-field:** 2 T along Z (solenoid), applied in both simulations  
**Energy-loss model for SimpleSim:** `AtELossCATIMA` with He material  

With 2 T the proton Larmor radius is r_p ≈ 72 mm and the He-4 recoil radius r_He ≈ 36 mm (at full beam momentum; shrinks as particles lose energy). Both tracks form visible helical arcs within the drift volume before stopping.

**Bragg information** is retained in summary plots: dE/dx vs Z per track provides the Bragg curve for each product.

---

## Folder Structure

```
macro/Simulation/AtSimValidation/   ← renamed from protonBragg/
├── geant4_fixed.C                  ← Geant4, fixed CMS angle (thetaCms parameter)
├── simpleSim_fixed.C               ← SimpleSim, same fixed angle, same seed
├── geant4_kinematic.C              ← Geant4, full kinematic curve (0–180° CMS)
├── simpleSim_kinematic.C           ← SimpleSim, full kinematic curve, same seed
├── compareFixed.C                  ← 3D track overlay + Bragg curves, fixed angle
├── compareKinematic.C              ← kinematic locus + Bragg curves, full range
└── data/
    ├── geant4_fixed.root
    ├── simpleSim_fixed.root
    ├── geant4_kinematic.root
    └── simpleSim_kinematic.root
```

The old files (`runGeant4_proton.C`, `simpleSim_Bfield.C`, `compareSimVsGeant.C`) are removed; their content is superseded.

---

## Generator Setup (canonical, shared across all four simulation macros)

Follows `C16_pp_sim.C` exactly.

```
Beam:    Z=1, A=1, Q=0, px=0, py=0, pz=0.04333 GeV/c/nucleon
         Bmass=0.938272 GeV/c², NomEnergy=1.0 MeV
         ionGen->SetSpotRadius(0, -100, 0)
         ionGen->SetDoReaction(kTRUE)   // beam drives AtVertexPropagator

Target:  ⁴He at rest: Z=2, A=4, mass=4.00260 amu, ExE=0

Product 1 (scattered proton):  Z=1, A=1, mass=1.00728 amu, ExE=0
Product 2 (recoil He-4):       Z=2, A=4, mass=4.00260 amu, ExE=0

mult = 4  (beam + target + 2 products — required by AtTPC2Body)
ResEner = 1.0   // MeV, nominal beam energy
```

`AtTPCIonGenerator` fires on beam events and uses `AtVertexPropagator` to record the beam stopping depth (the reaction vertex). `AtTPC2Body` fires on reaction events, reads the vertex and residual beam momentum, and adds the two products.

For SimpleSim, `AtTestSimulation::Exec()` calls `fPrimGen->GenerateEvent(&fCollector)` every event. The same even/odd alternation applies because `AtVertexPropagator` is a singleton: beam events deposit the beam track into the collector and update the vertex; reaction events deposit the two reaction products. `AtSimpleSimulation` propagates whichever particles appear in the collector each event.

**Same ground truth / seed:**  
Both the Geant4 macro and the mirrored SimpleSim macro call `gRandom->SetSeed(42)` before `run->Init()`. This produces the same sequence of:
- random CMS angles (from AtTPC2Body's uniform cos-theta sampling)
- `fRndELoss` values that determine beam stopping depth (from AtTPCIonGenerator)

Because CATIMA and Geant4 EMSTD agree closely on proton energy loss in He, the actual vertex depths will be similar, giving visually compatible track patterns.

---

## Macro 1 & 2: Fixed Angle (`geant4_fixed.C` / `simpleSim_fixed.C`)

**Signature:** `(Double_t thetaCms = 45.0, Int_t nEvents = 100, UInt_t seed = 42)`

**Key parameters in AtTPC2Body:**
```cpp
Double_t ThetaMinCMS = thetaCms;
Double_t ThetaMaxCMS = thetaCms;   // fixed angle: min == max
```

**B-field (Geant4):** `AtConstField` with (0, 0, 20) kG (= 2 T); region covers full detector.

**B-field (SimpleSim):** `sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0))`

**Output:** `./data/geant4_fixed.root` / `./data/simpleSim_fixed.root`

---

## Macro 3 & 4: Kinematic Curve (`geant4_kinematic.C` / `simpleSim_kinematic.C`)

**Signature:** `(Int_t nEvents = 1000, UInt_t seed = 42)`

**AtTPC2Body:** `ThetaMinCMS = 0`, `ThetaMaxCMS = 180` (full range, cos-theta uniform).

Everything else identical to macros 1 & 2.

**Output:** `./data/geant4_kinematic.root` / `./data/simpleSim_kinematic.root`

---

## Comparison Macro 1: `compareFixed.C`

Reads `geant4_fixed.root` + `simpleSim_fixed.root`.

**Plots:**

| # | Title | Description |
|---|-------|-------------|
| 1 | 3D track overlay | `TGraph2D` of (x, y, z) for each track; Geant4 in blue, SimpleSim in red; separate panels for proton and He-4 products; should visually coincide |
| 2 | XY projection | `TH2D` of all hit positions; circles expected from helical projection |
| 3 | XZ projection | Shows helical pitch |
| 4 | Bragg curve — proton | mean dE/dx [MeV/mm] vs Z [mm] from both simulations |
| 5 | Bragg curve — He-4 | same for recoil He-4 |
| 6 | Track length | distribution of total path length per track |

For the 3D overlay (plot 1): iterate over events, group MCPoints by `GetTrackID()`, draw a `TPolyLine3D` for each product track. Up to 20 events overlaid.

---

## Comparison Macro 2: `compareKinematic.C`

Reads `geant4_kinematic.root` + `simpleSim_kinematic.root`.

**Plots:**

| # | Title | Description |
|---|-------|-------------|
| 1 | Kinematic locus | p_t [MeV/c] vs p_z [MeV/c] for the proton product; Geant4 vs SimpleSim should trace the same curve |
| 2 | Kinematic locus | same for He-4 recoil |
| 3 | Bragg curve — proton | mean dE/dx vs Z, all events averaged |
| 4 | Bragg curve — He-4 | same |
| 5 | Range distributions | stopping Z for each product, both simulations |
| 6 | XY projection | all hits, full kinematic range |

Kinematic locus: use first MCPoint of each track as the initial momentum. In the limit of no energy loss at vertex, both simulations should produce the same two-body kinematic curve.

---

## Energy-Loss Model for SimpleSim

```cpp
// He gas at 1 bar
constexpr double He_density = 1.664e-4; // g/cm³ at 20°C

// Proton in He
auto eloss_p = std::make_shared<AtTools::AtELossCATIMA>(
   He_density,
   std::vector<std::tuple<int,int,int>>{{4, 2, 1}} // He-4: {A, Z, stoich}
);
eloss_p->SetProjectile(1, 1, 1.007276); // proton: A, Z, mass_amu
sim->AddModel(1, 1, eloss_p);

// He-4 in He
auto eloss_he = std::make_shared<AtTools::AtELossCATIMA>(
   He_density,
   std::vector<std::tuple<int,int,int>>{{4, 2, 1}}
);
eloss_he->SetProjectile(4, 2, 4.002602); // He-4: A, Z, mass_amu
sim->AddModel(2, 4, eloss_he);
```

---

## Geant4 Fix (present in both Geant4 macros)

The crash from the previous `runGeant4_proton.C` was caused by calling `AtVertexPropagator::Instance()` before `FairRunSim` was constructed. Fix: **remove the explicit `AtVertexPropagator::Instance()` call entirely** (follow `C16_pp_sim.C` — `AtTPCIonGenerator` calls it internally). `SetDoReaction(kTRUE)` is used (default) so the reaction generator can fire.

---

## Verification

1. **Rename folder**, remove old files, create new ones
2. Build: no code changes — all classes already compiled
3. **Run fixed-angle pair:**
   ```bash
   cd macro/Simulation/AtSimValidation
   source ../../../build/config.sh
   root -l -q 'geant4_fixed.C(45., 100)'
   root -l -q 'simpleSim_fixed.C(45., 100)'
   root -l -q 'compareFixed.C'
   ```
4. **Run kinematic pair:**
   ```bash
   root -l -q 'geant4_kinematic.C(1000)'
   root -l -q 'simpleSim_kinematic.C(1000)'
   root -l -q 'compareKinematic.C'
   ```
5. **Pass criteria:**
   - 3D track overlays from fixed-angle run show Geant4 (blue) and SimpleSim (red) helices coinciding within the expected energy-straggling spread
   - Kinematic locus from both simulations traces the same two-body kinematic curve
   - Bragg curves from both methods agree in peak position ± ~1 cm

---

## Key Files

| File | Role |
|------|------|
| `macro/Simulation/AtSimValidation/geant4_fixed.C` | new |
| `macro/Simulation/AtSimValidation/simpleSim_fixed.C` | new |
| `macro/Simulation/AtSimValidation/geant4_kinematic.C` | new |
| `macro/Simulation/AtSimValidation/simpleSim_kinematic.C` | new |
| `macro/Simulation/AtSimValidation/compareFixed.C` | new |
| `macro/Simulation/AtSimValidation/compareKinematic.C` | new |
| `macro/Simulation/ATTPC/16C_pp/C16_pp_sim.C` | canonical generator pattern |
| `AtGenerators/AtTPC2Body.h/.cxx` | reaction generator |
| `AtGenerators/AtTPCIonGenerator.h/.cxx` | beam generator |
| `AtDigitization/AtTestSimulation.cxx` | SimpleSim FairTask wrapper |
| `AtTools/AtELossCATIMA.h` | energy-loss model |
