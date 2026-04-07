# Migrating a Simulation Macro to SimpleSim

This guide shows how to convert an existing Geant4/VMC simulation macro to use SimpleSim transport instead. The macro structure, generator setup, and detector configuration stay the same.

## What stays the same

Keep these parts of your Geant4 macro unchanged:

- `FairRunSim` setup (output file, parameter files, random seed)
- Detector modules (`AtCave`, `AtTpc`)
- Geometry files
- Magnetic field configuration on `FairRunSim`
- Generator chain (`FairPrimaryGenerator`, `AtTPCIonGenerator`, `AtTPC2Body`, etc.)
- Downstream digitization tasks (`AtClusterizeTask`, `AtPulseTask`)

## What changes

You replace the transport hookup. In a Geant4 macro, the generator connects directly to the run:

```cpp
run->SetGenerator(primGen);
```

In a SimpleSim macro, the run gets a dummy generator for the event loop, and the real generator is passed to the SimpleSim task:

```cpp
// Give FairRunSim a dummy generator to drive the event loop
run->SetGenerator(new FairPrimaryGenerator());

// Build SimpleSim -- uses the geometry already loaded by FairRunSim
auto sim = std::make_unique<AtSimpleSimulation>();

// Register energy-loss models for every particle species
auto carbonModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, gasMaterial);
carbonModel->SetProjectile(16, 6, 16.014701);
sim->AddModel(6, 16, carbonModel, 16.014701);

auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, gasMaterial);
protonModel->SetProjectile(1, 1, 1.0078250322);
sim->AddModel(1, 1, protonModel, 1.0078250322);

// Set the magnetic field (in Tesla)
sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));

// Create the task, connect the generator and detector
auto *simTask = new AtSimpleSimulationGeneratorTask(std::move(sim));
simTask->SetPrimaryGenerator(primGen);
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

## Required configuration

### Energy-loss models

Every particle species that will be transported must have a registered energy-loss model. Call `sim->AddModel(Z, A, model)` for each species. If a particle has no model, the simulation terminates with a fatal error.

Available model types:
- `AtTools::AtELossCATIMA` -- CATIMA-based energy loss (recommended)
- `AtTools::AtELossTable` -- SRIM table lookup

See [energy-loss.md](energy-loss.md) for details.

### Detector coupling

`SetDetector(tpc)` connects SimpleSim to the `AtTpc` detector so that steps are processed through the same hit-recording and reaction-trigger logic used by Geant4. This is required for correct output.

### Magnetic field

If the experiment uses a magnetic field, set it on the `AtSimpleSimulation` instance:

```cpp
sim->SetMagneticField(ROOT::Math::XYZVector(Bx, By, Bz));  // Tesla
```

This enables curved-track propagation via RK4. Without a field, particles propagate in straight lines.

### Geometry

`AtSimpleSimulation()` (default constructor) automatically uses the geometry that `FairRunSim` loads. No separate geometry file is needed. If you need to use a standalone geometry file (e.g., for testing outside FairRunSim), pass it to the constructor:

```cpp
auto sim = std::make_unique<AtSimpleSimulation>("path/to/geomanager.root");
```

## Replay mode

`AtSimpleSimulationReplayTask` re-transports primary tracks from a prior Geant4 run through SimpleSim. This enables direct A/B comparison with identical kinematics:

```cpp
auto *simTask = new AtSimpleSimulationReplayTask(std::move(sim));
simTask->SetPrimaryTrackSource("geant4_output.root");  // must have MCTrack branch on "cbmsim" tree
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

The source file must contain a `cbmsim` TTree with an `MCTrack` branch from a prior Geant4 run.

## Validation macros

Working examples are in `macro/Simulation/AtSimValidation/`:

- `simpleSim_fixed.C` / `geant4_fixed.C` -- fixed-angle comparison
- `simpleSim_kinematic.C` / `geant4_kinematic.C` -- full kinematic sweep
- `compareFixed.C`, `compareKinematic.C` -- automated comparison plots
