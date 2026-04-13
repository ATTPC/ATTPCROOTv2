# Migrating a Simulation Macro to SimpleSim

This guide shows how to convert an existing Geant4/VMC simulation macro to use SimpleSim transport instead. The macro structure, generator setup, and detector configuration stay the same.

## Class vocabulary (after the rename)

SimpleSim is split into two user-facing classes plus one FairRoot task:

- **`AtSimTransport`** — low-level transport engine. Callback-based, no hit recording, no FairRoot. Used by everything else.
- **`AtSimpleSimulation`** — standalone user-facing class. Owns an `AtSimTransport` and records hits into a `TClonesArray`. Used by `AtMCFitter`, `AtMCFission`, and analysis macros that run their own event loop.
- **`AtSimTransportTask`** — FairRoot-integration task (plus `AtSimTransportGeneratorTask` and `AtSimTransportReplayTask` subclasses). Couples an `AtSimTransport` engine to `AtTpc` via the shared `ProcessStep` contract.

Configuration forwarders on `AtSimpleSimulation` (`AddModel`, `SetMagneticField`, `SetMaxStep`, …) pass through to the engine, so macros that predate the split do not need to reach through `GetEngine()`.

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

// Build an energy-loss manager with the models this run needs
auto manager = std::make_shared<AtTools::AtELossManager>();

auto carbonModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, gasMaterial);
carbonModel->SetProjectile(16, 6, 16.014701);
manager->AddModel(6, 16, carbonModel);

auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, gasMaterial);
protonModel->SetProjectile(1, 1, 1.0078250322);
manager->AddModel(1, 1, protonModel);

// Build the transport engine (uses the geometry already loaded by FairRunSim)
auto sim = std::make_unique<AtSimTransport>(manager);

// Set the magnetic field (in Tesla)
sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));

// Create the task, connect the generator and detector
auto *simTask = new AtSimTransportGeneratorTask(std::move(sim));
simTask->SetPrimaryGenerator(primGen);
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

## Required configuration

### Energy-loss models

Every particle species that crosses a given material needs a model. Two approaches:

#### Auto-generating manager (recommended)

Pass an `AtELossManagerBetheBloch` or `AtELossManagerCATIMA`; they synthesize a model the first time any (Z, A, material) combination is encountered during transport, using density and composition from the geometry:

```cpp
auto manager = std::make_shared<AtTools::AtELossManagerCATIMA>();
// optional: manager->SetConfig(myCatimaConfig);
auto sim = std::make_unique<AtSimTransport>(manager);
```

No per-species configuration needed. Pre-registering a model with `manager->AddModel(Z, A, model)` still works and takes priority over auto-generation.

#### Manual-only registration

Use the accept-only base class if you want full control (or need to register SRIM/LISE tables that the factory cannot synthesize):

```cpp
auto manager = std::make_shared<AtTools::AtELossManager>();
auto table = std::make_shared<AtTools::AtELossTable>();
table->LoadSrimTable("PbinHe.txt");
manager->AddModel(82, 208, table);
```

If a particle has no registration and the manager is accept-only, `GetModel` returns `nullptr` and transport stops.

#### Available types

- `AtTools::AtELossManager` — accept-only (manual registrations only).
- `AtTools::AtELossManagerCATIMA` — auto-generates CATIMA models (recommended).
- `AtTools::AtELossManagerBetheBloch` — auto-generates Bethe-Bloch models (lighter, less accurate).
- `AtTools::AtELossCATIMA` / `AtTools::AtELossTable` / `AtTools::AtELossBetheBloch` — concrete model implementations registered via `AddModel`.

See [energy-loss.md](energy-loss.md) for details.

### Detector coupling

`simTask->SetDetector(tpc)` connects the transport engine to the `AtTpc` detector so that steps go through the same entering/accumulate/react pipeline used by Geant4 (`AtTpc::ProcessStep`). This is required for correct output.

### Magnetic field

If the experiment uses a magnetic field, set it on the engine:

```cpp
sim->SetMagneticField(ROOT::Math::XYZVector(Bx, By, Bz));  // Tesla
```

This enables curved-track propagation via RK4. Without a field, particles propagate in straight lines.

### Step size

`sim->SetMaxStep(stepMm)` sets the maximum distance per step (mm). It applies to both the straight-line and curved paths — whichever is active based on field settings. The legacy name `SetDistanceStep` is kept as an alias.

### Geometry

`AtSimTransport()` (default constructor) uses the geometry that `FairRunSim` loads. No separate geometry file is needed. If you need a standalone geometry file (e.g., for testing outside FairRunSim), pass it to the constructor:

```cpp
auto sim = std::make_unique<AtSimTransport>("path/to/geomanager.root", manager);
```

## Replay mode

`AtSimTransportReplayTask` re-transports primary tracks from a prior Geant4 run through SimpleSim. This enables direct A/B comparison with identical kinematics:

```cpp
auto *simTask = new AtSimTransportReplayTask(std::move(sim));
simTask->SetPrimaryTrackSource("geant4_output.root");  // must have MCTrack branch on "cbmsim" tree
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

The source file must contain a `cbmsim` TTree with an `MCTrack` branch from a prior Geant4 run.

## Validation macros

Working examples are in `macro/Simulation/AtSimValidation/`:

- `simpleSim_fixed.C` / `geant4_fixed.C` — fixed-angle comparison (manual model registration)
- `simpleSim_kinematic.C` / `geant4_kinematic.C` — full kinematic sweep (manual model registration)
- `simpleSim_fixed_factory.C` / `simpleSim_kinematic_factory.C` — auto-generating CATIMA manager variants
- `simpleSim_fixed_bethebloch.C` — auto-generating Bethe-Bloch manager variant
- `compareFixed.C`, `compareKinematic.C` — automated comparison plots (accept configurable file paths)
