# User Guide: Migrating a Simulation Macro from Geant to SimpleSim

## Purpose

This guide is for users who already have a working simulation macro and want to swap the transport from Geant/VMC to SimpleSim without changing the physics setup.

The goal is simple:

- keep the same macro structure
- keep the same generator setup
- keep the same detector setup
- replace only the transport hookup

## What You Usually Start With

A typical user macro already has:

- a `FairRunSim`
- detector modules such as `AtCave` and `AtTpc`
- an inline `FairPrimaryGenerator` setup
- a line like `run->SetGenerator(primGen)`
- `run->Init()` and `run->Run(nEvents)`

That is enough. You do not need to restructure the macro into helper functions to use SimpleSim.

## What Stays the Same

Keep these parts unchanged unless you have a physics reason to change them:

- the beam definition
- the reaction-generator configuration
- the detector modules
- the run geometry
- the magnetic field
- the random seed handling
- the output file and parameter-file setup

If the Geant macro already produces the right physics input, preserve that input.

## What Actually Changes

You replace the transport hookup.

In a Geant macro, the physics generator is usually connected directly to the run:

```cpp
run->SetGenerator(primGen);
```

In a SimpleSim macro, the run gets a dummy event-loop generator, and the real physics generator is passed to `AtSimpleSimulationGeneratorTask`:

```cpp
run->SetGenerator(new FairPrimaryGenerator());

auto *simTask = new AtSimpleSimulationGeneratorTask(BuildSimpleSimulation());
simTask->SetPrimaryGenerator(primGen);
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

That is the main migration step.

## Minimal Migration Procedure

### 1. Copy the working Geant macro

Start from the macro that already works for your case.

Do not refactor the macro at the same time. First make the transport swap only.

### 2. Keep the generator block as it is

If your macro builds the generator inline, keep it inline.

For example, if you already have:

```cpp
auto *primGen = new FairPrimaryGenerator();

auto *ionGen = new AtTPCIonGenerator(...);
primGen->AddGenerator(ionGen);

auto *twoBody = new AtTPC2Body(...);
primGen->AddGenerator(twoBody);
```

leave that section alone.

The migration should not require users to move their generator code into helper functions.

### 3. Add a SimpleSim configuration block

You need one place where `AtSimpleSimulation` is configured.

This can be:

- a helper function such as `BuildSimpleSimulation(...)`
- or inline setup code if you prefer

What matters is that you configure:

- the geometry file for SimpleSim
- the transported species
- the energy-loss models
- the field, if needed
- the step settings, if needed

Minimal example:

```cpp
std::unique_ptr<AtSimpleSimulation> BuildSimpleSimulation()
{
   auto sim = std::make_unique<AtSimpleSimulation>(); // uses FairRunSim geometry

   constexpr double gasDensity = 1.664e-4;
   std::vector<std::tuple<int, int, int>> material{{4, 2, 1}};

   auto ionModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, material);
   ionModel->SetProjectile(16, 6, 16.014701);
   sim->AddModel(6, 16, ionModel, 16.014701);

   auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(gasDensity, material);
   protonModel->SetProjectile(1, 1, 1.0078250322);
   sim->AddModel(1, 1, protonModel, 1.0078250322);

   sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));
   sim->SetMaxPropagationStep(1e-3);
   return sim;
}
```

You must add a model for every species that will be transported.

### 4. Replace the Geant transport hookup

Leave the run setup mostly alone and replace only the generator hookup.

Typical Geant-style pattern:

```cpp
run->SetGenerator(primGen);
```

SimpleSim pattern:

```cpp
run->SetGenerator(new FairPrimaryGenerator());

auto *simTask = new AtSimpleSimulationGeneratorTask(BuildSimpleSimulation());
simTask->SetPrimaryGenerator(primGen);
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

Important:

- `FairRunSim` still needs a generator object for the event loop
- the real physics generator is now passed into the SimpleSim task
- `AtSimpleSimulation()` (default constructor) automatically uses the geometry loaded by `FairRunSim` -- no separate geometry file needed

### 5. Keep the detector coupled

Always connect the detector:

```cpp
simTask->SetDetector(tpc);
```

That keeps detector-side behavior such as:

- reaction handling
- `AtVertexPropagator` updates
- `AtTpcPoint` production
- active-volume stopping behavior

## Copy-and-Edit Checklist

For most user macros, the migration is:

1. Copy the Geant macro to a new SimpleSim macro.
2. Leave the generator block unchanged.
3. Add SimpleSim configuration.
4. Replace `run->SetGenerator(primGen)` with:
   - `run->SetGenerator(new FairPrimaryGenerator())`
   - `AtSimpleSimulationGeneratorTask`
   - `simTask->SetPrimaryGenerator(primGen)`
   - `simTask->SetDetector(tpc)`
   - `run->AddTask(simTask)`
5. Add energy-loss models for all transported species.
7. Run a small sample first.
8. Verify the output before scaling up.

## What to Verify After Migration

### Output checks

- The output ROOT file is produced.
- The `cbmsim` tree contains `AtTpcPoint`.
- The truth branches expected by your downstream macros are present.
- Downstream analysis macros can open the file without special-case handling.

### Physics checks

- The beam and reaction setup are unchanged from the Geant macro.
- The geometry and field are unchanged.
- Track shapes look reasonable.
- Path lengths are physically credible.
- Stopping behavior looks right for your detector geometry.

For the local validation geometry in this directory, particles are expected to stop when they leave the active reaction volume because that boundary corresponds to chamber material, not open vacuum.

## How to Compare Against Geant

Do not rely on file order alone when comparing outputs.

Use truth-matched comparisons and keep these categories separate:

- usable matched events
- generator-only events
- incomplete events

Local comparison macros in this directory already do that:

- [compareFixed.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/compareFixed.C)
- [compareKinematic.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/compareKinematic.C)

## Practical Notes

- `AtSimpleSimulation` needs explicit energy-loss models.
- The SimpleSim task skips particles that start outside `drift_volume`.
- `AtSimpleSimulation` uses mm and MeV internally.
- The generator side still comes from the normal FairRoot macro world, which uses cm and GeV.
- When editing ROOT macros, prefer adapting an existing working macro instead of inventing a new structure.

## Suggested Workflow

1. Start from your existing Geant macro.
2. Change only the transport hookup.
3. Run a very small sample.
4. Check that `AtTpcPoint` and truth output look sane.
5. Compare against the Geant output.
6. Only then scale up to larger production runs.

## Optional Local References

If you want concrete examples of this migration pattern, see:

- [geant4_fixed.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/geant4_fixed.C)
- [simpleSim_fixed.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/simpleSim_fixed.C)
- [geant4_kinematic.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/geant4_kinematic.C)
- [simpleSim_kinematic.C](/home/adam/ATTPCROOTv2-Sim/macro/Simulation/AtSimValidation/simpleSim_kinematic.C)
