# Transitioning a Geant Simulation Macro to AtSimpleSim

## Status

This is local documentation for the `AtSimValidation` campaign. It records the migration path that is currently validated in this directory for swapping Geant/VMC transport with `AtSimpleSimulation` while preserving the ATTPC generator and detector contract.

## Migration Goal

The target transition is:

- preserve the existing ATTPC generator physics,
- preserve the detector geometry and field setup,
- preserve the output and runtime-db structure as much as possible,
- replace Geant/VMC transport with `AtSimpleSimulation`.

The migration should be about swapping the transport layer, not rewriting the reaction setup.

## Validated Hook

The framework adapter is `AtDigitization/AtTestSimulation`.

Its role is:

- own an `AtSimpleSimulation` instance,
- drive a `FairPrimaryGenerator` each event,
- collect generated primaries through `AtSimParticleCollector`,
- convert FairRoot units to `AtSimpleSimulation` units,
- forward the particles into `AtSimpleSimulation`,
- write `AtTpcPoint` output through the normal branch contract.

This is the hook that should be used when adapting an existing Geant-style macro in this repository.

This directory has now exercised that hook in the interpreted validation macros: `AtTestSimulation` can replace the macro-local transport task and still produce the standard `AtTpcPoint` branch.

Current validated behavior:

- `AtTestSimulation` now has a detector-coupled mode that feeds shared `AtTpc` step logic.
- The fixed validation macro uses that detector-coupled mode.
- Beam-event `AtTpcPoint` output is now produced through the detector path.
- In the fixed validation case, detector-side reaction handoff now reaches `AtTPC2Body` with a
  non-zero residual beam energy and produces reaction-event `AtTpcPoint` output.
- In the kinematic validation case, the adapter now also restores the canonical `MCTrack` truth
  branch while preserving Geant-style reaction-event IDs (`track 0` scattered ion, `track 1`
  recoil proton), so downstream truth consumers such as `visualizeKinematic.C` can find the
  transported reaction products without a placeholder beam slot.
- The detector now stops transport when a particle exits the active reaction volume. In this
  validation geometry that is the physically correct approximation because the active gas is bounded
  by chamber walls. The Geant path and the SimpleSim path now use the same stop rule.

## Migration Recipe

### 1. Start from a working Geant macro

Keep these pieces unchanged unless the migration experiment proves otherwise:

- detector modules such as `AtCave` and `AtTpc`,
- geometry file selection,
- magnetic-field setup,
- random seed policy,
- runtime-db output handling,
- output file naming pattern,
- generator construction function such as `BuildElasticGenerator(...)`.

### 2. Preserve the generator block

Build the same `FairPrimaryGenerator` and ATTPC generator chain used by the Geant macro. The same beam and reaction generators should define the physics input on both sides.

### 3. Replace Geant transport with AtTestSimulation

For the SimpleSim version:

- keep `FairRunSim`,
- keep the detector geometry and field configuration,
- do not hand the physics generator to Geant transport with `run->SetGenerator(physicsGenerator)`,
- instead, give the run a minimal driver generator and attach an `AtTestSimulation` task configured with:
  - a new `AtSimpleSimulation`,
  - the required energy-loss models for every transported species,
  - the physics generator that was preserved from the Geant macro.

This is the validated transport substitution mechanism in this directory.

Minimal pattern:

```cpp
run->SetGenerator(new FairPrimaryGenerator());

auto *simPrimGen = BuildElasticGenerator(...);
auto *simTask = new AtTestSimulation(BuildSimpleSimulation(dir + "/geometry/ATTPC_He1bar_geomanager.root"));
simTask->SetPrimaryGenerator(simPrimGen);
simTask->SetDetector(tpc);
run->AddTask(simTask);
```

### 4. Configure AtSimpleSimulation explicitly

The migration must define:

- all species that need `AtTools::AtELossModel` entries,
- the geometry assumption for the active volume,
- field configuration if curved transport is required,
- propagation step settings if they matter for the validation case.

If any transported species lack a model, the migration is incomplete.

Minimal pattern:

```cpp
std::unique_ptr<AtSimpleSimulation> BuildSimpleSimulation(const TString &geoFile)
{
   auto sim = std::make_unique<AtSimpleSimulation>(geoFile.Data());

   constexpr double heDensity = 1.664e-4;
   std::vector<std::tuple<int, int, int>> material{{4, 2, 1}};

   auto carbonModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   carbonModel->SetProjectile(16, 6, 16.014701);
   sim->AddModel(6, 16, carbonModel, 16.014701);

   auto protonModel = std::make_shared<AtTools::AtELossCATIMA>(heDensity, material);
   protonModel->SetProjectile(1, 1, 1.0078250322);
   sim->AddModel(1, 1, protonModel, 1.0078250322);
   sim->SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0));
   sim->SetMaxPropagationStep(1e-3);

   return sim;
}
```

### 5. Keep the detector in the loop

The migration is not just a branch writer swap. `AtTpc` must still own:

- reaction triggering,
- `AtVertexPropagator` updates,
- detector-side hit semantics,
- stopping transport at the active-volume boundary.

This is why the migration uses `AtTestSimulation` in detector-coupled mode instead of a macro-local
task that writes `AtTpcPoint` objects directly.

### 6. Validate with truth-matched comparisons

When comparing Geant and SimpleSim outputs:

- match reaction events by `reactionIndex`,
- do not drop an event from the truth bookkeeping just because one side produced zero points,
- report `generator-only` and `incomplete` events separately from usable matched pairs.

This matters because the earlier local comparison macros hid zero-point events before matching and
made the transport disagreement look worse than it was.

## Current Known Constraints

- `AtTestSimulation` skips particles that start outside `drift_volume`.
- `AtSimpleSimulation` requires explicit energy-loss models for each `(Z, A)` species.
- The detector geometry file used by `AtTpc` is not necessarily the file that should be passed to `AtSimpleSimulation`. In this validation area the run uses `ATTPC_He1bar.root`, while `AtSimpleSimulation` needs the importable `ATTPC_He1bar_geomanager.root`.
- FairRoot generator output uses cm and GeV; `AtSimpleSimulation` uses mm and MeV.
- Interpreted ROOT macros are sensitive to explicit header inclusion. In this directory the stable pattern is to keep the macro header surface minimal and rely on the loaded dictionaries for FairRoot and ATTPC classes where possible.
- A migration is not considered successful just because the macro runs. The resulting tracks must also look physically credible.
- For this geometry, a particle leaving the active reaction volume should be treated as stopped by the
  wall boundary condition. Allowing Geant to continue transport past active-volume exit produces
  unphysical ranges for this validation problem.

## What Must Be Checked During Each Migration Attempt

### Structural checks

- Does the macro still have the same generator construction logic as the Geant source?
- Does the SimpleSim path write the expected `AtTpcPoint` branch?
- Can downstream comparison or digitization scripts read the result without special handling?

### Physics checks

- Are both sides running the same reaction setup?
- Are the geometry and magnetic field the same?
- Do the track shapes look qualitatively correct?
- Is the stopping behavior reasonable for the configured energy-loss model?

### Cleanliness checks

- Was `AtTestSimulation` sufficient, or did the macro need custom task code?
- Are the remaining edits small enough to describe as a checklist?
- Is there repeated boilerplate that points to missing framework support?

## Current Verified Result

The migration path is viable in this directory.

Verified local result:

- the macro-local transport task was removed,
- `AtTestSimulation` runs successfully in the validation macros,
- the detector-coupled path produces both beam-event and reaction-event `AtTpcPoint` output,
- the detector and SimpleSim path preserve the canonical truth contract used by local visualization,
- the Geant detector path now stops transport at active-volume exit just as the SimpleSim path does,
- the fixed comparison now gives `50` usable truth-matched proton pairs with `0/0` generator-only
  and `0/0` incomplete events,
- a reduced kinematic comparison now gives `100` usable truth-matched proton pairs with `0/0`
  generator-only and `0/0` incomplete events,
- the earlier catastrophic Geant path-length excess is gone once the active-volume stop rule is
  enforced.
