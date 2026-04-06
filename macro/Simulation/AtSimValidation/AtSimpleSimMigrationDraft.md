# Draft: Transitioning a Geant Simulation Macro to AtSimpleSim

## Status

This is a local draft for the `AtSimValidation` campaign. It is not framework documentation yet. Its purpose is to describe the candidate migration path, test that path against real macros, and record what has to change before the migration can be treated as clean and reusable.

## Migration Goal

The target transition is:

- preserve the existing ATTPC generator physics,
- preserve the detector geometry and field setup,
- preserve the output and runtime-db structure as much as possible,
- replace Geant/VMC transport with `AtSimpleSimulation`.

The migration should be about swapping the transport layer, not rewriting the reaction setup.

## Current Candidate Hook

The current framework adapter is `AtDigitization/AtTestSimulation`.

Its role is:

- own an `AtSimpleSimulation` instance,
- drive a `FairPrimaryGenerator` each event,
- collect generated primaries through `AtSimParticleCollector`,
- convert FairRoot units to `AtSimpleSimulation` units,
- forward the particles into `AtSimpleSimulation`,
- write `AtTpcPoint` output through the normal branch contract.

At present, this is the first hook that should be used when adapting an existing macro.

This directory has now exercised that hook in the interpreted validation macros: `AtTestSimulation` can replace the macro-local transport task and still produce the standard `AtTpcPoint` branch.

## Draft Migration Recipe

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

This is the current transport substitution mechanism being tested in this directory.

### 4. Configure AtSimpleSimulation explicitly

The migration must define:

- all species that need `AtTools::AtELossModel` entries,
- the geometry assumption for the active volume,
- field configuration if curved transport is required,
- propagation step settings if they matter for the validation case.

If any transported species lack a model, the migration is incomplete.

## Current Known Constraints

- `AtTestSimulation` skips particles that start outside `drift_volume`.
- `AtSimpleSimulation` requires explicit energy-loss models for each `(Z, A)` species.
- The detector geometry file used by `AtTpc` is not necessarily the file that should be passed to `AtSimpleSimulation`. In this validation area the run uses `ATTPC_He1bar.root`, while `AtSimpleSimulation` needs the importable `ATTPC_He1bar_geomanager.root`.
- FairRoot generator output uses cm and GeV; `AtSimpleSimulation` uses mm and MeV.
- Interpreted ROOT macros are sensitive to explicit header inclusion. In this directory the stable pattern is to keep the macro header surface minimal and rely on the loaded dictionaries for FairRoot and ATTPC classes where possible.
- A migration is not considered successful just because the macro runs. The resulting tracks must also look physically credible.

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

## Current Working Assumption

The migration path is viable if the validation macros can be rewritten around `AtTestSimulation` without introducing new macro-local transport logic. If that succeeds and the resulting behavior is physically credible, this draft can be promoted later into framework documentation. If it fails, this draft should be revised to record the exact missing framework support instead of papering over the problem.

Current local result:

- the macro-local transport task was removed,
- `AtTestSimulation` runs successfully in the validation macros,
- the SimpleSim output file contains the expected `AtTpcPoint` branch,
- the next unresolved issue is physics/configuration parity with the Geant comparison macros, not the transport hook itself.
