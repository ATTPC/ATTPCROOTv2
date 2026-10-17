# Plan: AtSimpleSim Validation and Migration Work

## Summary

This directory now has Geant transport macros, comparison macros, and a first pass at `AtSimpleSim` transport for the same validation campaign. The next step is not to update framework-wide documentation yet. The next step is to make the local migration story real, test it against the current macros, and iterate until there is a clean path from an existing Geant-style simulation macro to `AtSimpleSim`.

This document records the current validated state of the local migration and validation effort, how the `AtSimpleSim` hook works in practice, and which physics checks matter when comparing the SimpleSim and Geant transport paths.

## Current State of the Code

### Framework pieces that already exist

- `AtDigitization/AtSimpleSimulation.{h,cxx}`
  direct `AtMCPoint` generation using configured energy-loss models; supports straight-line transport in zero field and curved transport through `AtPropagator` when E/B fields are set.
- `AtDigitization/AtSimParticleCollector.{h,cxx}`
  a `FairGenericStack` stub that captures primaries pushed by `FairPrimaryGenerator::GenerateEvent()`.
- `AtDigitization/AtTestSimulation.{h,cxx}`
  a `FairTask` bridge that:
  - owns an `AtSimpleSimulation` instance,
  - drives a `FairPrimaryGenerator`,
  - collects generated particles through `AtSimParticleCollector`,
  - converts FairRoot units to `AtSimpleSimulation` units,
  - forwards each particle to `AtSimpleSimulation::SimulateParticle()`,
  - registers the `AtTpcPoint` branch.
- `AtDigitization/AtSimTest.cxx`
  compiled unit coverage for `AtSimpleSimulation` transport behavior.
- `AtDetectors/AtTpc/AtTpc.{h,cxx}`
  now has a transport-neutral step-processing entrypoint so detector-owned reaction logic can be
  exercised outside VMC.

### Validation work that already exists in this directory

- Geant macros run in `macro/Simulation/AtSimValidation/`.
- Visualization and comparison macros exist.
- SimpleSim validation macros now run through `AtTestSimulation` and write `AtTpcPoint` output.
- The fixed SimpleSim validation macro now uses the detector-coupled adapter path instead of the
  collector-only branch writer.

### What was wrong and is now fixed

- The SimpleSim validation macros originally introduced their own macro-local `SimpleSimTask` class instead of using `AtTestSimulation`.
- That duplication has been removed.
- The Geant detector path allowed transport to continue after active-volume exit, which is not the
  intended physics for this validation geometry.
- The comparison macros previously dropped zero-point events before truth matching and used an
  unstable truth key based on proton start-state coordinates.
- The fixed and kinematic validation macros now run the same `16C + p` configuration on both sides
  and compare truth-matched reaction events by reaction index.

## How the AtSimpleSim Hook Was Added

The existing bridge was added in the framework, not in this macro directory.

The transport substitution works like this:

1. Build the normal FairRoot generator chain with `FairPrimaryGenerator` plus existing ATTPC generators such as `AtTPCIonGenerator` and `AtTPC2Body`.
2. Instead of sending those generated primaries into Geant/VMC transport, call `FairPrimaryGenerator::GenerateEvent()` with `AtSimParticleCollector`.
3. `AtSimParticleCollector` records the generated primaries with FairRoot units:
   - position in cm
   - momentum and energy in GeV
4. `AtTestSimulation` converts each collected particle to `AtSimpleSimulation` units:
   - cm to mm
   - GeV to MeV
5. `AtTestSimulation` calls `AtSimpleSimulation::SimulateParticle(...)` for each primary.
6. `AtSimpleSimulation` writes `AtMCPoint` objects directly to the standard simulation branch.

One important implementation detail uncovered during this migration work: the geometry file handed to `AtTpc` for the run is not automatically the right file for `AtSimpleSimulation`. In this directory the run geometry uses `ATTPC_He1bar.root`, while `AtSimpleSimulation` must be constructed with the importable `ATTPC_He1bar_geomanager.root`.

This means the intended migration is to preserve the generator physics and replace only the transport mechanism.

## Working Migration Path

### 1. Keep the validation macros on the framework bridge

The validation macros in this directory now use `AtTestSimulation` directly. Any future migration attempt in this area should keep that pattern. Reintroducing a private transport task in a macro body would be a regression back to ad hoc glue.

### 2. Write and maintain local draft documentation

Keep draft documentation local to this directory until behavior is verified.

Required local documents:

- `AtSimValidationPlan.md`
  this status-and-work document.
- `AtSimpleSimMigrationDraft.md`
  the working migration guide for converting an existing Geant-style macro to `AtSimpleSim`.

### 3. Attempt a real macro transition

Use the Geant validation macro shape already present in this directory as the first migration target.

The migration attempt should answer:

- what can stay identical,
- what must change,
- whether `AtTestSimulation` is sufficient without additional framework work,
- whether the remaining differences are small enough to describe as a repeatable checklist.

### 4. Only then identify framework follow-up

If the migration is still awkward after using `AtTestSimulation`, record the missing framework work as concrete follow-up items tied to observed problems. Do not invent a second local harness and do not promote any behavior to main docs before it has been exercised here.

## Actual Validation Procedure

1. Regenerate the Geant reference output with the current detector code.
2. Regenerate the SimpleSim output with the same generator and geometry choices.
3. Compare only truth-matched reaction events.
4. Report usable pairs separately from generator-only and incomplete events.
5. Inspect both kinematics and path-length distributions, not just whether the macro ran.

## Acceptance Criteria for a Viable Migration Strategy

A migration strategy is viable only if all of the following are true:

- an existing Geant-style macro can be adapted without introducing new macro-local transport glue,
- the adaptation preserves the original generator block and detector setup as much as possible,
- the SimpleSim path produces the expected `AtTpcPoint` output branch for downstream tasks,
- the resulting tracks and stopping behavior are physically credible for the chosen validation case,
- the required edits are small and stable enough to document as a repeatable procedure.

Until those conditions are met, this work remains a local validation and design iteration effort.

## Current Verified Result

- `AtTpc` now owns a shared step-processing path used by both VMC and the SimpleSim adapter.
- Focused unit coverage now exists for:
  - detector reaction triggering,
  - detector-side `AtVertexPropagator` writes and resets,
  - non-beam metadata lookup,
  - transport stop on active-volume exit.
- `simpleSim_fixed.C` now produces non-empty beam-event `AtTpcPoint` output through the shared
  detector path.
- `simpleSim_fixed.C` now triggers detector-side `AtVertexPropagator` handoff and produces
  non-empty reaction-event `AtTpcPoint` output in the fixed validation case.
- `simpleSim_kinematic.C` now preserves the canonical track-ID contract:
  - beam remains `trackID == 0`,
  - the reaction-event scattered ion remains `trackID == 0`,
  - the recoil proton is written as `trackID == 1`,
  - the standard `MCTrack` branch is populated without duplicate `MCTrack_*` aliases.
- The detector-side beam-only semantics are now driven by an explicit beam-phase flag carried in
  the shared `AtTpc` step payload. That avoids the earlier adapter bug where reaction-event
  `trackID == 0` was mistaken for the beam after `AtReactionGenerator` had already toggled
  `AtVertexPropagator` to the next event phase.
- `AtTpc` now stops transport when a particle exits the active reaction volume. For this local
  validation geometry that is the intended wall boundary condition.
- `visualizeKinematic.C("./data/simpleSim_kinematic.root", 2, 4)` now finds the recoil-proton
  truth track. For the current 2-event spot check it reports `Drew 1 trajectories and 1 event
  points`; for the earlier 10-event run it reported `Drew 4 trajectories and 5 event points`.
- The long-run kinematic validation no longer stalls around event 190. The failure was a curved-
  transport stopping tail for very low-energy recoil protons in the detector-coupled SimpleSim
  path. `AtSimpleSimulation` now stops curved tracks below a configurable `0.1 MeV` tolerance by
  default, which avoids spending pathological CPU time on the last few millimeters of sub-100 keV
  proton range. After that change:
  - `simpleSim_kinematic.C(190, 42)` completes in about `2.94 s`
  - `simpleSim_kinematic.C(200, 42)` completes in about `2.98 s`
  - `visualizeKinematic.C("./data/simpleSim_kinematic.root", 2, 4)` still reports
    `Drew 4 trajectories and 100 event points` on the 200-event output.
- The validation comparison macros now match by `reactionIndex` and keep zero-point events in the
  truth bookkeeping.
- On regenerated outputs:
  - `compareFixed.C` reports `50` usable truth-matched pairs with `0/0` generator-only and `0/0`
    incomplete events.
  - `compareKinematic.C` reports `100` usable truth-matched pairs with `0/0` generator-only and
    `0/0` incomplete events for the reduced 200-event spot check.
- The previous Geant path-length inflation disappeared once the detector-side active-volume stop was
  enforced. Residual Geant vs SimpleSim path differences are now at the mm-to-tens-of-mm scale
  instead of the earlier hundreds-to-thousands-of-mm scale.
