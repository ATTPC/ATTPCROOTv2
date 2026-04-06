# Plan: Integrating AtSimpleSim as a Natural Simulation Hook

## Purpose

This document is a standalone implementation plan for adapting `AtSimpleSimulation` into the main ATTPC simulation pathway without breaking the existing simulation contract.

The goal is not to create a parallel ad hoc pathway that happens to produce `AtTpcPoint`. The goal is to make `AtSimpleSimulation` act as a transport replacement while preserving the detector-side logic and shared simulation state that the current Geant/VMC path relies on.

This document is intended to be handed to a new agent with no prior conversation context.

## Current Problem

The existing SimpleSim bridge captures particles too early.

Current hook:

- `AtDigitization/AtTestSimulation.{h,cxx}`
- `AtDigitization/AtSimParticleCollector.{h,cxx}`

Current behavior:

1. `FairPrimaryGenerator::GenerateEvent()` is called against `AtSimParticleCollector`
2. `AtSimParticleCollector` stores the generated primaries at `PushTrack(...)` time
3. `AtTestSimulation` converts those particles to SimpleSim units
4. `AtTestSimulation` calls `AtSimpleSimulation::SimulateParticle(...)`

This preserves generator logic, but it bypasses the detector stepping logic that the Geant/VMC path uses to:

- decide where a reaction occurs
- update `AtVertexPropagator`
- populate track-angle and track-energy metadata for reaction products
- apply detector-specific hit logic inside `AtTpc`

As a result, the current SimpleSim hook is not a drop-in replacement for the transport layer.

## Key Conclusion

`AtSimpleSimulation` should not be integrated primarily by intercepting particles at the custom stack level.

Why:

- `AtStack` and `AtSimParticleCollector` only see particles when they are pushed by the generator
- they do not know about detector volume entry, energy-loss accumulation, stopping condition, or reaction position
- those semantics currently live in `AtDetectors/AtTpc/AtTpc.cxx`

Therefore the correct integration point is not “generator to stack.”

The correct integration point is the transport-to-detector boundary.

## Compatibility Constraint

Any integration work proposed here must preserve the existing behavior of `AtSimpleSimulation` for other code paths that already use it outside the main simulation flow.

Known existing users include:

- MC fitting and reconstruction-side tools that depend on `AtSimpleSimulation` directly
- existing analysis or visualization macros that construct `AtSimpleSimulation` without the full detector simulation stack

Relevant files to inspect before refactoring:

- `AtReconstruction/AtFitter/AtMCFitter.cxx`
- `AtReconstruction/AtFitter/AtMCFission.cxx`
- `macro/e12014/adam/simulation/simpleSim.C`
- `macro/e12014/adam/determineZ/run_eve_sim.C`

Implementation implication:

- do not repurpose `AtSimpleSimulation` itself into something that requires `AtTpc`, `FairRunSim`, or `AtVertexPropagator` to function in its existing standalone uses
- keep the new hook logic in an adapter layer around `AtSimpleSimulation`, not by breaking the current direct-usage API
- any shared detector-side path introduced for the main simulation flow must be additive and must not regress current SimpleSim-based fitting or analysis workflows

## Existing Contract in the Geant/VMC Path

### Relevant files

- `AtDetectors/AtTpc/AtTpc.cxx`
- `AtSimulationData/AtVertexPropagator.{h,cxx}`
- `AtGenerators/AtReactionGenerator.{h,cxx}`
- `AtGenerators/AtTPCIonGenerator.cxx`
- `AtGenerators/AtTPC2Body.cxx`
- `AtSimulationData/AtStack.{h,cxx}`

### Current data flow

1. `FairPrimaryGenerator` runs the generator chain.
2. `AtStack` stores pushed particles for VMC transport.
3. VMC transports the beam particle through `AtTpc`.
4. `AtTpc::ProcessHits()` is called on detector steps.
5. `AtTpc` accumulates energy loss and tracks volume entry/exit.
6. `AtTpc::reactionOccursHere()` checks whether the sampled reaction point has been reached.
7. `AtTpc::startReactionEvent()` writes the reaction state into `AtVertexPropagator`.
8. On the following reaction event, `AtTPC2Body` reads `AtVertexPropagator` and generates the reaction products from the correct residual beam state.

### Critical detector-side logic

The core contract currently lives in `AtDetectors/AtTpc/AtTpc.cxx`.

Important methods:

- `trackEnteringVolume()`
- `getTrackParametersFromMC()`
- `getTrackParametersWhileExiting()`
- `reactionOccursHere()`
- `startReactionEvent()`
- `addHit()`
- `resetVertex()`

Most important line of contract:

- `startReactionEvent()` calls `AtVertexPropagator::SetVertex(...)`

That write includes:

- reaction vertex position
- beam entrance position into the TPC
- beam momentum at the reaction point
- residual beam energy at the reaction point

This is the state that the reaction generators need in active-target mode.

### Generator dependence on that contract

`AtGenerators/AtTPC2Body.cxx` uses:

- `AtVertexPropagator::GetEnergy()`
- `AtVertexPropagator::GetVx()/GetVy()/GetVz()`
- `AtVertexPropagator::GetPx()/GetPy()/GetPz()`

If that state is not populated by the beam transport phase, the reaction generator does not reproduce the Geant pathway.

## Why the Current SimpleSim Hook Is Insufficient

The current SimpleSim bridge bypasses `AtTpc::ProcessHits()`.

That means it bypasses:

- detector-side reaction trigger logic
- the handoff into `AtVertexPropagator`
- detector-side hit population semantics

Even if the same `FairPrimaryGenerator` object is reused, that is not enough. The Geant pathway depends on detector-produced state, not just generator-produced state.

So the current bridge is preserving generator syntax, but not the simulation contract.

## Verified on This Branch

The shared detector-path refactor is now in place and the current local status is:

- `AtTpc` owns the shared step-processing logic for both VMC and the SimpleSim adapter.
- `AtTestSimulation` now preserves the canonical track-ID convention:
  - beam is `trackID == 0`,
  - the reaction-event scattered ion also remains `trackID == 0`,
  - the recoil proton is `trackID == 1`,
  - the standard `MCTrack` branch is filled through the existing stack-owned branch name.
- The shared detector step now carries explicit beam-phase information from the transport adapter.
  This is required because `AtReactionGenerator::ReadEvent()` toggles
  `AtVertexPropagator::IsBeamEvent()` before transport starts, so detector stepping cannot infer
  the current transport phase from the singleton alone.
- `simpleSim_kinematic.C` now produces visible reaction tracks in
  `visualizeKinematic.C`; with `simpleSim_kinematic.C(10, 42)` the viewer reports
  `Drew 4 trajectories and 5 event points`.
- The earlier failure mode was adapter-side:
  reaction-event `trackID == 0` was first handled by shifting products away from slot `0`, but
  that diverged from the Geant truth contract. The current fix keeps Geant-style IDs and instead
  prevents reaction-event `trackID == 0` from tripping beam-only detector semantics by carrying the
  beam/reaction phase explicitly into the shared detector-step path. A duplicate `MCTrack`
  registration had also caused ROOT to rename the truth branches away from the canonical
  `MCTrack` name expected by the visualizer.

## Proposed Integration Strategy

### Design principle

Treat `AtSimpleSimulation` as a transport replacement, not as a detector replacement.

The detector logic should remain the owner of:

- reaction triggering
- `AtVertexPropagator` updates
- detector hit semantics

### Primary refactor target

Refactor `AtDetectors/AtTpc/AtTpc.cxx` so the transport-specific VMC reads are separated from the detector’s core logic.

The intended structure is:

1. a thin VMC adapter layer in `AtTpc::ProcessHits()`
2. a transport-neutral internal interface that consumes step state

Conceptually:

- keep `AtTpc::ProcessHits(FairVolume *vol)` for Geant/VMC
- add a new internal method that processes one transport step from plain data

Example conceptual shape:

```cpp
struct AtTpcStepState {
   int trackID;
   int pdg;
   std::string volumeName;
   int volumeID;
   int detCopyID;
   bool isEntering;
   bool isExiting;
   bool isStopping;
   bool isDisappeared;
   double eDep;
   double trackTimeNs;
   double trackLength;
   TLorentzVector posIn;
   TLorentzVector posOut;
   TLorentzVector momIn;
   TLorentzVector momOut;
   double etotGeV;
   double trackMassGeV;
};
```

The exact type names can differ, but the intent should be this:

- Geant/VMC fills this from `gMC`
- SimpleSim fills this from its own propagation state
- the detector core uses the same path in both cases

### Target shared logic inside AtTpc

These behaviors should move under the shared transport-neutral path:

- beam entering-volume bookkeeping
- energy-loss accumulation
- reaction-point decision
- `AtVertexPropagator::SetVertex(...)`
- detector hit creation
- track-angle and track-energy lookup for non-beam products

### Role of AtSimpleSimulation after refactor

`AtSimpleSimulation` should provide transport evolution and step-state generation, not the top-level detector semantics.

That means:

- it can still be responsible for energy loss and curved/straight propagation
- but it should not be the sole owner of writing final `AtTpcPoint` objects for the main simulation path if that bypasses `AtTpc`

Instead, it should feed a detector-facing adapter that calls the shared `AtTpc` step-processing code.

## Concrete Hook Points

### Hook point 1: AtTpc internal refactor

File:

- `AtDetectors/AtTpc/AtTpc.cxx`

Work:

- isolate all direct `gMC` reads behind a thin adapter
- introduce a transport-neutral step-processing path
- move `reactionOccursHere()` and `startReactionEvent()` under that shared path

Why:

- this is where the real Geant contract currently lives

### Hook point 2: SimpleSim transport adapter

Likely files:

- `AtDigitization/AtSimpleSimulation.{h,cxx}`
- new adapter file in `AtDigitization/`
- possibly refactor or replace `AtTestSimulation`

Work:

- drive the same generator chain
- consume generated beam/reaction tracks
- propagate them with SimpleSim
- emit transport step-state objects compatible with the new `AtTpc` shared interface

Why:

- this keeps SimpleSim as a transport engine while preserving detector semantics

### Hook point 3: Stack and generator flow

Files:

- `AtSimulationData/AtStack.{h,cxx}`
- `AtDigitization/AtSimParticleCollector.{h,cxx}`
- `AtDigitization/AtTestSimulation.{h,cxx}`

Current status:

- useful for studying generator output
- not sufficient as the primary integration layer

Expected future role:

- either reduced to a helper for capturing generated primaries
- or replaced by a more natural transport runner once the detector-side shared path exists

Important note:

- do not put `AtVertexPropagator` contract logic into the collector
- do not reimplement detector reaction semantics inside a custom stack

## Expected Data Flow After Refactor

### Geant/VMC path

1. Generator chain runs as it does now.
2. `AtStack` stores primaries.
3. VMC transports tracks.
4. `AtTpc::ProcessHits()` builds step state from `gMC`.
5. Shared `AtTpc` transport-neutral logic processes that state.
6. `AtVertexPropagator` is updated by detector logic.
7. Reaction generators consume that state as they do now.

### SimpleSim path

1. Same generator chain runs.
2. Transport runner obtains the generated primaries.
3. `AtSimpleSimulation` propagates the tracks.
4. SimpleSim transport emits step-state objects equivalent to the Geant detector-facing information.
5. Shared `AtTpc` transport-neutral logic processes that state.
6. `AtVertexPropagator` is updated by detector logic, not by macro glue.
7. Reaction generators consume that state without special-case changes.

This is the required shape for “drop-in replacement” to be credible.

## What Must Not Be Done

- Do not solve this by setting fixed target positions in macros.
- Do not solve this by bypassing `AtVertexPropagator`.
- Do not encode reaction-trigger semantics directly into `AtTestSimulation`.
- Do not document a user-facing migration path until the active-target contract is actually preserved.
- Do not treat non-empty `AtTpcPoint` output alone as proof that the contract is correct.

## Test Plan

The test plan must validate both structure and physics.

### 1. Unit tests for transport-neutral detector logic

Target:

- new tests around the shared `AtTpc` step-processing logic

What to validate:

- entering-volume state is tracked correctly
- energy-loss accumulation matches expected behavior
- reaction trigger fires only for beam track in the right volume
- `AtVertexPropagator::SetVertex(...)` receives the correct position, momentum, and residual energy
- non-beam tracks receive angle/energy metadata consistently

Why:

- this is the core contract that must be shared between Geant and SimpleSim

### 2. Unit tests for VertexPropagator handoff

Files:

- extend `AtSimulationData/AtVertexPropagatorTest.cxx` or add adjacent tests

What to validate:

- detector-side update path writes the expected state
- reaction generators can read the written state and produce non-zero kinematics
- event alternation still behaves as expected

### 3. Generator compatibility tests

Targets:

- `AtTPCIonGenerator`
- `AtTPC2Body`
- `AtReactionGenerator`

What to validate:

- the same generator chain works with detector-produced `AtVertexPropagator` state
- no macro-level fixed-target workaround is required for the active-target case

### 4. Integration test for the current fixed validation case

Macro:

- `macro/Simulation/AtSimValidation/simpleSim_fixed.C`

Success criteria:

- macro runs from a clean output directory
- beam event produces the detector-side reaction-state handoff
- reaction event produces non-empty `AtTpcPoint`
- `AtVertexPropagator` state is populated through the shared detector logic, not macro glue

### 5. Side-by-side contract checks against Geant

Macros:

- `macro/Simulation/AtSimValidation/geant4_fixed.C`
- `macro/Simulation/AtSimValidation/simpleSim_fixed.C`

What to compare:

- event pairing structure
- reaction vertex location distribution
- residual beam energy at reaction
- non-zero reaction product generation
- `AtTpcPoint` occupancy and track IDs

This should happen before any final physics overlay claims.

### 6. Physics validation after contract validation

Only after the shared contract is working:

- compare track topology
- compare stopping behavior
- compare kinematic loci
- compare Bragg-like observables

Do not use physics plots to hide a broken contract.

## Immediate Implementation Order

1. Refactor `AtTpc` to expose a transport-neutral step-processing path.
2. Write unit tests around the shared detector logic and `AtVertexPropagator` handoff.
3. Build a SimpleSim-side transport adapter that feeds that shared path.
4. Replace the current validation macro’s ad hoc assumptions with the shared adapter.
5. Re-run the fixed validation case and confirm non-empty reaction-event output.
6. Only after that revisit documentation of macro migration.

## Current Status Summary

Current verified state:

- the custom collector can reuse generator syntax
- `AtTpc` now exposes a transport-neutral detector-side step path
- unit tests cover detector-trigger logic, vertex handoff state writes, and the new SimpleSim callback transport
- the fixed SimpleSim macro can be made to run with `TGeant3`
- the detector-coupled SimpleSim adapter now produces non-empty beam-event `AtTpcPoint` output through shared detector logic
- the fixed validation macro now triggers detector-side reaction handoff and reaches `AtTPC2Body` with non-zero residual beam energy
- the fixed validation macro now produces non-empty reaction-event `AtTpcPoint` output through the shared path

Current blocker:

- end-to-end detector contract is now working in the fixed validation case
- the remaining follow-up is physics parity and broader comparison against the Geant validation macros
