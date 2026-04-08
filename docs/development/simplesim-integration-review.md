# SimpleSim FairRoot Integration Review

Review of the `SimpleSimAddition` branch, which integrates `AtSimpleSimulation` into the FairRoot simulation pipeline as a drop-in replacement for Geant4 transport.

Scope: integration layer design only. The physics of `AtSimpleSimulation` itself is assumed correct.

## 1. Architectural Overview

The integration layer runs `AtSimpleSimulation` (an RK4/straight-line particle propagator with energy loss) inside FairRoot's event loop, so that its output feeds the same downstream digitization chain (`AtClusterizeTask` -> `AtPulseTask`) as Geant4 transport.

### Abstractions introduced

- **`AtTpc::StepState` + `ProcessStep()`** -- The central architectural move. `AtTpc::ProcessHits()` was refactored to extract all VMC/`gMC` queries into a plain data struct (`StepState`), then delegate to a new `ProcessStep(const StepState&)` method. This decouples the detector's hit-recording and reaction-trigger logic from the VMC transport engine. SimpleSim can call `ProcessStep()` directly without a running VMC.

- **`AtSimParticleCollector`** -- A minimal `FairGenericStack` stub that captures `PushTrack()` calls. This lets existing `FairPrimaryGenerator` + `AtReactionGenerator` chains run unchanged: the generators push particles to what they think is the VMC stack, but the particles land in a simple vector instead.

- **`AtSimpleSimulationTask` (abstract base)** -- Template Method pattern: `Init()` -> `InitEventSource()`, `Exec()` -> `LoadEvent()` -> `TransportCurrentEvent()`, `Finish()` -> `FinishEventSource()`. Owns the simulation, the particle collector, the MCTrack branch, and the detector-stepping logic.

- **`AtSimpleSimulationGeneratorTask`** -- Generates events live via a `FairPrimaryGenerator`.

- **`AtSimpleSimulationReplayTask`** -- Reads primary MCTracks from a prior Geant4 run and re-transports them through SimpleSim, enabling direct A/B comparison.

### What it leaves to the user

FairRunSim boilerplate (cave, geometry, materials, dummy generator, parameter I/O), energy loss model configuration per particle species, and magnetic/electric field setup.

## 2. API Design Assessment

### Discoverability

The user cannot configure and run this without reading source code or the validation macros. There is no documentation of `AtSimpleSimulationGeneratorTask` or `AtSimpleSimulationReplayTask` in the docs directory -- the simulation pipeline doc (`subsystems/simulation-pipeline.md`) still describes SimpleSim as a standalone path, not as a FairTask. The macros in `macro/Simulation/AtSimValidation/` are the de facto documentation.

A user encountering this for the first time must discover:
- That they need `FairRunSim` with a dummy `FairPrimaryGenerator`
- That `SetDetector(tpc)` is required for detector-coupled output
- That the replay task needs a Geant4 truth file with MCTrack branch on a "cbmsim" tree
- That they must configure energy loss models for every particle species they want transported

None of this is documented outside the macros.

### Error surface

- **Forgetting `SetDetector()`:** The task falls through to legacy `SimulateParticle()` mode, which hardcodes "drift_volume" containment and writes its own `AtTpcPoint` branch directly. The user gets output that looks correct but bypasses the detector contract entirely. This should probably be an error or at minimum a warning -- the two paths produce structurally different output.

- **Missing energy loss model:** Throws `std::invalid_argument`, which is caught and logged at `debug` level in `TransportParticle()`. The particle is silently skipped. A user might not notice reaction products disappearing.

- **Geometry mismatch:** The simulation loads its own geometry file (e.g. `ATTPC_He1bar_geomanager.root`) while `FairRunSim` loads geometry for the detector module. If these don't match, volume lookups in SimpleSim and hit positions in `AtTpc` will silently disagree.

- **Replay mode with wrong event count:** If the user requests more events than exist in the source file, `LoadPrimaryTracksFromSource()` returns `false`, `LoadEvent()` returns `hasEvent=false`, and `Exec()` silently returns. FairRoot happily runs empty events.

### Consistency

Mostly consistent. The pattern of `FairTask` subclass + `Init()`/`Exec()` is standard. Using `FairPrimaryGenerator` with a custom stack is clever and preserves the generator API. The `SetDetector()`/`SetSensitiveDetector()` naming follows FairRoot conventions.

However, requiring `FairRunSim` with `run->SetName("TGeant3")` and a dummy generator to use a non-Geant transport is dissonant. The user is setting up Geant3 to not use Geant3.

### Completeness

- No way to set the B-field from a `FairField` object -- the user must manually translate `AtConstField` parameters to `sim->SetMagneticField(XYZVector(...))`. In the Geant4 path, `run->SetField()` handles this.
- No integration with `FairRunSim::SetStoreTraj()`.
- No event header population (event number, vertex, etc.).
- The generator task has no way to set a seed or control reproducibility independently of `gRandom`.

## 3. Separation of Concerns

**Class responsibilities are well-defined.** The base task handles transport mechanics and detector stepping. Subclasses handle only event sourcing. `AtSimParticleCollector` handles only particle capture. This is clean.

**The generator/replay split is well-motivated.** The replay task enables controlled validation against Geant4 with identical kinematics. This is a legitimate use case that would be awkward to express as a mode flag on a single class.

**FairRoot logic is mostly separated from physics logic.** The physics lives in `AtSimpleSimulation`; the FairRoot adaptation lives in the task hierarchy. The boundary is at `TransportParticle()` with its `StepCallback`.

### Entanglement issues

1. **Unit conversion is scattered across the boundary.** `AtSimpleSimulationTask::TransportParticle()` converts cm->mm and GeV->MeV with raw `* 10.` and `* 1000.` factors. `ProcessDetectorStep()` converts back with `/ 10.` and `/ 1000.`. `SubmitInitialSensitivePoint()` does the same. These magic numbers appear in ~15 places across the file. A single wrong factor produces wrong physics with no error.

2. **Sensitive volume identification is duplicated.** `AtSimpleSimulationTask::IsSensitiveVolume()` is a static method with hardcoded volume name checks (`"drift_volume"`, `"window"`, `"cell"`) that duplicates `AtTpc::CheckIfSensitive()`. If the detector geometry adds a new sensitive volume, both must be updated independently.

3. **`AtTpc::ProcessStep()` now embeds transport-termination logic** (stop on exiting reaction volume) that was not in the original Geant4 path. This is a behavioral change visible to all callers of `ProcessStep()`, including `ProcessHits()`, meaning the Geant4 path's behavior was changed too.

## 4. Framework Integration Quality

**The `StepState` refactoring of `AtTpc` is the strongest part of this branch.** It correctly preserves all the fields that `ProcessHits()` used to extract from `gMC`, and the Geant4 path still works through `ProcessHits()` -> populate `StepState` -> `ProcessStep()`. The test coverage of the detector contract (`AtTpcTest.cxx`) is solid.

**Composition with upstream:** The generator task correctly reuses `FairPrimaryGenerator` with `AtSimParticleCollector` as the stack. The beam/reaction alternation via `AtVertexPropagator` is preserved.

**Composition with downstream:** The task writes `AtTpcPoint` and `MCTrack` branches in the same format as the Geant4 path, so `AtClusterizeTask` and `AtPulseTask` should work unchanged.

### Implicit assumptions that could break

- **`correctPosOut()` is only applied in the Geant4 path.** When `ProcessHits()` calls `ProcessStep()`, exit positions have been corrected. When SimpleSim calls `ProcessStep()` directly, they haven't. This means the two paths produce slightly different hit positions near volume boundaries.

- **The replay task hardcodes `fSourceEventIndex % 2 == 0` for beam event detection** (`AtSimpleSimulationReplayTask.cxx:45`). This assumes the source file always uses strict even/odd alternation. If the source was generated with a non-alternating generator, this will misidentify beam events.

- **`RegisterMCTrackBranch()` reuses an existing "MCTrack" branch if one exists** (`AtSimpleSimulationTask.cxx:94-98`). In the replay scenario, the source file has an MCTrack branch, and FairRoot may expose it through `FairRootManager`. The task would then write into someone else's array.

- **LinkDef streamer suffixes are wrong.** The task classes use `+;` (full streamer, for disk-persisted objects) but tasks should use `-!;` per project conventions.

## 5. Tradeoffs and Direction

### Tradeoffs made

1. **Reuse FairRunSim infrastructure vs. standalone run.** The design chose to embed SimpleSim inside `FairRunSim` as a `FairTask`. This provides access to geometry, I/O, and the event loop -- but forces users to set up a dummy VMC engine they're explicitly trying to avoid. A standalone `FairRunAna`-based path would have been cleaner for the user but would have required reimplementing geometry loading.

2. **Detector coupling vs. standalone MCPoint writing.** The task supports both: with `SetDetector()`, it feeds steps through `AtTpc::ProcessStep()`; without, it uses the legacy `SimulateParticle()` path that writes MCPoints directly. This provides flexibility but creates two output formats with subtly different semantics (the detector path accumulates energy loss per track, applies reaction triggers, etc.).

3. **FairPrimaryGenerator reuse via fake stack.** This is the right call. It preserves the full generator ecosystem without modification. The tradeoff is that `AtSimParticleCollector` must implement a large interface of stubs, but the stubs are trivial.

### Assessment

For validation purposes (comparing SimpleSim to Geant4), these tradeoffs are reasonable. For production use by physics users, the FairRunSim boilerplate burden is too high -- but production use isn't the stated goal yet.

This is moving toward a genuine pipeline replacement pattern, not a workaround. The `StepState` abstraction is the right seam. The design would need one more iteration -- extracting the sensitive-volume contract and making the FairRunSim dependency optional -- to be a clean, general-purpose alternative transport.

## 6. Weak Points / Design Smells

In order of severity:

1. **Behavioral change to the Geant4 path.** `AtTpc::ProcessStep()` adds `if (step.exiting && IsReactionVolume(fVolName)) return true;` -- this stops the track when exiting the reaction volume. Previously in `ProcessHits()`, the track was not stopped on exit; only `resetVertex()` was called. Now `ProcessHits()` calls `gMC->StopTrack()` whenever `ProcessStep()` returns true. This changes Geant4 simulation behavior and needs careful validation or should be behind a flag.

2. **Silent fallback to legacy mode.** When `fDetector` is null, the task silently switches to `SimulateParticle()` which writes MCPoints directly with different semantics (no reaction trigger, no beam/reaction alternation, hardcoded "drift_volume" containment). This is a footgun for users who forget `SetDetector()`.

3. **Unit conversion by magic number.** The mm<->cm and MeV<->GeV conversions are scattered across 15+ call sites as raw `* 10.`, `/ 10.`, `* 1000.`, `/ 1000.`. A single wrong factor is a silent physics error. These should be named constants or conversion functions.

4. **Duplicated sensitive volume logic.** `IsSensitiveVolume()` in the task and `CheckIfSensitive()` / `IsReactionVolume()` in the detector are independent implementations of the same concept. They also differ: `IsReactionVolume` checks `drift_volume` and `cell`; `IsSensitiveVolume` also checks `window`. This means the task considers windows sensitive but the detector's reaction logic doesn't -- by design or by accident?

5. **`AtTestSimulation` is now vestigial.** After the refactoring, it's an empty class that inherits `AtSimpleSimulationGeneratorTask` with no additions. The test file accesses its internals via `#define private public`. This class should either be removed (tests use the generator task directly) or given a clear purpose.

## 7. Missed Opportunities

1. **The `StepState` contract could have been an interface.** Instead of a struct on `AtTpc`, `StepState` could live in a shared header and define the transport-neutral contract between any transport engine and any sensitive detector. This would make the pattern reusable beyond `AtTpc`.

2. **`FindSensitiveEntry()` could use the geometry.** The brute-force 1mm linear scan up to 5m is slow and fragile. `TGeoManager::FindNextBoundary()` would find the volume crossing analytically.

3. **A builder or factory for SimpleSim runs** would eliminate the FairRunSim boilerplate. A function like `AtSimpleSimRun::Create(geoFile, generators, elossModels)` that internally sets up FairRunSim with the right dummy objects would make the user-facing API dramatically simpler.

   **Update:** Energy loss model configuration has been addressed. `AtELossModelFactory` (with `AtELossFactoryCATIMA` and `AtELossFactoryBetheBloch` implementations) provides automatic model creation from geometry materials via `AtSimpleSimulation::SetModelFactory()`. See [energy-loss.md](../subsystems/energy-loss.md#model-factories). The FairRunSim boilerplate reduction (builder pattern) remains an open opportunity.

4. **The detector-coupled and standalone modes should be separate paths, not a runtime branch in `Init()`.** The current `if (fDetector != nullptr)` split in `TransportParticle()` combines two fundamentally different output contracts in one method.

## 8. Overall Judgment

**This is a good integration design with one excellent core idea and several execution issues.**

### Where it succeeds

The `StepState` / `ProcessStep()` refactoring of `AtTpc` is the key insight. By extracting all VMC queries into a plain data struct and making the detector's step-processing logic transport-agnostic, this branch creates a clean, testable seam that any future transport engine can target. The `AtSimParticleCollector` adapter is similarly well-conceived -- it lets the full generator ecosystem work unchanged without any modifications to existing generators.

The test coverage is notably good: the detector contract, the task internals, the MCTrack fill logic, and the physics (straight-line and Larmor radius) all have meaningful tests.

### Where it falls short

- The behavioral change to the Geant4 path (stopping on reaction volume exit) is the most consequential issue. It needs to be validated or guarded.
- The user experience is still FairRoot-heavy: users set up a Geant3 run to not use Geant3. A thin convenience layer would make a large difference.
- Unit conversion by magic number across 15+ call sites is a maintenance and correctness risk.
- The dual standalone/detector-coupled modes create an implicit contract that will confuse users.

### Is this sustainable?

Yes, with one more iteration. The `StepState` contract is the right foundation. The task hierarchy is extensible. The main work remaining is: (a) fix the Geant4 behavioral regression, (b) unify or document the sensitive volume logic, (c) centralize unit conversions, and (d) consider a convenience API that hides FairRunSim setup from physics users.

The direction is coherent and this is not a workaround -- it is a genuine architectural step toward transport-engine independence.

---

# SimpleSim Integration Correctness Review

Scope: whether the integration correctly fulfills FairRoot contracts and produces output that downstream tasks can consume without modification. Does not revisit API design or architecture (covered above).

## 1. Framework Contract Compliance

### 1a. Beam/reaction event flag inversion in generator task (CRITICAL)

In `AtSimpleSimulationGeneratorTask::LoadEvent()` (line 33):

```cpp
fPrimGen->GenerateEvent(&fCollector);
state.beamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
```

Inside `GenerateEvent`, `AtReactionGenerator::ReadEvent()` reads the flag, then calls `EndEvent()` which **toggles it** before returning. By the time `LoadEvent` reads the flag, it has been inverted. The result:

- **Event 0 (beam)**: `state.beamEvent = false` (wrong)
- **Event 1 (reaction)**: `state.beamEvent = true` (wrong)

This propagates through the full chain:

```
LoadEvent().beamEvent
  -> TransportCurrentEvent(beamEvent)
    -> TransportParticle(particle, beamEvent)
      -> beamTrack = beamEvent && particle.trackID == 0
        -> StepState.beamTrack
          -> AtTpc::ProcessStep -> fIsBeamTrack
```

`fIsBeamTrack` controls:
- `trackEnteringVolume`: whether `InPos` (beam entry position) is recorded
- `getTrackParametersWhileExiting`: whether `resetVertex()` is called on beam exit
- `reactionOccursHere()`: `isPrimaryBeam = fIsBeamTrack` -- gates the reaction trigger entirely

With the flag inverted:
- **Event 0 (beam)**: `fIsBeamTrack=false` -> reaction never fires -> vertex never set
- **Event 1 (reaction)**: `fIsBeamTrack=true` for trackID=0 -> reaction fires for FairBoxGenerator's beam particle -> calls `ResetVertex()` then `SetVertex()` at the wrong position

The vertex propagation chain breaks. `AtTPC2Body::GenerateReaction` on Event 1 reads the vertex from `AtVertexPropagator` -- which was never properly set by Event 0. Products start from wrong positions.

**Fix**: Capture the flag before calling `GenerateEvent()`:

```cpp
bool wasBeamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
fPrimGen->GenerateEvent(&fCollector);
state.beamEvent = wasBeamEvent;
```

**Note**: The validation macros use `AtSimpleSimulationReplayTask`, which sets the flag explicitly and avoids this bug. The generator task is not exercised by any macro on this branch.

### 1b. ProcessStep exit-stop changes Geant4 behavior

`AtTpc::ProcessStep()` (line 235-236) adds:

```cpp
if (step.exiting && IsReactionVolume(fVolName))
   return true;
```

This stops **all** particles on exit from drift_volume/cell, not just the beam. In the old code, only `startReactionEvent()` called `gMC->StopTrack()` (beam-only). Now in the Geant4 path, reaction products exiting the active gas are also stopped. While `AtClusterize` only processes drift_volume hits (so this is benign for standard digitization), it changes Geant4 behavior for any analysis reading raw `AtTpcPoint` data that expects window-region hits from exiting products.

### 1c. MCTrack branch registration

`AtSimpleSimulationTask::RegisterMCTrackBranch()` checks for an existing `MCTrack` branch and reuses it, or creates one. In the FairRunSim setup, `AtStack` also registers `MCTrack`. The task handles this correctly (line 94-98), reusing the existing branch if present.

## 2. Integration Correctness

### 2a. Unit conversions -- correct

All conversion points were traced:

| Direction | Quantity | Conversion | Location |
|-----------|----------|------------|----------|
| Collector -> SimpleSim | position | cm x 10 -> mm | `TransportParticle` L130 |
| Collector -> SimpleSim | momentum | GeV x 1000 -> MeV | `TransportParticle` L131 |
| SimpleSim -> StepState | energyLoss | MeV / 1000 -> GeV | `ProcessDetectorStep` L215 |
| SimpleSim -> StepState | trackLength | mm / 10 -> cm | `ProcessDetectorStep` L216 |
| SimpleSim -> StepState | position | mm / 10 -> cm | `ProcessDetectorStep` L223 |
| SimpleSim -> StepState | momentum | MeV / 1000 -> GeV | `ProcessDetectorStep` L224 |
| SimpleSim -> StepState | totalEnergy | MeV / 1000 -> GeV | `ProcessDetectorStep` L221 |

All conversions are consistent and correct. The `StepState` unit annotations (`// GeV`, `// cm`, etc.) match the values produced.

### 2b. Generator invocation

`AtSimParticleCollector` inherits `FairGenericStack` and intercepts `PushTrack()`. This captures all the particles that generators would normally push onto the VMC stack, preserving their cm/GeV units. The 18-param and 19-param `PushTrack` overloads are both handled.

`AtSimParticleCollector` does not support `PopNextTrack` / `PopPrimaryForTracking` (return nullptr) or `GetCurrentTrack` (returns nullptr). These are acceptable since no VMC transport runs, but could break generators that call `GetStack()->GetCurrentTrack()` during `ReadEvent`.

### 2c. Edge cases

**Particles starting outside the active volume**: `FindSensitiveEntry()` probes forward in 1 mm steps along the momentum direction, up to 5 m. This is reasonable but imprecise -- the entry point could miss by up to 1 mm, and energy loss in non-sensitive material between the actual start and the probe hit is not accounted for (SimpleSim applies the same dE/dx model everywhere).

**Missing energy-loss model**: `TransportParticle` catches `std::invalid_argument` and logs at `debug` level. The particle is silently skipped. A user might not notice reaction products disappearing.

**Particles that never stop**: The curved-track path has a `kMaxCurvedTransportSteps = 200000` guard. The straight-line path stops when exiting the geometry or KE < 1 keV. Both are adequate.

**Zero-length tracks**: `SubmitInitialSensitivePoint` creates a hit with `energyLoss=0` and `trackLength=0`. `AtClusterize` skips zero-loss hits, so this contributes no electrons. It correctly sets up entering-volume state in `AtTpc` without producing a physics contribution.

### 2d. `correctPosOut()` not applied in SimpleSim path

In the Geant4 `ProcessHits`, exit positions are refined using `TGeoManager` boundary correction (`correctPosOut()`). In the SimpleSim path, exit positions come directly from the propagator's last step with no boundary refinement. This causes minor position imprecision at volume boundaries (up to one step size, typically ~1 mm).

### 2e. Time field is always zero

`ProcessDetectorStep` sets `detectorStep.timeNs = 0.` for all steps. `AtClusterize` does not use the time field, so this doesn't affect current downstream processing. Any analysis reading `AtMCPoint::GetTime()` would see zero.

### 2f. Replay task doesn't set AtVertexPropagator track metadata

In the Geant4 path, `AtTPC2Body` populates `SetTrackEnergy(trackID, ...)` and `SetTrackAngle(trackID, ...)`. The replay task reads raw MCTracks and pushes them directly to the collector without running any generator, so these are never set. `AtTpc::addHit()` reads them for `EIni` / `AIni` fields in `AtMCPoint` -- they'll be 0.0 for all points. `AtClusterize` doesn't use them, but any analysis reading these fields sees wrong values.

## 3. Pipeline Trace

Tracing a single primary proton through the SimpleSim **generator** path (illustrating the flag bug from 1a):

1. **FairPrimaryGenerator::GenerateEvent** runs generators. `FairBoxGenerator` pushes the beam particle (trackID=0). `AtTPC2Body::GenerateReaction` reads vertex from `AtVertexPropagator`, computes kinematics, pushes products (trackID=1,2). `AtReactionGenerator::EndEvent` toggles the flag.

2. **LoadEvent** reads the inverted flag. On Event 0 (beam): `beamEvent=false`.

3. **TransportCurrentEvent(false)**: For the beam (trackID=0): `beamTrack = false && (0==0) = false`.

4. **FindSensitiveEntry**: If the beam starts outside the drift volume (typical -- generated at z=-100 cm), probes forward to find the entry.

5. **SubmitInitialSensitivePoint**: Creates an entering `StepState` with `beamTrack=false`. `AtTpc::ProcessStep` sets `fIsBeamTrack=false`, calls `trackEnteringVolume` (but `InPos` is NOT set because `fIsBeamTrack` is false).

6. **TransportParticle callback**: For each RK4 step inside the drift volume, `ProcessDetectorStep` builds a `StepState` (units converted correctly) and calls `ProcessStep`. Energy loss accumulates in `fELossAcc`.

7. **reactionOccursHere**: `isPrimaryBeam = fIsBeamTrack = false` -> **never fires**. The beam particle traverses the entire volume or stops without setting the vertex.

8. **Exit**: When the proton exits the drift volume, `step.exiting && IsReactionVolume` -> `ProcessStep` returns true -> transport stops.

9. **Vertex**: `AtVertexPropagator::SetVertex()` is never called. The vertex remains at default (0,0,0).

10. **Event 1 (reaction)**: `AtTPC2Body::GenerateReaction` reads vertex -> (0,0,0) -> products start from wrong position.

For the **replay** path, this trace does not apply -- the replay task sets the beam flag explicitly and reads particle kinematics from the Geant4 truth file, bypassing the vertex propagation chain entirely.

## 4. Failure Mode Assessment

### Flag inversion (generator task only)

**What breaks**: The reaction vertex is never set on beam events and incorrectly triggered on reaction events. Products start from wrong positions.

**Sensitivity**: Every downstream observable that depends on vertex position -- track angles, reaction kinematics, Q-value reconstruction.

**Visibility**: **Silently wrong**. The simulation runs to completion and produces output with correct-looking structure but wrong physics. A comparison with Geant4 truth would reveal it immediately (vertex z mismatch), but a standalone run would not obviously fail.

**Mitigation on this branch**: Both validation macros use the `ReplayTask` (which avoids the bug), so the current validation pipeline does not exercise this failure.

### ProcessStep exit-stop in Geant4 path

**What breaks**: Reaction products stopped at the drift volume boundary. Secondaries from stopped products are lost.

**Sensitivity**: Low for standard digitization (only drift_volume hits used). Could matter for efficiency studies or background estimates.

**Visibility**: Visible as fewer hits in boundary volumes (`window`) when comparing old vs. new Geant4 output.

## 5. High-Risk Findings

### 1. Beam/reaction event flag inversion in generator task

- **What**: `LoadEvent` reads `IsBeamEvent()` after `EndEvent()` has toggled it inside `GenerateEvent()`, yielding the inverted value.
- **Why**: `AtReactionGenerator::ReadEvent()` calls `EndEvent()` before returning to `GenerateEvent()`, but `LoadEvent` reads the flag after `GenerateEvent()` returns.
- **Impact**: Vertex never set, reaction products at wrong positions, silently wrong physics.
- **Confidence**: Very high -- mechanically verified from the call sequence.
- **File**: `AtSimpleSimulationGeneratorTask.cxx:33`

### 2. ProcessStep stops all particles exiting reaction volumes (Geant4 side-effect)

- **What**: The `step.exiting && IsReactionVolume(fVolName)` return in `ProcessStep` applies to both SimpleSim and Geant4 paths.
- **Why**: `ProcessStep` is shared code, but this exit-stop was added for SimpleSim transport semantics.
- **Impact**: Non-beam Geant4 tracks stopped at drift volume boundary; minor data loss for boundary analyses.
- **Confidence**: High -- visible in the diff and `ProcessHits` calls `ProcessStep`.
- **File**: `AtTpc.cxx:235-236`

### 3. Replay task doesn't populate AtVertexPropagator track energy/angle

- **What**: `SetTrackEnergy` / `SetTrackAngle` are never called in the replay path.
- **Why**: No generator runs; particles are read from file.
- **Impact**: `EIni` / `AIni` fields in `AtMCPoint` are 0.0. Not used by digitization, but wrong for direct analysis which is not an issue for this use case.
- **Confidence**: High.
- **File**: `AtSimpleSimulationReplayTask.cxx:40-55`, `AtTpc.cxx:261-280`
- **Do not patch - not an issue**

### 4. Sensitive volume identification is duplicated and divergent

- **What**: `AtSimpleSimulationTask::IsSensitiveVolume()` checks `"drift_volume"`, `"window"`, `"cell"`. `AtTpc::IsReactionVolume()` checks only `"drift_volume"` and `"cell"`. `AtTpc::CheckIfSensitive()` checks all three.
- **Why**: Three independent implementations of the same concept.
- **Impact**: If detector geometry adds a new sensitive volume, all three must be updated independently.
- **Confidence**: Medium -- the current set of volumes is consistent, but the maintenance risk is real.

## 6. Suggested Validation Tests

### For Finding #1 (flag inversion)

**Unit test**: Create an `AtSimpleSimulationGeneratorTask` with a generator that calls `EndEvent()`. Verify that `LoadEvent().beamEvent` matches the pre-toggle flag value:

```
Event 0: expect beamEvent == true  (currently returns false)
Event 1: expect beamEvent == false (currently returns true)
```

**Integration test**: Run the generator task with `AtTPC2Body` and verify that `AtVertexPropagator::GetVz()` is non-zero after the beam event's transport completes.

### For Finding #2 (exit-stop behavior)

**A/B comparison**: Run the Geant4 path before and after this branch. Count `AtTpcPoint` entries with `GetVolName() == "window"` from non-beam tracks. The new code should produce fewer (or zero) such hits.

### For Finding #3 (missing metadata)

**Comparison test**: Compare `AtMCPoint::GetEIni()` and `GetAIni()` between Geant4 output and replay-task output for the same events. Geant4 should have non-zero values; replay should have all zeros.

### For general output correctness

**Bragg curve comparison**: For a fixed-angle single event, compare the dE/dx vs. range profile between Geant4 and SimpleSim outputs. This catches unit errors, energy loss model discrepancies, and step-size artifacts simultaneously. The existing `compareFixed.C` and `compareKinematic.C` macros appear designed for this.

**Vertex position match**: For the generator task (once the flag bug is fixed), verify that the vertex Z from `AtVertexPropagator::GetVz()` matches between Geant4 and SimpleSim to within the step-size uncertainty (~1 mm).
