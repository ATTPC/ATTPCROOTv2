# SimpleSim FairRoot Integration Review

> **Historical note (post-review rename):** the class names below were renamed after this review.
> `AtSimpleSimulation` (transport engine) → `AtSimTransport`;
> `AtStandaloneSimulation` (hit-recording wrapper) → `AtSimpleSimulation`;
> `AtSimpleSimulationTask` / `AtSimpleSimulationGeneratorTask` / `AtSimpleSimulationReplayTask` → `AtSimTransportTask` / `AtSimTransportGeneratorTask` / `AtSimTransportReplayTask`.
> `AtELossModelFactory` / `AtELossFactoryBetheBloch` / `AtELossFactoryCATIMA` → `AtELossManager` / `AtELossManagerBetheBloch` / `AtELossManagerCATIMA`.
> The review text below is preserved with the original names for historical context.

Review of the `SimpleSimAddition` branch, which integrates `AtSimpleSimulation` into the FairRoot simulation pipeline as a drop-in replacement for Geant4 transport.

Scope: integration layer design only. The physics of `AtSimpleSimulation` itself is assumed correct.

## 1. Architectural Overview

The integration replaces Geant4 transport with AtSimpleSimulation's RK4 propagator while reusing the rest of the FairRoot simulation pipeline (geometry, generators, detector hit recording, digitization).

Three key abstractions bridge the gap:

1. **`AtSimParticleCollector`** -- a `FairGenericStack` stub that intercepts `PushTrack()` calls from `FairPrimaryGenerator::GenerateEvent()`. Generators run unchanged; their output lands in a vector instead of going to Geant4.

2. **`AtSimpleSimulationTask`** (abstract base) -- a `FairTask` that orchestrates event flow: load particles from a source, transport them through AtSimpleSimulation, and feed each step to `AtTpc::ProcessStep()` for hit recording.

3. **`AtTpc::StepState` + `ProcessStep()`** -- the detector was refactored to extract a transport-neutral step contract. Both Geant4 (`ProcessHits`) and SimpleSim feed the same internal logic via this struct.

The design solves: letting physics users swap Geant4 for a fast, controllable transport without changing their generators, geometry, or downstream analysis.

Left to the user: choosing a generator, providing an energy loss model (or factory), and wiring the macro. The physics configuration (E/B fields, step sizes) is auto-extracted from FairRun where possible.

## 2. API Design Assessment

### Discoverability

Good, with one gap. The generator and replay tasks have clean, small interfaces. A user who has seen a Geant4 macro can follow the pattern: create simulation, create task, set detector, set generator, run. The model factory auto-creates energy loss models from geometry materials, which is the right default -- users shouldn't need to manually register models per species.

However, the `SetSensitiveDetector(AtTpc*)` requirement is non-obvious. A user who forgets it gets a fatal at `Init()` -- which is correct -- but the error message is the only documentation of this requirement. The alias `SetDetector()` helps.

### Error surface

Mostly narrow, one silent footgun.

- Missing energy loss model: clear `invalid_argument` exception.
- Particle outside geometry: clear exception.
- Missing detector: fatal at Init with descriptive message.
- **Footgun:** `AtSimpleSimulationReplayTask::LoadEvent()` hardcodes `beamEvent = (eventIndex % 2 == 0)`. This is a fragile heuristic that silently produces wrong beam/reaction event pairing if the source file doesn't alternate. A physics user replaying their own Geant4 output would not expect this behavior and would get silently wrong results.

### Consistency

Strong. Follows the `FairTask` pattern exactly: `Init()` -> `Exec()` -> `Finish()`. Uses `FairPrimaryGenerator` and `FairRootManager` the same way Geant4 simulations do. The generator task is particularly clean -- a user familiar with FairRoot would recognize the pattern immediately.

### Completeness

Two gaps:

1. No electric field auto-configuration (only magnetic field is extracted from FairRun). The `ConfigureFieldFromFairRun` method samples B but not E, though `SetElectricField()` exists. This is probably fine for now (E-field is a drift field, not a transport field), but worth noting.
2. No way to set per-particle energy loss models from the task level without reaching through `GetSimulation()`. The factory pattern mostly eliminates this need, but direct model registration requires breaking the abstraction: `task->GetSimulation()->AddModel(...)`.

## 3. Separation of Concerns

The split between GeneratorTask and ReplayTask is well-motivated. They share the core transport loop (base class) but differ only in how particles are sourced. This is the right seam.

AtSimParticleCollector is clean. It implements just enough of `FairGenericStack` to capture generator output. The pure-virtual stubs are no-ops with appropriate comments. It avoids pulling in VMC/Geant4 dependencies.

FairRoot-specific logic is well-separated from physics logic. `AtSimpleSimulation` knows nothing about `FairTask`, `FairRun`, or `AtTpc`. It deals in mm/MeV coordinates and callbacks. The unit conversion (mm<->cm, MeV<->GeV) lives entirely in `AtSimpleSimulationTask`, which is the right place.

One entanglement: `AtSimpleSimulation` still depends on `FairRootManager` in `RegisterBranch()` -- this is the legacy standalone API and doesn't affect the new integration path, but it means the physics class isn't fully decoupled from FairRoot.

## 4. Framework Integration Quality

The detector step contract is the strongest part of this design. By extracting `AtTpc::ProcessStep(StepState)` and routing both Geant4 and SimpleSim through it, the integration ensures hit recording logic stays in one place. This is exactly the right pattern -- the detector doesn't care who drives transport.

Pipeline composition is correct. The task reads nothing from upstream `FairRootManager` branches; it writes `MCTrack` and feeds `AtTpc` directly. Downstream digitization tasks (`AtClusterizeTask`, etc.) consume `AtTpcPoint` from the detector's collection, which is populated identically regardless of transport engine.

### Implicit assumptions that could break

- `ConfigureFieldFromFairRun()` calls `gGeoManager->FindVolumeFast("drift_volume")` -- hardcoded volume name. If the geometry uses a different naming convention, the field auto-config silently falls back to sampling at (0,0,0), which may be wrong.
- `fDetector->SetStopOnReactionVolumeExit(true)` is called unconditionally in `Init()`. This changes AtTpc behavior globally. If another task also uses the detector (unlikely but possible), this side effect is invisible.
- `IsSensitiveVolume` is a static method checking for hardcoded volume name substrings ("drift_volume", "window", "cell"). This is a framework-wide convention, not something introduced here, but the integration depends on it critically for stepping logic.

## 5. Tradeoffs and Direction

### Key tradeoffs made

| Decision | In favor of | At the cost of |
|---|---|---|
| Uniform field assumption | Simplicity, speed | Cannot handle field maps |
| Single-point field sampling | Drop-in behavior | Wrong if field varies spatially |
| Factory auto-creates models | User doesn't register per species | Factory must handle all materials correctly |
| Callback-based stepping | Clean separation from detector | Slightly more complex internal flow |
| `ELossModelShared` wrapper | Shared ownership in SimpleSim, unique in Propagator | Extra indirection, wrapper class |

These are appropriate. The uniform field assumption is reasonable for AT-TPC (the solenoidal field is approximately uniform in the drift volume). The factory pattern is the right answer for a multi-species simulation. The callback design is the right structural choice for keeping transport and detection separate.

This is a clean pipeline replacement pattern, not a workaround. The detector step contract means future transport engines could be plugged in the same way. The design is coherent and sustainable.

## 6. Weak Points / Design Smells

In order of importance:

1. **Replay task beam-event heuristic.** `beamEvent = (eventIndex % 2 == 0)` in `AtSimpleSimulationReplayTask::LoadEvent()` is an implicit contract that will silently produce wrong physics. The source file may not follow this convention. This should either be read from the source file metadata or made configurable.

2. **`ELossModelShared` wrapper class.** AtPropagator requires `unique_ptr<AtELossModel>` but AtSimpleSimulation holds `shared_ptr`. The wrapper is a workaround for a design mismatch. The real fix is to have AtPropagator accept a non-owning reference or `shared_ptr` -- the current pattern creates a fake `unique_ptr` that secretly shares ownership, which violates the semantic contract of `unique_ptr`.

3. **Field auto-config hardcodes "drift_volume".** The volume name is also hardcoded in `fStandaloneVolumeName`. These should be the same string by construction, but they're set independently and could diverge.

4. **`RegisterBranch()` on the physics class.** FairRootManager coupling in AtSimpleSimulation is vestigial. The integration path doesn't use it, but a user might call it accidentally, creating confusion about which path records hits.

5. **Thread-local `fMCPoints` and `fTrackID`.** These exist for the standalone API but have no role in the detector-coupled path. They add cognitive load and a footgun -- if someone calls `SimulateParticle()` and `TransportParticle()` in the same event, track IDs will collide.

## 7. Missed Opportunities

1. **The `AtELossModelFactory` could have been an argument to the `AtSimpleSimulationTask` constructor** rather than a `SetModelFactory()` call. Since it's always needed (or the user must register models manually), making it a constructor parameter would make the requirement explicit and eliminate a misconfiguration path.

2. **`FindSensitiveEntry()` walks in 1mm steps up to 5m.** A TGeo ray-trace (`FindNextBoundary`) would be exact, faster, and wouldn't miss thin volumes. The linear scan is the obvious implementation but the geometry manager already solves this problem.

3. **The `TransportStep` struct duplicates `AtTpc::StepState`** with different units and types (`std::string` vs `TString`, mm/MeV vs cm/GeV). A shared step type with unit-tagged fields, or a conversion constructor, would make the mapping less error-prone and reduce the 30-line `ProcessDetectorStep` method to a few lines.

## 8. Overall Judgment

This is a good integration design. The core architectural decisions -- intercepting the generator stack, routing transport through a callback, unifying detector logic through `ProcessStep` -- are sound. The separation between physics (AtSimpleSimulation), pipeline orchestration (AtSimpleSimulationTask), and event sourcing (Generator/Replay subclasses) is clean and well-motivated.

### Where it succeeds

- The detector step contract is the design's best contribution. It makes the transport engine pluggable without the detector caring.
- AtSimParticleCollector is an elegant solution for bypassing VMC without rewriting generators.
- The factory pattern for energy loss models is the right abstraction for multi-species physics.
- Unit conversion is concentrated in one place (the task), not scattered.

### Where it falls short

- The replay task's beam-event heuristic is a latent correctness bug.
- The `ELossModelShared` wrapper papers over an ownership design mismatch rather than fixing it.
- There are two parallel hit-recording paths (standalone `AddHit` vs detector-coupled `ProcessStep`) sharing a class, which muddies the API boundary.

### Is this sustainable?

Yes. The `StepState` contract is the right foundation. The task hierarchy is extensible. The weak points are real but fixable without structural changes. The biggest risk for users is the replay beam-event logic; the biggest risk for maintainers is the dual standalone/detector-coupled API living on the same class.

The direction is coherent and this is not a workaround -- it is a genuine architectural step toward transport-engine independence.

---

# SimpleSim Integration Correctness Review

Review of the `SimpleSimAddition` branch, focused on whether the integration correctly fulfills FairRoot framework contracts and produces output that downstream tasks can consume without modification.

Scope: correctness of the integration layer. The physics of `AtSimpleSimulation` and the design quality are reviewed separately above.

## 1. Integration Summary

### Geant4 path (framework level)

`FairPrimaryGenerator` pushes particles onto `AtStack`. Geant4 transports them step-by-step through the ROOT geometry. At each step inside a sensitive volume, `AtTpc::ProcessHits()` extracts VMC state into a `StepState`, delegates to `ProcessStep()`, and records `AtMCPoint` entries into a `TClonesArray` registered as the `AtTpcPoint` branch. The `MCTrack` branch is filled by `AtStack`. Beam/reaction alternation is controlled by `AtVertexPropagator` -- the beam event accumulates energy loss until a reaction threshold (`RndELoss`) is reached, at which point `startReactionEvent()` populates the vertex state for the subsequent reaction event.

### SimpleSim path

`AtSimpleSimulationTask` (a `FairTask`) runs inside the same `FairRunSim` event loop, after a no-op Geant4 transport (dummy generator produces zero primaries). On each `Exec()`:

1. `LoadEvent()` runs generators through the same `FairPrimaryGenerator::GenerateEvent()` machinery, capturing particles into an `AtSimParticleCollector`.
2. Particles are transported through the geometry via `AtSimpleSimulation::TransportParticle()`, with each step delivered to `AtTpc::ProcessStep()` through the same `StepState` struct used by the Geant4 path.
3. `MCTrack` is filled by the task itself from the collector.

### Integration layer responsibilities

- Unit conversion between SimpleSim internals (mm/MeV) and FairRoot conventions (cm/GeV)
- Mapping transport steps to the `AtTpc::StepState` struct
- Driving generators and capturing primaries without a real VMC stack
- Determining entering/exiting/stopping flags from volume boundary crossings
- Identifying beam vs. reaction tracks

## 2. Framework Contract Compliance

**FairTask lifecycle -- correct.** `Init()` configures the detector, auto-extracts the B-field from FairRun, initializes the event source, and registers the MCTrack branch. `Exec()` runs per event. `Finish()` cleans up. No issues.

**Branch naming and types -- correct.** `AtTpcPoint` is registered by `AtTpc::Register()` (called by FairRunSim detector initialization). `MCTrack` is registered by the task (`AtSimpleSimulationTask.cxx:208`). Both match the Geant4 path naming.

**AtTpcPoint production -- correct with one caveat.** The detector-coupled path feeds steps through `AtTpc::ProcessStep()`, which calls `addHit()` exactly as in the Geant4 path. All fields (position, momentum, energy loss, A, Z, EIni, AIni) are populated by the same detector code. The one issue is the `entering` flag (see Finding 1 below).

**AtVertexPropagator state -- correct for the primary use case.** `AtTPCIonGenerator` only adds beam on beam events when `fDoReact=true` (`AtTPCIonGenerator.cxx:157-170`). The reaction trigger fires correctly during beam transport: `ProcessStep` -> `reactionOccursHere()` -> `startReactionEvent()` -> `SetVertex()`. The subsequent reaction event reads vertex/momentum from the propagator through the same generator chain.

**Detector hook management -- correct but AtTpc-specific.** `SetDetector(tpc)` is required and validated in `Init()`. `ProcessStep()` is called on the same `AtTpc` instance. `SetStopOnReactionVolumeExit(true)` is set to prevent transport beyond the active volume. `IsSensitiveVolume` is hardcoded to `AtTpc::IsSensitiveVolume`, so the task is AtTpc-specific. Other detectors would need to implement the same `ProcessStep(StepState)` interface.

## 3. Integration Correctness

### Finding 1 (HIGH): `entering` flag misses inter-sensitive-volume boundaries

**Location:** `AtSimpleSimulationTask.cxx:255`

```cpp
const bool entering = !preSensitive && postSensitive;
const bool exiting = preSensitive && !postSensitive;
```

This only detects transitions from non-sensitive to sensitive. When a particle crosses between two *different* sensitive volumes (e.g. window -> drift_volume), both are sensitive, so `entering = false`.

In Geant4, `gMC->IsTrackEntering()` is true at every volume boundary, including sensitive-to-sensitive transitions. `AtTpc::trackEnteringVolume()` (`AtTpc.cxx:63`) fires on each entering, resetting `fELossAcc` and capturing `InPos` for beam tracks.

**Consequences:**

1. **`fELossAcc` not reset at drift_volume entry** -- energy loss accumulated in the window is carried into the drift_volume's reaction threshold check. For typical AT-TPC windows (5-50 um Mylar), this is ~0.01-0.1 MeV, negligible vs. the ~10+ MeV reaction threshold. **Low practical impact.**
2. **`InPos` never set for beam tracks** -- `trackEnteringVolume` sets `InPos` only when `fIsBeamTrack && IsReactionVolume(fVolName)` (line 75). Since `trackEnteringVolume` never fires at the drift_volume entry, `InPos` stays at its default `(0,0,0,0)`. This propagates into `SetVertex()` as the input vertex `(invx, invy, invz)`. However, `GetInVx/GetInVy/GetInVz` are only used in *commented-out* code in `AtTPC2Body.cxx` (lines 327-335). **No active downstream consumer, but a latent correctness issue.**

**Suggested fix:**

```cpp
const bool volumeChanged = step.preVolumeName != step.postVolumeName;
const bool entering = (!preSensitive && postSensitive) || (volumeChanged && postSensitive);
const bool exiting = (preSensitive && !postSensitive) || (volumeChanged && preSensitive);
```

**Confidence: High** -- the mechanism is clear from code inspection.

### Finding 2 (MEDIUM): `timeNs` always zero

**Location:** `AtSimpleSimulationTask.cxx:284,314`

Both `SubmitInitialSensitivePoint` and `ProcessDetectorStep` set `detectorStep.timeNs = 0.0`. In Geant4, this is `gMC->TrackTime() * 1e9` (nanoseconds). `AtMCPoint` stores this as the time field. Any downstream code that uses the time field from MC points (e.g. timing resolution studies) will silently get zeros.

`AtClusterizeTask` and `AtPulseTask` compute drift time from position, so the main digitization chain is unaffected.

**Confidence: High.**

### Finding 3 (LOW): Mass inconsistency between generator and transport in straight-line path

**Location:** `AtSimpleSimulation.cxx:312,323-326`

In the straight-line propagation path:

```cpp
double KE = mom.E() - mom.M();           // mom.M() = PDG mass (from generator's E^2 - p^2)
double eLoss = model->GetEnergyLoss(KE, fDistStep);  // model expects KE relative to model mass
auto E = mom.E() - eLoss;
double p = sqrt(E * E - mom.M2());       // uses PDG mass again
```

The 4-vector's invariant mass comes from the generator (PDG mass), while `info.mass` in `ParticleInfo` comes from `AddModel(Z, A, model, massAmu)` using `massAmu * 931.494 MeV/c^2`. For protons: PDG mass = 938.272 MeV/c^2 vs. 1.0078 amu x 931.494 = 938.783 MeV/c^2. The ~0.5 MeV difference produces ~0.05% KE offset.

The curved path correctly uses `info.mass` throughout via `AtTools::Kinematics::KE(momentum, info.mass)`.

**Confidence: Medium** -- the mass difference is real but the practical impact is small.

### Finding 4 (LOW): MCTrack metadata sparse

**Location:** `AtSimpleSimulationTask.cxx:218`

```cpp
new ((*fMCTrackArray)[particle.trackID]) AtMCTrack(particle.pdgCode, -1,
    particle.px, particle.py, particle.pz,
    particle.vx, particle.vy, particle.vz, 0.0, 0);
```

`parentID = -1`, `time = 0`, `nPoints = 0` for all tracks. In the Geant4 path, `AtStack` fills these with correct values. Code that distinguishes primary from secondary via `parentID == -1` works (both paths use -1 for primaries), but code checking `nPoints` or birth time would get wrong values.

**Confidence: High.**

### Finding 5 (LOW): No `correctPosOut()` equivalent

**Location:** `AtTpc.cxx:131-158` vs `AtSimpleSimulationTask.cxx:296-331`

In the Geant4 path, `ProcessHits` calls `correctPosOut()` to adjust exit positions to the precise volume boundary using the geometry navigator's safety distance. SimpleSim's exit position is wherever the last step landed, which may overshoot by up to one step size (~1 mm). This affects the last MC point of each track.

**Confidence: Medium.**

### Unit conversions -- correct throughout

The cm<->mm and GeV<->MeV conversions in `TransportParticle` (lines 235-237) and `ProcessDetectorStep` (lines 313-327) are applied consistently. `step.trackMass` is correctly converted from MeV/c^2 to GeV/c^2. Each field in `StepState` was verified against the Geant4 `ProcessHits` path.

### Generator invocation -- correct

`FairPrimaryGenerator::GenerateEvent(&fCollector)` correctly drives the same generator chain. The collector captures the same particles that would land on the VMC stack. `AtReactionGenerator::ReadEvent` handles alternation as usual. The `wasBeamEvent` capture before `GenerateEvent` (`AtSimpleSimulationGeneratorTask.cxx:32`) correctly identifies the event type before the internal `EndEvent()` toggle.

## 4. Pipeline Trace

**Single primary particle: proton from AtTPC2Body reaction**

1. **Generator invocation** (reaction event): `LoadEvent()` calls `fPrimGen->GenerateEvent(&fCollector)`. `AtTPC2Body::GenerateReaction()` reads vertex/momentum from `AtVertexPropagator` (set during the prior beam event), computes 2-body kinematics, calls `primGen->AddTrack()`. The collector's `PushTrack` stores `{trackID=0, pdg, px, py, pz, e, vx, vy, vz}` in GeV/cm.

2. **Unit conversion**: `TransportParticle` converts to mm/MeV: `pos = vertex * 10`, `mom = (p*1000, E*1000)`.

3. **Sensitive entry**: Vertex is inside drift_volume (that's where the reaction happened). `IsSensitiveVolume` returns true, no `FindSensitiveEntry` needed.

4. **Initial point**: `SubmitInitialSensitivePoint` sends an entering `StepState` to `AtTpc::ProcessStep`. `trackEnteringVolume` resets `fELossAcc`, captures position/momentum. Since `beamTrack=false`, `InPos` is not updated (correct -- products don't need it). `addHit` records the initial MC point.

5. **Transport loop**: `AtSimpleSimulation::TransportParticle` drives the RK4 propagator (if B != 0) or straight-line stepper. Each step: compute energy loss from `AtELossModel`, advance position/momentum, invoke callback.

6. **Step processing**: Callback calls `ProcessDetectorStep` -> `AtTpc::ProcessStep`. `getTrackParametersFromStep` accumulates `fELossAcc`. `addHit` writes an `AtMCPoint` to the detector's `fAtTpcPointCollection` with position (cm), momentum (GeV/c), energy loss (GeV), track length (cm), A, Z, EIni, AIni.

7. **Exit**: When the proton exits drift_volume to non-sensitive material: `exiting = true`, callback returns `false`, transport stops.

8. **Tree fill**: After `Exec()` returns, `FairMCApplication::FinishEvent()` calls `FairRootManager::Fill()`, writing the `AtTpcPoint` and `MCTrack` branches. Then `AtTpc::EndOfEvent()` clears the collection.

**Where output could differ from Geant4:**

- **Energy loss**: SimpleSim uses a single `AtELossModel` per species; Geant4 models discrete interactions, delta rays, straggling. The overall dE/dx curve should match for CATIMA models but individual point-by-point energy deposits will differ.
- **Step granularity**: SimpleSim's step size is controlled by `fDistStep` (straight line) or `fMaxPropStep` (RK4). Geant4 has its own stepping. Different step counts means different numbers of MC points.
- **No secondary particles**: SimpleSim doesn't produce delta rays, photons, or nuclear fragments.
- **No multiple scattering**: SimpleSim follows the energy-loss direction only. No lateral straggling.
- **Time field is always 0** (Finding 2).

## 5. Failure Mode Assessment

| Failure | Downstream impact | Visibility |
|---------|------------------|------------|
| Missing `entering` at drift_volume (Finding 1) | Reaction vertex slightly early due to window energy in fELossAcc. InPos stale. | **Silent** -- shifts are tiny, InPos unused in active code |
| Zero time in MCPoints (Finding 2) | Any timing analysis on MC truth gives 0 | **Visible** if time is plotted, **silent** otherwise |
| Mass mismatch in straight-line path (Finding 3) | ~0.05% KE bias on initial energy, propagates through stopping range | **Silent** -- within model uncertainty |
| MCTrack metadata sparse (Finding 4) | nPoints and birth time wrong | **Silent** unless explicitly checked |
| No correctPosOut (Finding 5) | Last MC point per track overshoots boundary by up to 1 step | **Silent** -- small positional error |

**Most sensitive downstream stages:**

- **AtClusterizeTask** output depends on the spatial distribution of energy deposits. Missing delta rays and different step sizes produce quantitatively different cluster distributions.
- **AtPulseTask** produces traces from clusters. Differences propagate to simulated pad responses but the overall pattern (track shape, energy scale) is preserved.
- **Reconstruction** should work correctly -- the track topology is preserved.

## 6. High-Risk Findings (Top 5)

1. **`entering` flag misses sensitive-to-sensitive volume boundaries** -- `fELossAcc` includes pre-drift-volume energy, `InPos` is stale. Low practical impact today (window energy negligible, InPos unused), but a latent contract violation that would break if geometry changes or InPos is activated. **Confidence: High.**

2. **`timeNs` always zero** -- All MC points lack transport time. Silently wrong for any code checking time. **Confidence: High.**

3. **Mass inconsistency in straight-line path** -- KE computed from PDG mass but fed to model configured with amu-derived mass. ~0.5 MeV offset for protons. Curved path is correct. **Confidence: Medium.**

4. **MCTrack metadata sparse** -- `parentID = -1`, `time = 0`, `nPoints = 0` for all tracks. Code that checks nPoints or birth time gets wrong values. **Confidence: High.**

5. **No `correctPosOut()` equivalent** -- Exit positions overshoot the volume boundary by up to one step size (~1 mm). **Confidence: Medium.**

## 7. Suggested Validation Tests

**Contract test: entering fires at every volume boundary**

```
Setup: geometry with window + drift_volume. Inject a beam starting in the cave.
Assert: trackEnteringVolume is called at least twice (window entry, drift_volume entry).
Assert: fELossAcc is 0 at the first drift_volume step.
Assert: InPos is set to the beam's drift_volume entry position (not (0,0,0)).
```

**A/B output comparison (existing macros extend well):**

- Run `geant4_fixed.C` and `simpleSim_fixed.C` with identical kinematics.
- Compare per-event total energy loss (sum of MCPoint eLoss), number of MC points, and reaction vertex z-coordinate.
- Expected: total energy agrees within CATIMA model accuracy; point count differs; vertex z agrees to within window energy (~0.1 MeV).

**Invariance check: round-trip energy**

```
For each particle: sum(MCPoint.eLoss) + final KE ~= initial KE.
Tolerance: ~1% for step discretization.
This catches unit conversion errors and mass mismatches.
```

**Edge case: particle starts outside geometry**

```
Inject a particle at (0, 0, -500) mm (outside any volume).
Assert: TransportParticle throws std::invalid_argument.
```

**Edge case: missing energy-loss model**

```
Transport a particle species (Z, A) without registering a model or factory.
Assert: throws std::invalid_argument with descriptive message.
```

**Edge case: zero-momentum particle**

```
Inject a particle at the drift_volume entrance with p = (0,0,0).
Assert: FindSensitiveEntry throws (momentum is zero).
Or: if already in sensitive volume, SubmitInitialSensitivePoint handles stopping = true.
```

**Replay fidelity test:**

```
Run Geant4, then AtSimpleSimulationReplayTask with the output.
Assert: reaction product MC points appear on odd events only.
Assert: beam events (even) produce no MC points (transportPrimaries=false).
Assert: reaction vertex matches the Geant4 vertex from the source file.
```
