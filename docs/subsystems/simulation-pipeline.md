# Simulation Pipeline

The simulation pipeline converts a generated reaction into MC truth, simulated detector responses, and finally `AtRawEvent` output that can be passed into reconstruction.

There are two transport engines for generating `AtMCPoint` data:

- **Geant4/VMC** -- full Monte Carlo transport through detector geometry
- **SimpleSim** -- model-driven propagation with user-configured energy loss models, running as a FairTask inside the same FairRunSim event loop

Both produce the same `AtMCPoint` and `MCTrack` output format, so the downstream digitization chain (`AtClusterizeTask` -> `AtPulseTask`) works unchanged with either.

## Flow

```
FairPrimaryGenerator + AtReactionGenerator
                 │
          ┌──────┴──────────────┐
          ▼                     ▼
  Geant4/VMC transport    SimpleSim FairTask
  (AtTpc::ProcessHits)    (AtSimTransportTask)
          │                     │
          └──────────┬──────────┘
                     ▼
                 AtMCPoint + MCTrack
                     ▼
              AtClusterizeTask
                     ▼
               AtPulseTask
                     ▼
                AtRawEvent
```

### Geant4/VMC Path

The standard path. `FairPrimaryGenerator` pushes particles onto `AtStack`; Geant4 transports them through the detector geometry; `AtTpc::ProcessHits()` records `AtMCPoint` entries.

### SimpleSim Path

SimpleSim runs as a `FairTask` inside the same `FairRunSim` event loop. It uses the `AtSimTransport` engine (owned by `AtSimTransportTask`) to propagate particles through the geometry with energy-loss models served by an `AtELossManager`, supporting both straight-line (no field) and curved-track (magnetic field via RK4) propagation. Steps are fed through `AtTpc::ProcessStep()` -- the same detector logic used by Geant4 -- so reaction triggers, vertex propagation, and hit recording work identically.

The standalone hit-recording class `AtSimpleSimulation` wraps `AtSimTransport` with a thread-local `TClonesArray` of `AtMCPoint` and is used by `AtMCFitter`, `AtMCFission`, and analysis macros that manage their own event loop.

Two task classes are provided for the FairRoot path:

- **`AtSimTransportGeneratorTask`** -- generates events live via a `FairPrimaryGenerator`, using the same generator chain as the Geant4 path. This is the primary task for production use.
- **`AtSimTransportReplayTask`** -- reads primary MCTracks from a prior Geant4 run and re-transports them through SimpleSim. Useful for A/B validation with identical kinematics.

Both write `AtMCPoint` and `MCTrack` branches in the same format as Geant4, so downstream tasks work unchanged.

See [simplesim-migration.md](simplesim-migration.md) for a step-by-step guide to converting a Geant4 macro.

### Shared Downstream Stages

Once `AtMCPoint` objects exist, the remaining stages are shared:

- `AtClusterizeTask` converts energy deposits into ionization-electron clusters represented as `AtSimulatedPoint`
- `AtPulseTask` drifts those electrons to the pad plane and produces simulated traces in `AtRawEvent`

## Key Runtime Objects

- `AtVertexPropagator`
  shared simulation-side state between generators and downstream logic
- `AtMCTrack`
  simulated particle tracks
- `AtMCPoint`
  MC-point type used by digitization logic
- `AtRawEvent`
  final simulated raw traces used by reconstruction

See [data-model.md](../reference/data-model.md) for the object-level view and [branch-io-contracts.md](../reference/branch-io-contracts.md) for task branch names.

## Event Structure

Both transport paths represent each physical beam-induced event as two consecutive FairRoot events:

- Even-indexed event (0, 2, 4, ...): beam phase -- the beam particle traverses the detector
- Odd-indexed event (1, 3, 5, ...): reaction phase -- the reaction products are transported

Events 0+1 form one complete beam-induced event; events 2+3 form the next, and so on. Code that loops over events or checks event indices must account for this pairing.

**Track IDs:** Within each FairRoot event the beam particle always has `fTrackID = 0`. Reaction products receive subsequent IDs.

## Required Pieces

For the **Geant4/VMC** path, a simulation run needs:

- detector geometry
- a configured `FairPrimaryGenerator` with one or more `AtReactionGenerator` subclasses
- the digitization stages `AtClusterizeTask` and `AtPulseTask`
- the experiment parameter set used by digitization

For the **SimpleSim** path, the run additionally needs:

- an `AtSimTransport` instance (uses the FairRunSim geometry automatically)
- an `AtELossManager` (or a subclass such as `AtELossManagerCATIMA` / `AtELossManagerBetheBloch`) carrying the energy-loss models for each particle species
- the detector set via `SetDetector(tpc)` on the SimpleSim task

Energy loss models must be available for every (Z, A) pair that will be transported in every material they traverse. Models can be registered manually via `manager->AddModel(...)`, or an auto-generating `AtELossManager` subclass can create them from the geometry on demand. If no matching model is registered or generated, the transport stops for that track.

See [generators.md](generators.md) for generator behavior, [energy-loss.md](energy-loss.md) for the model layer, and [simplesim-migration.md](simplesim-migration.md) for the migration guide.
