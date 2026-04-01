# Simulation Pipeline

The simulation pipeline converts a generated reaction into MC truth, simulated detector responses, and finally `AtRawEvent` output that can be passed into reconstruction.

Like reconstruction, the task-facing branch objects here are usually `TClonesArray` containers. The detector-point branch used by digitization is `AtTpcPoint`, not a generic `AtMCPoint` branch.

## Steps

```
FairPrimaryGenerator
  └─ AtReactionGenerator subclass
        │  generates primary particles and writes to particle stack
        ▼
Geant4 / VMC transport
        │  produces detector-point branches such as AtTpcPoint
        ▼
AtClusterizeTask
        │  reads TClonesArray(AtTpcPoint) and converts deposits -> ionization electron clusters
        ▼
AtPulseTask
        │  drifts electrons and produces simulated pad traces
        └─ output branch: AtRawEvent -> TClonesArray[AtRawEvent]
```

## Key Runtime Objects

- `AtVertexPropagator`
  shared simulation-side state between generators and downstream logic
- `AtMCTrack`
  simulated particle tracks
- `AtTpcPoint`
  detector-specific transport hits consumed by current digitization tasks
- `AtMCPoint`
  generic base-family concept for detector points, but usually not the concrete branch a digitization task reads
- `AtRawEvent`
  final simulated raw traces used by reconstruction

See [data-model.md](../reference/data-model.md) for the object-level view and [branch-io-contracts.md](../reference/branch-io-contracts.md) for task branch names.

## Running a Simulation

Simulation macros live mostly in `macro/Simulation/` and `macro/examples/`. A typical macro:

1. Creates a `FairRunSim`
2. Attaches the detector geometry
3. Configures a `FairPrimaryGenerator` with one or more `AtReactionGenerator` subclasses
4. Adds `AtClusterizeTask` and `AtPulseTask`
5. Calls `fRun->Run(nEvents)`

The output ROOT file contains MC truth alongside detector-specific point branches and the simulated `AtRawEvent`, so it can be fed directly into the reconstruction pipeline.

Geometry comes from `geometry/`; runtime parameters typically come from `parameters/`.

See [generators.md](generators.md) for generator behavior and [macro-cookbook.md](../reference/macro-cookbook.md) for example macros.
