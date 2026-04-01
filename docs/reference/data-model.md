# Data Model

Core runtime objects passed between simulation, unpacking, reconstruction, and fitting.

Most FairRoot branches in this tree are not bare `AtEvent*` or `AtRawEvent*` objects. They are `TClonesArray` branch containers, usually holding one event object at slot `0`. Agents should treat the branch name and the contained event class as related but distinct concepts.

## Reconstruction Objects

| Object | Typical branch container | Main producer | Main consumer | Persisted | Key downstream fields |
|--------|--------------------------|---------------|---------------|-----------|-----------------------|
| `AtRawEvent` | `TClonesArray` branch such as `AtRawEvent` / `AtRawEventFiltered` | `AtUnpackTask`, `AtPulseTask` | `AtFilterTask`, `AtPSAtask` | yes | pad traces, aux/FPN pads, good flag, optional MC map |
| `AtEvent` | `TClonesArray` branch such as `AtEventH` / `AtEventCleaned` | `AtPSAtask` | `AtDataCleaningTask`, `AtPRAtask`, `AtSampleConsensusTask` | yes | hit list, event charge, mesh signal |
| `AtPatternEvent` | `TClonesArray` branch such as `AtPatternEvent` / `AtPatternEventModified` | `AtPRAtask`, `AtSampleConsensusTask` | `AtFitterTask`, `AtMCFitterTask` | yes | candidate tracks, noise hits |
| `AtTrack` | contained inside `AtPatternEvent`, not its own top-level branch | pattern-recognition code | fitters, track-level analysis | yes | track ID, hit/cluster-hit collections, geometric estimates, Bragg-curve values |
| `AtTrackingEvent` | `TClonesArray` branch `AtTrackingEvent` | `AtFitterTask` | downstream analysis | yes | fitted tracks, optional copied track array, event vertex fields |
| `AtFittedTrack` | contained inside `AtTrackingEvent`, not its own top-level branch | fitter implementations | downstream analysis | yes | current branch layout only; treat as unstable |
| `AtMCResult` | `TClonesArray` branch `AtMCResult` | `AtMCFitterTask` | MC-fitting analysis | yes | objective value and fitted/sampled parameter summary |

## Simulation Objects

| Object | Typical branch container | Main producer | Main consumer | Persisted | Notes |
|--------|--------------------------|---------------|---------------|-----------|-------|
| `AtMCPoint` | MC-point branches such as `AtTpcPoint` | Geant/VMC transport, `AtSimpleSimulation` | `AtClusterizeTask` | yes | primary simulation-to-digitization handoff object; also used for MC bookkeeping in simulated-data reconstruction |
| `AtSimulatedPoint` | `TClonesArray` branch `AtSimulatedPoint` | `AtClusterizeTask` | `AtPulseTask` | yes | intermediate ionization-electron / charge-cluster representation between MC points and pad traces |
| `AtMCTrack` | `TClonesArray` branch `MCTrack` | simulation stack | analysis, truth matching | yes | simulated particle tracks |
| `AtVertexPropagator` | no FairRoot branch | generators/runtime simulation logic | generators and downstream simulation code | no | singleton shared state |

## Container Pattern

```text
FairRoot branch name -> TClonesArray container -> runtime object

AtRawEvent      -> TClonesArray -> AtRawEvent
AtEventH        -> TClonesArray -> AtEvent
AtPatternEvent  -> TClonesArray -> AtPatternEvent
AtTrackingEvent -> TClonesArray -> AtTrackingEvent
MCTrack         -> TClonesArray -> AtMCTrack
AtTpcPoint      -> TClonesArray -> AtMCPoint
AtSimulatedPoint-> TClonesArray -> AtSimulatedPoint
```

For the tasks that produce and consume these objects, see [branch-io-contracts.md](branch-io-contracts.md). For pipeline order, see [simulation-pipeline.md](../subsystems/simulation-pipeline.md) and [reconstruction-pipeline.md](../subsystems/reconstruction-pipeline.md).
