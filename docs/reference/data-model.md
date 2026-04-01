# Data Model

Core runtime objects passed between simulation, unpacking, reconstruction, and fitting.

Most FairRoot branches in this tree are not bare `AtEvent*` or `AtRawEvent*` objects. They are `TClonesArray` branch containers, usually holding one event object at slot `0`. Agents should treat the branch name and the contained event class as related but distinct concepts.

## Reconstruction Objects

| Object | Typical branch container | Produced by | Consumed by | Persisted | Key downstream fields |
|--------|--------------------------|-------------|-------------|-----------|-----------------------|
| `AtRawEvent` | `TClonesArray` branch such as `AtRawEvent` / `AtRawEventFiltered` | `AtUnpackTask`, `AtPulseTask` | filters, PSA | yes | pad traces, aux/FPN pads, good flag, optional MC map |
| `AtEvent` | `TClonesArray` branch such as `AtEventH` / `AtEventCleaned` | `AtPSAtask` | cleaning, pattern recognition | yes | hit list, event charge, mesh signal |
| `AtPatternEvent` | `TClonesArray` branch `AtPatternEvent` | `AtPRAtask`, `AtSampleConsensusTask` | fitting | yes | candidate tracks, noise hits |
| `AtTrack` | contained inside `AtPatternEvent`, not its own top-level branch | pattern recognition code | fitters | yes | track ID, hit/cluster-hit collections, geometric estimates, Bragg-curve values |
| `AtTrackingEvent` | `TClonesArray` branch `AtTrackingEvent` | `AtFitterTask` | downstream analysis | yes | fitted tracks, optional copied track array, event vertex fields |
| `AtFittedTrack` | contained inside `AtTrackingEvent`, not its own top-level branch | fitter implementations | downstream analysis | yes | current branch layout only; treat as unstable |

## Simulation Objects

| Object | Typical branch container | Produced by | Consumed by | Persisted | Notes |
|--------|--------------------------|-------------|-------------|-----------|-------|
| `AtTpcPoint` | `TClonesArray` branch `AtTpcPoint` | Geant/VMC transport through the AT-TPC detector | digitization tasks such as `AtClusterizeTask` and `AtPulseTask` | yes | detector-specific point branch seen by downstream digitization code |
| `AtMCPoint` | module-specific MC point branches | Geant/VMC transport | analysis, truth matching, detector-specific point types | yes | generic MC-point base family; downstream digitization usually works with detector-specific derived point classes |
| `AtMCTrack` | `TClonesArray` branch `MCTrack` | simulation stack | analysis, truth matching | yes | simulated particle tracks |
| `AtVertexPropagator` | no FairRoot branch | generators/runtime simulation logic | generators and downstream simulation code | no | singleton shared state |

## Flow Summary

```text
TClonesArray(AtRawEvent branch) -> AtRawEvent
TClonesArray(AtEventH branch)   -> AtEvent
TClonesArray(AtPatternEvent)    -> AtPatternEvent
TClonesArray(AtTrackingEvent)   -> AtTrackingEvent
                |            |
                |            -> contains AtTrack
                -> contains hits
```

```text
Generator / AtVertexPropagator
  -> MCTrack / AtTpcPoint branches
  -> AtSimulatedPoint branch
  -> AtRawEvent branch
```

For the tasks that produce and consume these objects, see [branch-io-contracts.md](branch-io-contracts.md). For pipeline order, see [simulation-pipeline.md](../subsystems/simulation-pipeline.md) and [reconstruction-pipeline.md](../subsystems/reconstruction-pipeline.md).
