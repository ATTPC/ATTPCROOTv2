# Reconstruction Pipeline

The reconstruction pipeline converts raw pad traces into hit clouds, candidate tracks, and fitted track results. Each stage is usually a `FairTask` that reads and writes branches through `FairRootManager`.

In the current tree, those branches are usually `TClonesArray` containers. Tasks typically fetch the branch container from `FairRootManager`, then read or construct the event object at slot `0`.

## Shared Early Stages

```text
AtUnpackTask
  │  reads raw GET/GRAW, HDF5, or ROOT data
  └─ output branch: AtRawEvent -> TClonesArray[AtRawEvent]
        ▼
AtFilterTask   [optional]
  │  filters raw traces
  └─ output branch: AtRawEventFiltered -> TClonesArray[AtRawEvent]
        ▼
AtPSAtask
  │  converts traces into hits
  └─ output branch: AtEventH -> TClonesArray[AtEvent]
        ▼
AtDataCleaningTask   [optional]
  │  removes or adjusts hits before pattern recognition
  └─ output branch: AtEventCleaned -> TClonesArray[AtEvent]
```

The exact branch names and defaults are documented in [branch-io-contracts.md](../reference/branch-io-contracts.md).

## Pattern Recognition

Two supported paths produce `AtPatternEvent → TClonesArray[AtPatternEvent]`. Both populate `fHitArray` on each track and `fNoise` on the event with unassigned hits. They differ in which `AtTrack` fields they fill:

| | `AtPRAtask` | `AtSampleConsensusTask` |
|-|-------------|-------------------------|
| Algorithm | triplet/hierarchical clustering (TriplClust default; `SetPRAlgorithm(n)`) | sample consensus (`SetPatternType`, `SetEstimator`) |
| `fPattern` | always `AtPatternCircle2D` | configurable: `kLine`, `kCircle2D`, `kRay`, `kY` |
| `fGeoRadius`, `fGeoCenter`, `fGeoTheta`, `fGeoPhi` | **filled** — via internal RANSAC after clustering | **not filled** (remain zero) |
| `fHitClusterArray` | populated | not populated |

The `fGeo*` fields are the momentum-seed inputs for GenFit-based fitters. `AtSampleConsensusTask` leaves them empty, so it requires additional processing (e.g. `AtPatternModificationTask`) before a GenFit-based fitter can run.

## Fitting

After pattern recognition, fitting produces the `AtTrackingEvent` branch container.

Current fit-related tasks include:

- `AtFitterTask`
- `AtMCFitterTask`

Fitting is unstable, so this page intentionally stays shallow. See [fitting-status.md](../development/fitting-status.md).

## Running Reconstruction

A typical macro creates a `FairRunAna`, adds tasks in order, and calls `Run(...)`.

For runtime objects, see [data-model.md](../reference/data-model.md). For example macros, see [macro-cookbook.md](../reference/macro-cookbook.md).
