# Branch and IO Contracts

This page records the current branch names and persistence defaults used by major tasks. These defaults come from the task constructors and `Init()` implementations in the current branch.

In most cases the branch object returned by `FairRootManager::GetObject(...)` is a `TClonesArray`, not a bare event object. The task then reads slot `0` as the current `AtRawEvent`, `AtEvent`, `AtPatternEvent`, or `AtTrackingEvent`.

## Reconstruction Tasks

| Task | Input Branches | Output Branches | Persistence Default | Notes |
|------|----------------|-----------------|---------------------|-------|
| `AtUnpackTask` | external file via unpacker | `AtRawEvent` | `true` | Registers a `TClonesArray` holding `AtRawEvent` objects |
| `AtFilterTask` | `AtRawEvent` | `AtRawEventFiltered` | `false` | Reads/writes `TClonesArray<AtRawEvent>`-style branches; can also filter aux/FPN pads |
| `AtPSAtask` | `AtRawEvent` | `AtEventH` | `false` | Reads `TClonesArray` input, writes `TClonesArray("AtEvent", 1)` output; optionally looks for `AtTpcPoint` |
| `AtDataCleaningTask` | `AtEventH` | `AtEventCleaned` | `true` | Reads the input event from a `TClonesArray` and writes a cleaned `AtEvent` copy |
| `AtPRAtask` | `AtEventH` | `AtPatternEvent` | `false` | PRA-based pattern recognition path using `TClonesArray` event containers |
| `AtSampleConsensusTask` | `AtEventH` | `AtPatternEvent` | `false` | Sample-consensus path using the same container pattern |
| `AtFitterTask` | `AtPatternEvent` | `AtTrackingEvent` | `false` | Output is a `TClonesArray("AtTrackingEvent", 1)`; verify input-branch behavior in source before relying on setters |
| `AtMCFitterTask` | `AtPatternEvent` | `AtMCResult`, `SimEvent`, `SimRawEvent` | `true`, `false`, `false` | Save toggles exist for each output branch |

## Simulation and Digitization Tasks

| Task | Input Branches | Output Branches | Persistence Default | Notes |
|------|----------------|-----------------|---------------------|-------|
| `AtClusterizeTask` | `AtTpcPoint` | `AtSimulatedPoint` | `false` | Reads the detector-specific `AtTpcPoint` `TClonesArray`, not a generic `AtMCPoint` branch |
| `AtPulseTask` | `AtSimulatedPoint`, optional `AtTpcPoint` | `AtRawEvent`, optional re-registered `AtTpcPoint` | `true`, `false` | Reads branch containers, not bare objects; `SetSaveMCInfo()` enables MC mapping behavior |

## Reading This Page

- These are default names, not hard-coded universal laws.
- Many tasks expose `SetInputBranch(...)`, `SetOutputBranch(...)`, or related setters.
- Do not assume every setter is honored consistently; check the task source before automating branch-name rewrites.
- If you add a task that reads/writes FairRoot branches, update this page.
- For the runtime objects behind these branches, see [data-model.md](data-model.md).
