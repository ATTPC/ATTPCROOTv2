# Module Overview

ATTPCROOT is organized as CMake library targets layered around data, simulation, unpacking, reconstruction, and analysis.

## Modules

`Also inspect` points to the most useful overview docs for that module.

| Module | Role | Main subareas | Common edit reasons | Also inspect |
|--------|------|---------------|---------------------|--------------|
| `AtData` | persisted event and track containers | event, track, pattern classes | add/modify data objects | [data-model.md](data-model.md), [branch-io-contracts.md](branch-io-contracts.md) |
| `AtReconstruction` | PSA, filtering, pattern recognition, fitting tasks | `AtPulseAnalyzer/`, `AtPatternRecognition/`, `AtPatternModification/`, `AtFitter/`, `AtFilter/` | change branch flow or task behavior | [reconstruction-pipeline.md](../subsystems/reconstruction-pipeline.md), [branch-io-contracts.md](branch-io-contracts.md) |
| `AtUnpack` | experimental data to `AtRawEvent` | unpackers, `GETDecoder2/`, `deprecated/` | add file-format/front-end support | [data-model.md](data-model.md) |
| `AtDigitization` | simulation truth to detector signals | clusterization, pulse generation, trigger/space-charge tasks | change MC-to-signal behavior | [simulation-pipeline.md](../subsystems/simulation-pipeline.md), [branch-io-contracts.md](branch-io-contracts.md) |
| `AtTools` | shared utilities | energy loss, kinematics, hit sampling, cleaning, electronic response | algorithmic utilities used across modules | [energy-loss.md](../subsystems/energy-loss.md) |
| `AtSimulationData` | MC truth and simulation shared state | MC objects, stack, propagator | change truth object layout or simulation state | [simulation-pipeline.md](../subsystems/simulation-pipeline.md), [data-model.md](data-model.md) |
| `AtGenerators` | FairRoot generator implementations | reaction and beam generators | change simulation source behavior | [generators.md](../subsystems/generators.md), [simulation-pipeline.md](../subsystems/simulation-pipeline.md) |
| `AtDetectors` | detector geometry and sensitive detectors | detector-specific subdirs, field/passive geometry | detector implementation changes | [geometry.md](../subsystems/geometry.md) |
| `AtAnalysis` | higher-level analysis code | experiment-facing analysis | analysis task changes | [data-model.md](data-model.md), [reconstruction-pipeline.md](../subsystems/reconstruction-pipeline.md) |
| `AtEventDisplay` | event visualization | `AtTabs/`, `AtSidebar/`, legacy display code | visualization changes | [visualization.md](../subsystems/visualization.md), [data-model.md](data-model.md) |

## Workflow Assets

| Directory | Role |
|-----------|------|
| `macro/` | example and test macros |
| `geometry/` | detector geometry assets |
| `parameters/` | runtime parameter files |
| `resources/` | shipped data assets such as tables and cross sections |
| `scripts/` | support scripts and mapping-related assets |
