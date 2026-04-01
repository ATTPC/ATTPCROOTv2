# Module Overview

ATTPCROOT is organized as CMake library targets layered around data, simulation, unpacking, reconstruction, and analysis.

## Modules

| Module | Role | Main subareas | Common edit reasons | Also inspect |
|--------|------|---------------|---------------------|--------------|
| `AtData` | persisted event and track containers | event, track, pattern classes | add/modify data objects | LinkDef, producing tasks, [data-model.md](data-model.md) |
| `AtReconstruction` | PSA, filtering, pattern recognition, fitting tasks | `AtPulseAnalyzer/`, `AtPatternRecognition/`, `AtFitter/`, `AtFilter/` | change branch flow or task behavior | [branch-io-contracts.md](branch-io-contracts.md), tests |
| `AtUnpack` | experimental data to `AtRawEvent` | unpackers, `GETDecoder2/` | add file-format/front-end support | maps, macros |
| `AtDigitization` | simulation truth to detector signals | clusterization, pulse generation | change MC-to-signal behavior | simulation pipeline |
| `AtTools` | shared utilities | energy loss, kinematics, hit sampling, cleaning | algorithmic utilities used across modules | dependent modules |
| `AtSimulationData` | MC truth and simulation shared state | MC objects, propagator | change truth object layout or simulation state | simulation pipeline |
| `AtGenerators` | FairRoot generator implementations | reaction and beam generators | change simulation source behavior | `AtVertexPropagator`, simulation pipeline |
| `AtDetectors` | detector geometry/sensitive detectors | detector-specific subdirs | detector implementation changes | geometry assets |
| `AtAnalysis` | higher-level analysis code | experiment-facing analysis | analysis task changes | data model |
| `AtEventDisplay` | event visualization | tabs/display code | visualization changes | data model |

## Workflow Assets

| Directory | Role |
|-----------|------|
| `macro/` | example and test macros |
| `geometry/` | detector geometry assets |
| `parameters/` | runtime parameter files |
| `resources/` | shipped data assets such as tables and cross sections |
| `scripts/` | support scripts and mapping-related assets |
