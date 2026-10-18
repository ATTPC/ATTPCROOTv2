# Development Documentation

Agent-facing docs for the current branch state of ATTPCROOT.

## Tasks

- install or first-time configure: [tooling/installation.md](tooling/installation.md)
- set up a session / build / run tests: [tooling/daily-use.md](tooling/daily-use.md)
- write and register tests: [tooling/testing.md](tooling/testing.md)
- add code safely: [contributing/guide.md](contributing/guide.md), [contributing/new-module.md](contributing/new-module.md)
- check formatting rules: [contributing/code-style.md](contributing/code-style.md)
- trace branch flow: [reference/branch-io-contracts.md](reference/branch-io-contracts.md), [subsystems/reconstruction-pipeline.md](subsystems/reconstruction-pipeline.md), [subsystems/simulation-pipeline.md](subsystems/simulation-pipeline.md)
- understand runtime objects: [reference/data-model.md](reference/data-model.md)
- find subsystem ownership: [reference/modules.md](reference/modules.md)
- find an example macro: [reference/macro-cookbook.md](reference/macro-cookbook.md)
- configure detector parameters: [reference/parameters.md](reference/parameters.md)
- prepare detector geometry: [subsystems/geometry.md](subsystems/geometry.md)
- browse data interactively: [subsystems/visualization.md](subsystems/visualization.md)

## Architecture

- module map: [reference/modules.md](reference/modules.md)
- data model: [reference/data-model.md](reference/data-model.md)
- simulation flow: [subsystems/simulation-pipeline.md](subsystems/simulation-pipeline.md)
- reconstruction flow: [subsystems/reconstruction-pipeline.md](subsystems/reconstruction-pipeline.md)
- generators: [subsystems/generators.md](subsystems/generators.md)
- PSA: [subsystems/psa.md](subsystems/psa.md)
- energy loss: [subsystems/energy-loss.md](subsystems/energy-loss.md)
- geometry: [subsystems/geometry.md](subsystems/geometry.md)
- visualization: [subsystems/visualization.md](subsystems/visualization.md)
- parameters: [reference/parameters.md](reference/parameters.md)

## Branch-Specific Notes

- fitting status: [development/fitting-status.md](development/fitting-status.md)
- MC truth design (under discussion): [development/mc-truth.md](development/mc-truth.md)
- MC fitter threading (how it parallelizes, known bugs, where the time goes): [development/mc-fitter-threading.md](development/mc-fitter-threading.md)
- simulation, digitization and MC fit speed (2026-10 study): [development/fit-speed-study.md](development/fit-speed-study.md)
- units and conventions: [development/units-and-conventions.md](development/units-and-conventions.md)

## Scope Notes

- These docs describe the current branch state.
- They are framework-centric and keep experiment-specific detail light.
