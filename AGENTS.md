# AGENTS.md

ATTPCROOT is a ROOT/FairRoot-based C++ framework for simulation and analysis of Active Target Time Projection Chamber (AT-TPC) detector data.

## Documentation

Full developer documentation lives in `docs/`. See [docs/index.md](docs/index.md) for the full map. Quick topic links:

| Topic | File |
|-------|------|
| First-time install | [tooling/installation.md](docs/tooling/installation.md) |
| Daily use (build/test) | [tooling/daily-use.md](docs/tooling/daily-use.md) |
| Testing patterns | [tooling/testing.md](docs/tooling/testing.md) |
| Contributor guide | [contributing/guide.md](docs/contributing/guide.md) |
| Adding a new module | [contributing/new-module.md](docs/contributing/new-module.md) |
| Code style | [contributing/code-style.md](docs/contributing/code-style.md) |
| Module overview | [reference/modules.md](docs/reference/modules.md) |
| Data model | [reference/data-model.md](docs/reference/data-model.md) |
| Branch I/O contracts | [reference/branch-io-contracts.md](docs/reference/branch-io-contracts.md) |
| Simulation pipeline | [subsystems/simulation-pipeline.md](docs/subsystems/simulation-pipeline.md) |
| Reconstruction pipeline | [subsystems/reconstruction-pipeline.md](docs/subsystems/reconstruction-pipeline.md) |
| Event generators | [subsystems/generators.md](docs/subsystems/generators.md) |
| Pulse shape analysis | [subsystems/psa.md](docs/subsystems/psa.md) |
| Energy loss | [subsystems/energy-loss.md](docs/subsystems/energy-loss.md) |

## Quick Reference: Build & Test

```bash
source build/config.sh          # load environment (do this first)
cmake --build build -j10        # build everything
cd build && ctest -V            # run all unit tests
```

## Code-Writing Rules

These rules apply whenever editing or adding C++ code in this repo.

### LinkDef Streamer Suffixes

Every class in a `*LinkDef.h` file must use the correct suffix:

- `ClassName +;` — generates a full I/O streamer. **Only** for classes written to a ROOT file (stored in a `TClonesArray` or `TTree` branch). Examples: `AtHit`, `AtEvent`, `AtRawEvent`, `AtMCPoint`.
- `ClassName -!;` — reflection only, no streamer. Use for **everything else**: tasks, algorithms, models, samplers. Examples: `AtPSAMax`, `AtELossModel`, `AtFitterTask`.

Default to `-!` unless disk persistence is actually required.

### C++ vs ROOT Typedefs

- Non-persisted classes (tasks, algorithms, models): use `bool`, `int`, `double`, `std::string`.
- Persisted data classes only: use `Bool_t`, `Int_t`, `Double_t`, etc.

### Test Isolation

Unit tests must not access external files or network resources. Hardcode test data inline.

## Contributing

- PRs target the `develop` branch; fast-forward only (no merge commits).
- Commit messages: present imperative mood, ≤72 characters.
- All PRs must pass `clang-format`, `clang-tidy`, and unit tests.
