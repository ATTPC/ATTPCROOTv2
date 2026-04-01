# AGENTS.md

ATTPCROOT is a ROOT/FairRoot-based C++ framework for simulation and analysis of Active Target Time Projection Chamber (AT-TPC) detector data.

## Documentation

Full developer documentation lives in `docs/development/`:

| Topic | File |
|-------|------|
| Environment setup | [docs/development/setup.md](../docs/development/setup.md) |
| Build & test commands | [docs/development/building.md](../docs/development/building.md) |
| Module overview | [docs/development/modules.md](../docs/development/modules.md) |
| Simulation pipeline | [docs/development/simulation-pipeline.md](../docs/development/simulation-pipeline.md) |
| Reconstruction pipeline | [docs/development/reconstruction-pipeline.md](../docs/development/reconstruction-pipeline.md) |
| Event generators | [docs/development/generators.md](../docs/development/generators.md) |
| Pulse shape analysis | [docs/development/psa.md](../docs/development/psa.md) |
| Energy loss | [docs/development/energy-loss.md](../docs/development/energy-loss.md) |
| Adding a new module | [docs/development/new-module.md](../docs/development/new-module.md) |
| Code style | [docs/development/code-style.md](../docs/development/code-style.md) |
| Testing | [docs/development/testing.md](../docs/development/testing.md) |

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
