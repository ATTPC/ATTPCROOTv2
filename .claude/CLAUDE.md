# CLAUDE.md

ATTPCROOT is a ROOT/FairRoot-based C++ framework for simulation and analysis of Active Target Time Projection Chamber (AT-TPC) detector data.

## Documentation

Full developer documentation lives in `docs/`. Before reading source files or writing code, `ls docs/` and read `docs/index.md` to orient yourself — it is the canonical map of all available docs.

Macros in the `macros/` folder are useful for understanding how the code is used in practice, but are often stale and should not be treated as authoritative specification.

## Quick Reference: Build & Test

```bash
source build/config.sh          # load environment (do this first)
cmake --build build -j10        # build everything
cd build && ctest -V            # run all unit tests
cd build && ctest -R TestName -V  # run a single test by name
./build/tests/AtToolsTests      # run a test binary directly
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

Register tests in CMakeLists.txt with:
```cmake
attpcroot_generate_tests(${LIBRARY_NAME}Tests SRCS test_foo.cxx DEPS SomeLib)
```

## Contributing

- Feature branches off `develop`; PRs target `develop`; fast-forward only (no merge commits).
- Commit messages: present imperative mood, ≤72 characters.
- All PRs must pass `clang-format-17` (3-space indent, 120-char line limit), `clang-tidy`, and unit tests.
