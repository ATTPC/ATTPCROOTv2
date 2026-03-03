# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About

ATTPCROOT is a ROOT/FairRoot-based C++ framework for simulation and analysis of Active Target Time Projection Chamber (AT-TPC) detector data. It integrates with FairSoft (provides ROOT, Geant4, VMC) and FairRoot (provides the task/run framework).

## Environment Setup

Before building or running anything, the environment must be loaded. The VSCode terminal auto-sources this on startup:

```bash
source build/config.sh
```

This sets `LD_LIBRARY_PATH`, `ROOTSYS`, `VMCWORKDIR`, `ROOT_INCLUDE_PATH`, and Geant4 data paths. The key install paths on this machine are:
- FairRoot: `~/fair_install/FairRootInstall`
- FairSoft: `~/fair_install/FairSoftInstall`

For CMake configuration (not the build itself), the following env vars are needed:
```bash
export FAIRROOTPATH=~/fair_install/FairRootInstall
export SIMPATH=~/fair_install/FairSoftInstall
```

## Build Commands

```bash
# Configure (from repo root, out-of-source into build/)
cmake -S . -B build -DCMAKE_PREFIX_PATH=~/fair_install/hdf5

# Build (10 parallel jobs)
cmake --build build -j10

# Build a specific target
cmake --build build --target AtSimulationData -j10

# Run all unit tests
cd build && ctest -V

# Run a specific test binary directly
./build/tests/AtSimulationDataTests
./build/tests/AtGeneratorsTests
./build/tests/AtToolsTests
```

Tests are built by default (`BUILD_TESTS=ON`). Test binaries are placed in `build/tests/`.

## Code Formatting

The project uses `clang-format-17` with the config in `.clang-format` (based on LLVM style, 3-space indent, 120-char column limit). Format on save is configured in VSCode. To format manually:

```bash
clang-format-17 -i <file>
# Or format all changed files:
scripts/formatAll.sh
```

Static analysis uses `clang-tidy` (config in `.clang-tidy`): modernize-* and cppcoreguidelines-* checks.

## Architecture

The framework follows FairRoot's task pipeline pattern: data flows through a chain of `FairTask` subclasses, persisted as ROOT TClonesArrays in a TTree.

### Module Overview

| Module | Purpose |
|--------|---------|
| `AtSimulationData` | MC truth data: `AtStack`, `AtMCTrack`, `AtMCPoint`, `AtVertexPropagator` |
| `AtGenerators` | FairGenerator subclasses for beam/reaction/decay event generation |
| `AtDetectors` | Geant4 sensitive detector implementations (AT-TPC, GADGET-II, SpecMAT, etc.) |
| `AtDigitization` | Converts MC points → simulated pad signals (`AtClusterize`, `AtPulse`) |
| `AtUnpack` | Unpacks raw experimental data (GET/GRAW, HDF5, ROOT formats) |
| `AtData` | Core data classes: `AtRawEvent`, `AtEvent`, `AtHit`, `AtPad`, `AtTrack` |
| `AtMap` | Pad mapping between electronics channels and detector geometry |
| `AtParameter` | FairRuntimeDb parameter containers |
| `AtReconstruction` | PSA, pattern recognition, fitting tasks |
| `AtTools` | Utilities: energy loss (CATIMA), kinematics, space charge, hit sampling |
| `AtAnalysis` | High-level analysis tasks and `AtRunAna` |
| `AtS800` | S800 spectrograph data handling |
| `AtEventDisplay` | ROOT Eve-based event display |

### Data Flow

**Simulation pipeline** (macros in `macro/Simulation/`):
1. `FairPrimaryGenerator` with an `AtReactionGenerator` subclass → particle stack
2. Geant4/VMC transport → `AtMCPoint` hits in sensitive volumes
3. `AtClusterizeTask` → electron clusters from ionization
4. `AtPulseTask` → simulated pad signals (`AtRawEvent`)

**Reconstruction pipeline** (macros in `macro/Unpack_*/` and `macro/Analysis/`):
1. `AtUnpackTask` → `AtRawEvent` (raw pad traces)
2. `AtFilterTask` → filtered `AtRawEvent`
3. `AtPSAtask` (Pulse Shape Analysis) → `AtEvent` with `AtHit` objects
4. `AtSampleConsensusTask` / `AtPRAtask` → `AtPatternEvent` with `AtTrack` objects
5. `AtMCFitterTask` / `AtFitterTask` → fitted tracks with kinematics

### Key Design Patterns

**FairTask subclasses**: Each processing step is a `FairTask`. They retrieve input branches via `TClonesArray*` from `FairRootManager` in `Init()` and process them in `Exec()`.

**AtReactionGenerator**: Abstract base for all reaction generators. Subclasses implement `GenerateReaction()`. The `ReadEvent()` method is `final` and handles the beam/reaction event alternation via `AtVertexPropagator`. Generators can be chained for sequential decays using `SetSequentialDecay(true)`.

**AtVertexPropagator**: Singleton (`AtVertexPropagator::Instance()`) that communicates vertex, momentum, and kinematics between chained generators and downstream tasks. Alternates beam/reaction events via `EndEvent()`. Use `ResetForTesting()` in unit tests.

**PSA (Pulse Shape Analysis)**: `AtPSA` is the abstract base. Concrete implementations (`AtPSAMax`, `AtPSAFull`, `AtPSADeconv`, etc.) extract hits from pad traces. `AtPSAComposite` allows chaining PSA methods.

**Energy loss**: `AtELossModel` is the base; `AtELossCATIMA` wraps the CATIMA library (fetched automatically if not installed). Used via `AtELossManager`.

### Adding a New Module Library

Each module's `CMakeLists.txt` follows this pattern:
```cmake
set(LIBRARY_NAME MyModule)
set(SRCS file1.cxx file2.cxx)
set(DEPENDENCIES ATTPCROOT::AtData ROOT::Core ...)
set(TEST_SRCS MyModuleTest.cxx)
attpcroot_generate_tests(${LIBRARY_NAME}Tests SRCS ${TEST_SRCS} DEPS ${LIBRARY_NAME})
generate_target_and_root_library(${LIBRARY_NAME} LINKDEF ${LIBRARY_NAME}LinkDef.h SRCS ${SRCS} DEPS_PUBLIC ${DEPENDENCIES})
```

ROOT dictionary generation requires a `*LinkDef.h` file listing every class that needs ROOT reflection. Every class with `ClassDef` in its header must appear in the LinkDef.

**LinkDef streamer suffix** — the suffix after the class name controls whether ROOT generates a full I/O streamer:

- `ClassName +;` — generates a full streamer. Use this **only** for classes that are written to a ROOT file (i.e. stored as a branch in a `TClonesArray` or `TTree`). Examples: `AtHit`, `AtEvent`, `AtRawEvent`, `AtMCPoint`.
- `ClassName -!;` — registers the class for reflection (usable in interpreted macros, usable as a pointer type) but **suppresses the streamer**. Use this for every class that is never written to disk: tasks, algorithms, models, samplers, etc. Examples: `AtELossModel`, `AtSpaceChargeModel`, `AtSample` subclasses.

Generating an unnecessary streamer bloats the dictionary and binary with unused I/O code, so the default should be `-!` unless disk persistence is actually required.

**C++ vs ROOT typedefs** — use plain C++ types (`bool`, `int`, `double`, `std::string`) in all classes that are not written to a ROOT file. Only use ROOT's portability typedefs (`Bool_t`, `Int_t`, `Double_t`, etc.) in classes that are persisted to disk, where ROOT's cross-platform I/O guarantees matter. Mixing ROOT types into purely algorithmic or task classes is unnecessary overhead.

### Unit Tests

Tests use Google Test, placed alongside source files (e.g., `AtVertexPropagatorTest.cxx` in `AtSimulationData/`). Register them in the module's `CMakeLists.txt` via `attpcroot_generate_tests()`. Tests must not access external files or network resources.

### Macros

ROOT macros (`.C` files) in `macro/` are the primary user interface for running simulations and analyses. They are not compiled into libraries—they run interpreted by ROOT. Integration tests live in `macro/tests/`.

## Contributing

- PRs must target the `develop` branch and apply cleanly (fast-forward, no merge commits).
- Commit messages: present imperative mood, ≤72 characters summary.
- All PRs are checked by `clang-format`, `clang-tidy`, and the unit test suite.
