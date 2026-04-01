# Daily Use

## Before Every Session

Source the generated environment script from the repo root:

```bash
source build/config.sh
```

This sets the following environment variables:

| Variable | Purpose |
|----------|---------|
| `ROOTSYS` | ROOT installation prefix |
| `VMCWORKDIR` | Repo root (used by macros to find geometry, parameters, etc.) |
| `ROOT_INCLUDE_PATH` | Headers exposed to the ROOT interpreter |
| `LD_LIBRARY_PATH` | Runtime library search path |
| Geant4 data paths | `G4...DATA` variables needed by Geant4 |

VSCode terminals auto-source this via the workspace settings.

If `build/config.sh` does not exist, the project has not been configured yet — see [installation.md](installation.md).

## Building

```bash
cmake --build build -j10
```

Adjust `-j` to your CPU count. To rebuild a single module:

```bash
cmake --build build --target AtSimulationData -j10
```

## Running Tests

Unit tests are built by default (`BUILD_TESTS=ON`). Binaries land in `build/tests/`.

Run all tests:

```bash
cd build && ctest -V
```

Run a specific test binary directly:

```bash
./build/tests/AtSimulationDataTests
./build/tests/AtGeneratorsTests
./build/tests/AtToolsTests
```

See [testing.md](testing.md) for how to write and register new tests.

## Verifying the Environment

```bash
root-config --version   # prints ROOT version
```
