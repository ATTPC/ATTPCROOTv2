# Code Style

This page covers naming conventions, formatting, and static-analysis tools. Contributor-critical ROOT and persistence rules live in [guide.md](guide.md).

## Naming Conventions

### Libraries and Classes

- **Libraries**: a folder named `AtFoo` produces shared object `libAtFoo`
- **Classes**: non-namespaced classes use the `At` prefix in CamelCase — `AtBaseEvent`, `AtPSAMax`, `AtFitterTask`
- Namespaced code (e.g., `EventFit::`, `kf::`) omits the `At` prefix

### Data Members

- Private and protected members: `f` prefix + capital letter — `fTrackID`, `fDriftVelocity`
- Boolean flags: `k` prefix — `kIsGood`, `kInitialized`
- Local variables and function parameters: no prefix, lowercase camelCase

### Methods

Follow ROOT naming conventions:
- Getters: `GetXaxis()` not `GetXAxis()` (avoid back-to-back capitals)
- Setters: `SetThreshold()`
- Boolean queries: `IsValid()`, `HasHits()`

### ClassDef / ClassImp

Use `ClassDef` and `ClassImp` **only** for `TObject` subclasses. When the memory layout changes, increment the version number in `ClassDef`:

```cpp
ClassDef(AtMyData, 2);  // bump when adding/removing/reordering members
```

Non-persisted classes (tasks, algorithms, models) do not need `ClassDef`.

## Formatting

The project uses `clang-format-17` with the repo-root `.clang-format`.

Format one file:

```bash
clang-format-17 -i path/to/file.cxx
```

Format changed files:

```bash
scripts/formatAll.sh
```

## Static Analysis

The repo also carries `.clang-tidy` configuration and CI checks for:

- `clang-format`
- `clang-tidy`
- `iwyu`

Run clang-tidy manually on a file:

```bash
clang-tidy path/to/file.cxx -- -I...
```

For framework-specific contributor rules, use [guide.md](guide.md).
