# Macro Cookbook

Example macro starting points referenced by these docs. 

## Notes

- `macro/examples/` contains adaptation starting points.
- `macro/tests/` contains repeatable workflow and regression-style examples.
- Many macros are experiment-specific or user-specific and are intentionally omitted.
- Files under `macro/` are ROOT / Cling scripts, not normal compiled translation units.
- ROOT/core headers such as `TClonesArray.h` or `TTreeReader.h`, and STL headers a macro genuinely uses, are normal.
- Do not treat a macro like standalone C++ or try to fix missing symbols by adding project headers first.

## Macro Do / Don't

- Do: copy an existing pattern from `macro/examples/` or `macro/tests/`.
- Don't: start by adding `#include "At*.h"` to compensate for missing dictionaries or libraries, or rewrite a macro as if it were a compiled source file.

## Starting Points

| Path | Use | Caveat |
|------|-----|--------|
| `macro/examples/run_sim.C` | minimal simulation structure | simple example, not a full workflow template |
| `macro/examples/rundigi_sim.C` | simulation plus downstream task ordering | parts of the task usage are older and should be checked against current APIs |
| `macro/tests/AT-TPC/run_unpack_attpc.C` | unpacking plus PSA/cleaning flow | experiment-facing example, not a framework contract |
| `macro/tests/AT-TPC/run_sim_attpc.C` | fuller simulation test flow | oriented toward test workflow rather than minimal onboarding |
