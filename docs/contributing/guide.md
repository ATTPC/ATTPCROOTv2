# Contributor Guide

## Persisted vs Non-Persisted Code

Use plain C++ types in non-persisted classes such as tasks, algorithms, models, samplers, and helpers:

```cpp
bool flag;
int count;
double value;
std::string name;
```

Use ROOT typedefs only in classes written to ROOT files:

```cpp
Bool_t fFlag;
Int_t fCount;
Double_t fValue;
```

## ROOT Dictionaries

Every class with `ClassDef` must appear in the appropriate `*LinkDef.h`.
Classes listed in `*LinkDef.h` do not automatically need `ClassDef` or `ClassImp`.

Preferred suffix rules for new or edited code:

- `ClassName +;`
  for classes written to disk
- `ClassName -!;`
  for reflection-only classes that should not get streamers

Default to `-!` unless the class is actually persisted.

If you add a nested type inside a persisted class and ROOT needs to stream it, register that nested type explicitly as well.
If a persisted class owns streamed pointees through pointers or smart pointers, the pointee types still need the required dictionary coverage.

Current LinkDefs in the tree still contain legacy `+` entries for some non-persisted classes. Treat the rule above as the preferred direction for new work, not as permission for mechanical repo-wide rewrites.

## Persisted Data Model Conventions

Do not assume older ROOT folklore applies everywhere in this tree. Persisted classes here already use STL containers and `std::unique_ptr` where ROOT dictionary support is in place.

Examples in the current branch:

- `AtEvent` stores hits in `std::vector<std::unique_ptr<AtHit>>`
- `AtTrack` stores hits and pattern state with smart pointers
- `AtTrackingEvent` stores fitted tracks in `std::vector<std::unique_ptr<AtFittedTrack>>`

For new code, follow the local data-model pattern of the owning module. Do not "simplify" these classes back to raw pointers or `TClonesArray` just because older ROOT examples elsewhere do that.

## Include and Header Norms

- Public headers are copied into `build/include`, so code in this tree normally includes project headers by basename such as `"AtEvent.h"`, not module-qualified paths.
- Prefer the include style already used in the surrounding module. Avoid mechanical rewrites to `AtData/AtEvent.h`-style paths.
- Public headers commonly use forward declarations for ROOT and FairRoot types. Follow the local header pattern instead of eagerly adding heavyweight includes.

## When You Add Code

For a new persisted class, usually update all of:

- header/source in the owning module
- module `CMakeLists.txt`
- module `*LinkDef.h`
- tests
- docs if branch flow or the runtime data model changes

For a new non-persisted task or algorithm, usually update all of:

- header/source in the owning module
- module `CMakeLists.txt`
- module `*LinkDef.h` with `-!`
- tests
- [branch-io-contracts.md](../reference/branch-io-contracts.md) if it reads or writes FairRoot branches

## CMake and Test Registration

- `generate_target_and_root_library(...)`
  wires libraries and dictionaries together
- `attpcroot_generate_tests(...)`
  wires test binaries into CTest

If you update code but miss one of those registrations, the branch will often build partially and fail later.

For test writing rules and registration patterns see [testing.md](../tooling/testing.md).

## ROOT / FairRoot Gotchas

- Objects read from `FairRootManager` or pulled out of framework-managed `TClonesArray` containers are framework-owned.
- Do not manually `delete` branch objects returned by the framework.
- Do not wrap FairRoot-owned branch objects in smart pointers.
- Macros still use raw `new` for tasks and generators because ownership is transferred through framework registration such as `AddTask(...)` and generator setup.
- Inside tasks and helper classes, prefer the ownership pattern already used locally. In current code that often means `std::unique_ptr` for owned algorithms or models.
- If a ROOT macro cannot resolve a symbol, first suspect the library, dictionary, or environment setup before adding project header includes.
