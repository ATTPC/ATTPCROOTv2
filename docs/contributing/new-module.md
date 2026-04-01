# Adding a New Module Library

Each ATTPCROOT module is a separate CMake shared library with an associated ROOT dictionary.

## CMakeLists.txt Pattern

```cmake
set(LIBRARY_NAME MyModule)
set(SRCS
   MyClass.cxx
   AnotherClass.cxx
)
set(DEPENDENCIES
   ATTPCROOT::AtData
   ROOT::Core
   FairRoot::Base
)

# Optional: unit tests
set(TEST_SRCS MyClassTest.cxx)
attpcroot_generate_tests(${LIBRARY_NAME}Tests SRCS ${TEST_SRCS} DEPS ${LIBRARY_NAME})

generate_target_and_root_library(
   ${LIBRARY_NAME}
   LINKDEF ${LIBRARY_NAME}LinkDef.h
   SRCS    ${SRCS}
   DEPS_PUBLIC ${DEPENDENCIES}
)
```

## LinkDef File

Every class with a `ClassDef` macro in its header must appear in `MyModuleLinkDef.h`. ROOT uses this file to generate the dictionary.

```cpp
#ifdef __CINT__
#pragma link off all globals;
#pragma link off all classes;
#pragma link off all functions;

#pragma link C++ class MyDataClass+;     // preferred for persisted classes
#pragma link C++ class MyTask-!;         // preferred for reflection-only code
#pragma link C++ class MyAlgorithm-!;

#endif
```

For the preferred streamer suffix rules, persisted-vs-non-persisted type rules, and notes on legacy LinkDef entries already in the tree, use [guide.md](guide.md).

## Common FairTask Pattern

Most framework tasks follow:

- `Init()`
  load/register branches and initialize owned algorithms
- `SetParContainers()`
  load runtime-db containers when needed
- `Exec()`
  clear outputs, read the input `TClonesArray` and current event object, and write the next branch container

If a task reads or writes documented branches, update [branch-io-contracts.md](../reference/branch-io-contracts.md).

## Registering with the Build

Add the new subdirectory to the top-level `CMakeLists.txt`:

```cmake
add_subdirectory(MyModule)
```

And add `ATTPCROOT::MyModule` to the `DEPENDENCIES` of any module that needs it.
