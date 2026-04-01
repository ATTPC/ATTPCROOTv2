# Detector Geometry

Use this file when the task involves detector geometry assets, simulation geometry loading, or geometry-related navigation.

## Start Here

- `AtDetectors/AtTpc/AtTpc.cxx`
- `AtDetectors/AtTpc/AtTpc.h`
- `geometry/`
- `macro/examples/run_sim.C`
- `macro/tests/AT-TPC/run_sim_attpc.C`
- `macro/tests/generateGeometry.sh`

If the task is about pad centers, pad IDs, or pad-plane layout, also inspect:

- `AtMap/AtTpcMap.cxx`
- `AtMap/AtSpecMATMap.cxx`

## Runtime Path

Simulation macros pass a geometry file to `AtTpc` with `SetGeometryFileName()`:

```cpp
AtTpc *tpc = new AtTpc("ATTPC", kTRUE);
tpc->SetGeometryFileName("ATTPC_v1.2.root");
run->AddModule(tpc);
```

`AtTpc::ConstructGeometry()` dispatches by extension:

- `.root` -> `ConstructRootGeometry()`
- `.geo` -> ASCII branch exists but is not the practical path here

For this tree, `AtTpc` geometry is normally loaded from generated ROOT files.

## Where Geometry Lives

`geometry/` contains:

- generator macros such as `ATTPC_v1_2.C`, `ATTPC_He1bar.C`, `GADGET_II.C`
- generated outputs such as `ATTPC_v1.2.root` and `ATTPC_v1.2_geomanager.root`
- `media.geo` for material definitions
- detector-specific helper assets

Do not assume geometry outputs are missing. This checkout already contains several generated `.root` geometry files.

## Generation

Generate a geometry only if the required output is missing or stale.

Example:

```bash
cd geometry
root -l -q ATTPC_v1_2.C
```

This writes:

- `ATTPC_v1.2.root`
- `ATTPC_v1.2_geomanager.root`

Test helper:

```bash
macro/tests/generateGeometry.sh
```

That script generates only a fixed subset of geometries used by tests. It is not a full geometry build step.

## Detector Variants

| Script | Detector | Notes |
|--------|----------|-------|
| `ATTPC_v1_2.C` | Full AT-TPC | Used by most unpacking/display macros |
| `ATTPC_v1_1.C` | Full AT-TPC | Older version; some example sim macros still reference this |
| `ATTPC_Proto_v1_0.C` | Prototype AT-TPC | Shorter drift length (500 mm) |
| `ATTPC_He1bar.C` | AT-TPC, He 1 atm | Used by the AT-TPC integration test |
| `ATTPC_He*.C`, `ATTPC_D*.C`, `ATTPC_H*.C` | Gas/pressure variants | Specific gas fill and pressure combinations |
| `SpecMAT_*.C` | SpecMAT | Different pad geometry; scintillator variants available |
| `GADGET_II.C` | GADGET-II | Cylindrical TPC variant |
| `FRIB_TPC_v*.C` | FRIB TPC | |

## Macro Anchors

Use real macros, not isolated snippets, when tracing behavior:

- `macro/examples/run_sim.C`
- `macro/tests/AT-TPC/run_sim_attpc.C`

Those show the surrounding `FairRunSim` setup, `run->SetMaterials("media.geo")`, and the other modules commonly registered with the TPC geometry such as `AtCave`, `AtMagnet`, and `AtPipe`.

## Variant Selection

There is no single geometry file that is clearly canonical across the tree.

Different workflows use different files:

- unpacking and display macros often use `ATTPC_v1.2.root`
- older example simulation macros use `ATTPC_v1.1.root`
- the AT-TPC integration test uses `ATTPC_He1bar.root`

When editing a workflow, trust the specific macro or test in that workflow over generic prose.

## Materials

`run->SetMaterials("media.geo")` supplies material definitions used by geometry generation and simulation setup.

Common modifications to `geometry/media.geo`:

- **Gas composition**: update the `AtTPC_Gas` entry — change density, atomic composition, and weight fractions
- **Window materials**: modify the entrance window material entry
- **Add a material**: append a new block following the existing format; the file header documents the required fields (`ncomp`, `aw`, `an`, `dens`, `wm`, `sensflag`, `fldflag`, `fld`, `epsil`)

If `media.geo` changes, regenerate any geometry outputs that depend on it.

## Geometry vs Mapping

Keep these separate:

- transport geometry: `AtDetectors/AtTpc`, `geometry/*.C`, simulation macros
- pad mapping and display-plane geometry: `AtMap/`, visualization, decoder/mapping setup

If the task mentions pad geometry or lookup tables, start in `AtMap/`, not `AtDetectors/AtTpc`.

## Navigation Order

1. Find the macro, task, or test that names the geometry file.
2. Check whether that `.root` file already exists in `geometry/`.
3. Inspect `AtDetectors/AtTpc/AtTpc.cxx` for the runtime load path.
4. Inspect the matching geometry generator macro in `geometry/` if the asset must change.
5. Branch into `AtMap/` only if the task is really about pad mapping rather than transport geometry.
