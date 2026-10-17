# Energy Loss

ATTPCROOT provides several energy-loss utilities in `AtTools/`. The model interface is `AtTools::AtELossModel`; model instances are served by an `AtTools::AtELossManager` (and subclasses) to transport code such as `AtSimTransport`.

## Main Types

| Type | Role |
|------|------|
| `AtELossModel` | abstract interface for stopping power, range, energy loss, and residual energy |
| `AtELossCATIMA` | CATIMA-backed implementation |
| `AtELossTable` | table-backed implementation, typically from SRIM/LISE-style data |
| `AtELossBetheBloch` | analytic Bethe-Bloch implementation |
| `AtELossManager` | accepts pre-built models and serves them to transport; base class is accept-only |
| `AtELossManagerBetheBloch` | `AtELossManager` subclass that also auto-generates Bethe-Bloch models from geometry materials |
| `AtELossManagerCATIMA` | `AtELossManager` subclass that auto-generates CATIMA models; carries a `catima::Config` |

## `AtELossModel` Interface

All `AtELossModel` implementations expose the same core API:

- `GetdEdx(energy)`
- `GetRange(energyIni, energyFin = 0)`
- `GetEnergyLoss(energyIni, distance)`
- `GetEnergy(energyIni, distance)`

Units in this layer are MeV and mm unless a class-specific note says otherwise.

## CATIMA

`AtELossCATIMA` wraps the CATIMA library and requires both material configuration and projectile configuration.

Typical setup:

```cpp
using MaterialComp = std::tuple<int, int, int>;

std::vector<MaterialComp> material = {
   {1, 1, 2},   // example stoichiometry entries
   {12, 6, 1},
};

AtTools::AtELossCATIMA model(density_g_cm3, material);
model.SetProjectile(projectileA, projectileZ, projectileMassAmu);

double dE = model.GetEnergyLoss(energyMeV, distanceMm);
double range = model.GetRange(energyMeV);
double residual = model.GetEnergy(energyMeV, distanceMm);
```

## Table and Analytic Helpers

- `AtELossTable` — same `AtELossModel` interface, backed by precomputed stopping/range tables loaded via `LoadSrimTable(...)` / `LoadLiseTable(...)`.
- `AtELossBetheBloch` — analytic helper, constructed directly with projectile charge/mass and target Z/A/density/I.

## `AtELossManager` hierarchy

`AtELossManager` is the single entry point transport code uses to obtain a model for a given `(Z, A, material)` combination. It has three responsibilities:

1. Accept pre-built models via `AddModel(...)`.
2. Serve them on `GetModel(Z, A, massAmu, material)`.
3. Optionally synthesize a model from geometry data when the cache misses (subclass hook `GenerateModel`).

The base class is **accept-only** — if no registration matches the requested particle + material, `GetModel` returns `nullptr`. Two subclasses add auto-generation:

- **`AtELossManagerBetheBloch`** — synthesizes an `AtELossBetheBloch` from density, Z/A, and mean ionization energy. Mixtures use electron-density-weighted effective Z/A and Bragg's additivity for I.
- **`AtELossManagerCATIMA`** — synthesizes an `AtELossCATIMA`, applying a user-configurable `catima::Config` to every model it creates.

### Registration forms

Two overloads of `AddModel` are available:

```cpp
// Material-agnostic: served whenever the lookup (Z, A) matches, regardless of material.
// Use this for MCFission / AtMCFitter where a single (Z, A) table covers the whole run.
manager->AddModel(Z, A, model);

// Material-specific: served only when the lookup material name matches.
// Takes priority over the agnostic registration when both are present.
manager->AddModel(Z, A, "iC4H10", model);
```

### Lookup priority (inside `GetModel`)

1. Material-specific registration for `(Z, A, material->GetName())`.
2. Material-agnostic registration for `(Z, A)`.
3. Previously auto-generated model cached under `(Z, A, material->GetName())`.
4. Subclass `GenerateModel(...)` — cached on first success.
5. `nullptr` if none of the above produce a model.

`ClearCache()` drops auto-generated entries only; registered models survive.

### Usage with `AtSimTransport` / `AtSimpleSimulation`

```cpp
auto manager = std::make_shared<AtTools::AtELossManagerCATIMA>();
// optionally: manager->SetConfig(myCatimaConfig);

// Pre-register a specific table alongside the auto-generator:
auto pbTable = std::make_shared<AtTools::AtELossTable>();
pbTable->LoadSrimTable("PbinHe.txt");
manager->AddModel(82, 208, pbTable);

auto sim = std::make_unique<AtSimTransport>("ATTPC_He1bar.root", manager);
```

`AtSimTransport` queries the manager at each volume crossing, so crossings into different materials yield different cached models. For the standalone hit-recording class (`AtSimpleSimulation`), the same registrations work via the thin forwarders (`AddModel`, `SetManager`) or by calling through `GetEngine()`.

### Utility methods

The following static helpers on `AtELossManager` are usable by any code that works with ROOT geometry materials:

- `ExtractComposition(material)` — extracts `(A, Z, stoichiometry)` tuples from a `TGeoMaterial` or `TGeoMixture`.
- `WeightFractionsToStoichiometry(weights, atomicMasses)` — converts weight fractions to integer stoichiometry.
- `EffectiveMeanIonization(material)` — effective mean ionization energy via Bragg's additivity rule.
