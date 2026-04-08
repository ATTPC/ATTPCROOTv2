# Energy Loss

ATTPCROOT provides several energy-loss utilities in `AtTools/`. The modern model interface is `AtTools::AtELossModel`; `AtTools::AtELossManager` is a separate legacy lookup-table helper, not the owner or facade for all models.

## Main Types

| Type | Role |
|------|------|
| `AtELossModel` | abstract interface for stopping power, range, energy loss, and residual energy |
| `AtELossCATIMA` | CATIMA-backed implementation |
| `AtELossTable` | table-backed implementation, typically from SRIM-style data |
| `AtELossBetheBloch` | standalone Bethe-Bloch helper for simpler analytic use cases |
| `AtELossModelFactory` | abstract factory for creating models from ROOT geometry materials |
| `AtELossFactoryCATIMA` | CATIMA-backed factory; stores a `catima::Config` applied to all created models |
| `AtELossFactoryBetheBloch` | Bethe-Bloch-backed factory; stateless |
| `AtELossManager` | older lookup-table utility; do not treat it as the generic `AtELossModel` entry point |

## `AtELossModel` Interface

All `AtELossModel` implementations expose the same core API:

- `GetdEdx(energy)`
- `GetRange(energyIni, energyFin = 0)`
- `GetEnergyLoss(energyIni, distance)`
- `GetEnergy(energyIni, distance)`

Units in this layer are MeV and mm unless a class-specific note says otherwise.

## CATIMA

`AtELossCATIMA` is the main model-backed implementation. It wraps the CATIMA library and requires both material configuration and projectile configuration.

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

CATIMA is fetched by CMake if it is not already available locally. It is the only backend in this tree that currently exposes straggling calculations through the `AtELossModel` interface.

## Table and Analytic Helpers

- `AtELossTable` follows the same `AtELossModel` interface, but uses precomputed stopping/range tables.
- `AtELossBetheBloch` is a lighter analytic helper and is not a replacement for the full CATIMA-backed workflow when you need straggling or richer material handling.

## Model Factories

`AtELossModelFactory` is an abstract interface for automatically creating energy loss models from ROOT geometry materials (`TGeoMaterial` / `TGeoMixture`). When used with `AtSimpleSimulation::SetModelFactory()`, models are created on demand as new particle species are encountered during transport — no manual `AddModel()` calls needed.

Two concrete factories are provided:

- **`AtELossFactoryCATIMA`** -- wraps CATIMA. Stores a `catima::Config` that is applied to every model it creates. This is the recommended factory for most use cases.
- **`AtELossFactoryBetheBloch`** -- uses the analytic Bethe-Bloch formula with effective Z/A for mixtures and Bloch approximation for mean ionization energy. Lighter weight but less accurate than CATIMA, especially for straggling.

### Usage

```cpp
auto sim = std::make_unique<AtSimpleSimulation>();
auto factory = std::make_shared<AtTools::AtELossFactoryCATIMA>();
// Optional: factory->SetConfig(myCatimaConfig);
sim->SetModelFactory(factory);
```

When a particle species (Z, A) is first encountered without a registered model, the factory extracts the material composition and density from the geometry at the particle's position and creates an appropriate `AtELossModel`.

Manually registered models via `AddModel()` take precedence -- the factory is only consulted for species without an explicit model.

### Factory vs manual registration

Use the **factory** approach when you want simpler setup or are running exploratory simulations where you may not know all particle species in advance. Use **manual `AddModel()`** when you need specific nuclear masses, custom material compositions, or non-standard density values.

### Utility methods

`AtELossModelFactory` provides static utility methods usable by any code that works with ROOT geometry materials:

- `ExtractComposition(material)` -- extracts `(A, Z, stoichiometry)` tuples from a `TGeoMaterial` or `TGeoMixture`
- `WeightFractionsToStoichiometry(weights, atomicMasses)` -- converts weight fractions to integer stoichiometry
- `EffectiveMeanIonization(material)` -- computes effective mean ionization energy via Bragg's additivity rule

## Legacy Lookup-Table Path

`AtELossManager` predates the `AtELossModel` hierarchy. It still exists in the tree, but it is a separate lookup-table class with its own API such as `GetEnergyLoss(...)`, `GetFinalEnergy(...)`, and `GetDistance(...)`.

Use it when you specifically need that legacy behavior; do not document or refactor it as if it were the generic model manager for the rest of `AtTools/`.
