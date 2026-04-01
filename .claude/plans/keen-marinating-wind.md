# Plan: Integrate UKF Track Fitter into Task System

## Context
The UKF track fitter (`kf::TrackFitterUKF`) is fully implemented as a standalone physics class and tested via macros in `macro/tests/UKF/`. It needs to be integrated into the FairTask-based reconstruction pipeline so it can be run event-by-event on data.

In parallel, PR #254 (`RealAurio/fitterRefactor`) proposes a clean refactored fitter architecture with a new `EventFit::AtFitter` base class and a redesigned `AtFittedTrack`. The UKF task should be implemented against this new interface rather than the deprecated `AtFITTER::AtFitter`.

**Outcome:** A complete `EventFit::AtFitterUKF` class that reads an `AtPatternEvent` (via `AtFitterTask`), fits each track with the UKF using a Brho-based momentum seed, and writes an `AtTrackingEvent` with `AtFittedTrack` objects containing vertex kinematics and smoothed trajectory positions.

---

## Step 1 — Merge fitterRefactor (PR #254) into OpenKF-Claude

Merge `RealAurio/fitterRefactor` into the `OpenKF-Claude` branch. This brings in all 36 changed files. Key files introduced/changed:

| File | Change |
|------|--------|
| `AtData/AtFittedTrack.h/cxx` | New nested-struct design (`Kinematics`, `ParticleInfo`, `TrackProperties`) |
| `AtData/AtFitMetadata.h/cxx` | New: event-level fit metadata |
| `AtData/AtFitTrackMetadata.h/cxx` | New: per-track fit statistics |
| `AtData/AtFittedTrackOld.h/cxx` | Backward-compat alias for old AtFittedTrack |
| `AtData/CMakeLists.txt` | Adds new data classes |
| `AtData/AtDataLinkDef.h` | Adds ROOT dictionary entries |
| `AtReconstruction/AtFitter/AtFitter.h/cxx` | New `EventFit::AtFitter` with `FitEvent()`/`GetFittedTrack()` |
| `AtReconstruction/AtFitter/AtFitterOld.h/cxx` | Old `AtFITTER::AtFitter` kept for backward compat |
| `AtReconstruction/AtFitter/AtGenfit.h` | Now inherits `AtFitterOld` (deprecated, not changed) |
| `AtReconstruction/AtFitter/AtMCFitter.cxx` | Minor updates |
| `AtReconstruction/AtFitterTask.h/cxx` | Updated to use `EventFit::AtFitter`; adds optional RawEvent/Event/FitMetadata branches |
| `AtReconstruction/AtReconstructionLinkDef.h` | Updated LinkDef |
| `AtReconstruction/AtPatternModification/*` | New BraggCurveFinder task (independent) |
| `AtEventDisplay/AtTabs/*` | Tab updates (independent) |

Resolve any merge conflicts; most conflict risk is in CMakeLists and LinkDef files.

---

## Step 2 — Add Smoothed Positions to `AtFittedTrack::TrackProperties`

**File:** `AtData/AtFittedTrack.h`

Add to `TrackProperties` struct:
```cpp
std::vector<ROOT::Math::XYZPoint> fSmoothedPositions; // UKF-fitted positions at each cluster
```

Add corresponding setter/getter to `AtFittedTrack`:
```cpp
void SetSmoothedPositions(std::vector<ROOT::Math::XYZPoint> pos) { fTrackProperties.fSmoothedPositions = std::move(pos); }
const std::vector<ROOT::Math::XYZPoint>& GetSmoothedPositions() const { return fTrackProperties.fSmoothedPositions; }
```

Bump `ClassDef(AtFittedTrack, 3)`.

---

## Step 3 — Implement `AtFitterUKF`

**New files:**
- `AtReconstruction/AtFitter/AtFitterUKF.h`
- `AtReconstruction/AtFitter/AtFitterUKF.cxx`

**Type convention (from CLAUDE.md):** `AtFitterUKF` is never written to disk, so use plain C++ types (`double`, `int`, `bool`) — not ROOT typedefs (`Double_t`, `Int_t`, `Bool_t`).

### Header

```cpp
namespace EventFit {

class AtFitterUKF : public AtFitter {
public:
   AtFitterUKF(double charge, double mass_MeV,
               std::unique_ptr<AtTools::AtELossModel> elossModel);

   void SetBField(ROOT::Math::XYZVector bField); // Tesla
   void SetEField(ROOT::Math::XYZVector eField); // V/m (default 0)
   void SetUKFParameters(double alpha, double beta, double kappa);
   void SetMeasurementSigma(double sigma_mm);    // default 1.0 mm
   void SetMomentumSigmaFrac(double frac);       // fractional, default 0.1
   void SetMinClusters(int n);                   // default 3

   void Init() override {}

protected:
   AtFittedTrack *GetFittedTrack(AtTrack *track,
                                 AtFitMetadata *fitMetadata = nullptr,
                                 AtRawEvent *rawEvent = nullptr,
                                 AtEvent *event = nullptr) override;
private:
   // Helpers
   std::unique_ptr<kf::TrackFitterUKF> CreateUKF() const;
   ROOT::Math::XYZPoint  GetInitialPosition(AtTrack *track) const;
   ROOT::Math::XYZVector GetInitialMomentum(AtTrack *track) const;
   TMatrixD              GetInitialCovariance(double p_mag_MeV) const;
   TMatrixD              GetMeasCovariance() const;
   double                GetBrho(AtTrack *track) const;

   // Particle / detector config (plain C++ types — not persisted)
   double fCharge;
   double fMass_MeV;
   ROOT::Math::XYZVector fBField{0, 0, 2.85};
   ROOT::Math::XYZVector fEField{0, 0, 0};
   std::unique_ptr<AtTools::AtELossModel> fELossModel;

   // UKF tuning
   double fAlpha{1e-3}, fBeta{2.0}, fKappa{0.0};
   double fMeasSigma_mm{1.0};
   double fMomSigmaFrac{0.1};
   int    fMinClusters{3};

   AtTools::AtKinematics fKinematics; // for GetMomFromBrho

   ClassDef(AtFitterUKF, 1);
};

} // namespace EventFit
```

### Implementation logic in `GetFittedTrack()`

1. **Get clusters**: `auto *clusters = track->GetHitClusterArray(); track->SortClusterHitArrayZ();`
   Skip track if `clusters->size() < fMinClusters`.

2. **Seed momentum from circle fit** (same as GenFit):
   ```
   radius = track->GetGeoRadius() / 1000.0  // mm → m
   theta  = track->GetGeoTheta()
   phi    = track->GetGeoPhi()
   brho   = fBField.Z() * radius / sin(theta)  // T·m
   [p_GeV, KE_GeV] = fKinematics.GetMomFromBrho(mass_amu, charge, brho)
   p_MeV = p_GeV * 1000.0
   ```

3. **Initial position**: `clusters->front().GetPosition()` (in mm).

4. **Initial momentum direction**: unit vector from (theta, phi); magnitude = `p_MeV`.

5. **Create and configure UKF**:
   ```cpp
   auto ukf = CreateUKF();   // new kf::TrackFitterUKF with AtPropagator+AtRK4AdaptiveStepper
   ukf->setParameters(fAlpha, fBeta, fKappa);
   ukf->SetInitialState(initialPos, initialMom, GetInitialCovariance(p_MeV));
   ukf->SetMeasCov(GetMeasCovariance());
   ukf->fEnableEnStraggling = true;
   ```

6. **Filter loop**:
   ```cpp
   for (size_t i = 1; i < clusters->size(); ++i) {
       XYZPoint measPt = clusters->at(i).GetPosition(); // mm
       ukf->predictUKF(measPt);
       ukf->correctUKF(measPt);
   }
   ukf->smoothUKF();
   ```
   Wrap in try/catch for Cholesky failures → mark `fitConverged = false`.

7. **Build `AtFittedTrack`**:
   - From `smoothedStates[0] = [x, y, z, p, theta_s, phi_s]`:
     - `KE = sqrt(p*p + m*m) - m`
     - `SetKinematics(KE, theta_s, phi_s)`
     - `SetVertex(XYZVector{x, y, z})`
   - `SetParticleInfo(pdg_string, charge, mass_amu)`
   - Extract smoothed positions from all states → `SetSmoothedPositions(...)`
   - Compute `trackLength` as sum of distances between consecutive smoothed positions
   - `SetTrackPropertiesStruct(...)` with `initialPosition`, `trackLength`, `trackPoints = clusters->size()`
   - Create `AtFitTrackMetadata`: set `trackID`, `fitConverged`; chi² = sum of squared smoothed residuals / `ndf`; `SetTrackMetadata()`

8. **Optionally store metadata** in `AtFitMetadata` (if non-null) via `SetTrackMetadatasVector(trackID, ...)`.

---

## Step 4 — Update Build System and LinkDef

**File:** `AtReconstruction/CMakeLists.txt`
- Add `AtFitter/AtFitterUKF.cxx` to `SRCS`

**File:** `AtReconstruction/AtReconstructionLinkDef.h`
- Add `#pragma link C++ class EventFit::AtFitterUKF-!;` — not persisted to disk; suppress streamer

**File:** `AtData/AtDataLinkDef.h` (already updated by Step 1 merge, but verify):
- `AtFittedTrack +;` — persisted, needs full streamer
- `AtFitMetadata +;`, `AtFitTrackMetadata +;` — persisted
- `AtFittedTrack::TrackProperties +;` — nested struct in a persisted class needs its own entry
- `AtFittedTrackOld -!;`, etc. — backward-compat wrappers, no streamer needed

**Code formatting:** Run `clang-format-17 -i` on all new/modified `.h` and `.cxx` files before committing, or use `scripts/formatAll.sh`.

---

## Step 5 — GTest Unit Tests

**New file:** `AtReconstruction/AtFitter/AtFitterUKFTest.cxx`

Register in `AtReconstruction/CMakeLists.txt` inside the existing `if(TARGET Eigen3::Eigen)` block (same block as `TrackFitterUKFTest.cxx`):

```cmake
set(OPENK_TEST_SRCS
  ...
  AtFitter/TrackFitterUKFTest.cxx   # already there
  AtFitter/AtFitterUKFTest.cxx      # new
)
attpcroot_generate_tests(OpenKFTests SRCS ${OPENK_TEST_SRCS} DEPS AtReconstruction)
```

Use the same test-class subclassing pattern as `TrackFitterUKFTest.cxx`: define `AtFitterUKFTestable` that exposes internals.

### Test fixture

From CLAUDE.md: **"Tests must not access external files or network resources."** Cluster positions are therefore hardcoded directly from the known simulation track (same values used in `UKFSingleTrack.C`). The energy loss model uses `AtELossTable` loaded via `$VMCWORKDIR` — the same `getEnergyPath()` pattern already present in `TrackFitterUKFTest.cxx` (file is part of the repo, not external).

```cpp
class AtFitterUKFFixture : public testing::Test {
protected:
   static constexpr double mass_p  = 938.272;           // MeV/c²
   static constexpr double charge_p = 1.602176634e-19;  // C
   static constexpr double B_z = 2.85;                  // T

   std::unique_ptr<EventFit::AtFitterUKF> fitter;

   // Cluster positions hardcoded from simulation (mm) — no file access
   const std::vector<ROOT::Math::XYZPoint> kClusters = {
       {-3.4e-4, -1.5e-4, 1.0018},   // converted from cm, first point
       {-14.895, -48.787, 10.1217},
       // ... (5–8 representative points from the known proton track)
   };

   void SetUp() override {
       auto eloss = std::make_unique<AtTools::AtELossTable>(0);
       eloss->LoadSrimTable(getEnergyPath()); // repo file via VMCWORKDIR
       eloss->SetDensity(3.553e-5);

       fitter = std::make_unique<EventFit::AtFitterUKF>(charge_p, mass_p, std::move(eloss));
       fitter->SetBField({0, 0, B_z});
       fitter->SetUKFParameters(1e-3, 2, 0);
       fitter->SetMeasurementSigma(1.0);
   }

   // Build AtTrack with hardcoded clusters + PRA-derived Brho seed
   AtTrack BuildTrack(bool tooFewClusters = false);
};
```

### Tests to implement

| Test name | What is checked |
|-----------|----------------|
| `Construction` | `AtFitterUKF` constructs without throwing |
| `SkipsTrackWithFewClusters` | Returns `nullptr` when cluster count < `fMinClusters` |
| `FittedTrackHasReasonableKinematics` | Fits synthetic proton track; `KE > 0`, `0 < theta < π`, smoothed positions non-empty |
| `FittedTrackKinematicsWithinTolerance` | Fitted KE within 20% of known true value (~47 MeV total for simulation proton) |
| `SmoothedPositionsPopulated` | `GetSmoothedPositions().size() == nClusters - 1` |
| `MetadataConvergedSet` | `GetTrackMetadata()->GetFitConverged() == true` for good track |
| `AtFittedTrackKinematicsRoundTrip` | `SetKinematics(E,θ,φ)` → `GetKinematics()` returns same values (no physics needed) |
| `AtFittedTrackSmoothedPositionsRoundTrip` | `SetSmoothedPositions(v)` → `GetSmoothedPositions()` returns same vector (no physics needed) |

The round-trip tests (`AtFittedTrack*`) require no energy loss model and test the data class in isolation.

---

## Step 6 — Write Usage Macro

**New file:** `macro/tests/UKF/UKFTask.C`

Demonstrates:
```cpp
auto eloss = std::make_unique<AtTools::AtELossCATIMA>(3.553e-5);
eloss->SetProjectile(1, 1, 1);
eloss->SetMaterial({{1, 1, 1}});

auto fitter = std::make_unique<EventFit::AtFitterUKF>(charge_p, mass_p, std::move(eloss));
fitter->SetBField({0, 0, 2.85});
fitter->SetUKFParameters(1e-3, 2, 0);
fitter->SetMeasurementSigma(1.0);

auto task = new AtFitterTask(std::move(fitter));
task->SetPersistence(kTRUE);
run->AddTask(task);
```

---

## MCFitter Assessment

`AtMCFitter` uses stochastic multi-iteration Monte Carlo optimization and produces `AtMCResult` arrays — not per-track `AtFittedTrack` objects. Its `Exec(AtPatternEvent&)` pattern is fundamentally different from `GetFittedTrack(AtTrack*)`. It does **not** combine well with the new `EventFit::AtFitter` base and should remain separate (`AtMCFitterTask`).

---

## Verification

1. **Build**: `cmake --build build -j10` — must compile without errors or clang-tidy warnings.
2. **Format**: `clang-format-17 -i` on all new files (or `scripts/formatAll.sh`).
3. **Unit tests**: `cd build && ctest -V -R OpenKFTests` — all new gtest cases must pass.
4. **Existing macro**: Run `macro/tests/UKF/UKFSingleTrack.C` to confirm `TrackFitterUKF` still works standalone.
5. **New macro**: Run `macro/tests/UKF/UKFTask.C` end-to-end on a simulation file; inspect the `AtTrackingEvent` branch in the output ROOT file — verify `AtFittedTrack` contains non-trivial kinematics and smoothed positions.
6. **Check merge**: Verify `AtGenfit` still compiles (it uses `AtFitterOld` which is unchanged).
