# Resume: UKF Task Integration

## Status
Plan is fully approved and ready to implement. No code has been written yet.

## Branch
`OpenKF-Claude` (based on `develop`)

## Plan file
Full implementation plan is at: `.claude/plans/keen-marinating-wind.md` (inside the repo)

## Summary of what to do (in order)

1. **Merge `RealAurio/fitterRefactor` into `OpenKF-Claude`** — this PR (#254) refactors `AtFittedTrack` (new nested structs), introduces `EventFit::AtFitter` base class, adds `AtFitMetadata`/`AtFitTrackMetadata`, and deprecates old classes as `*Old`. 36 files changed.

2. **Add `fSmoothedPositions` to `AtFittedTrack::TrackProperties`** — `std::vector<ROOT::Math::XYZPoint>` for the UKF smoothed trajectory.

3. **Implement `EventFit::AtFitterUKF`** in `AtReconstruction/AtFitter/AtFitterUKF.h/cxx`:
   - Inherits `EventFit::AtFitter`, implements `GetFittedTrack(AtTrack*, ...)`
   - Momentum seed from circle-fit Brho (same as GenFit): `brho = B*radius/sin(theta)` → `AtKinematics::GetMomFromBrho()`
   - Internally owns a `kf::TrackFitterUKF` (already implemented)
   - Outputs smoothed vertex state + smoothed positions into `AtFittedTrack`
   - Use plain C++ types (`double`/`int`/`bool`), not ROOT typedefs — class is never persisted

4. **Update CMakeLists + LinkDef**:
   - Add `AtFitter/AtFitterUKF.cxx` to `AtReconstruction/CMakeLists.txt`
   - `AtReconstructionLinkDef.h`: `EventFit::AtFitterUKF -!;`
   - `AtDataLinkDef.h`: verify `AtFittedTrack +;`, `AtFittedTrack::TrackProperties +;`, new metadata classes

5. **Write gtests** in `AtReconstruction/AtFitter/AtFitterUKFTest.cxx` (registered in existing `OpenKFTests` target). Cluster positions hardcoded (no file access). Key tests: construction, skip-few-clusters, reasonable kinematics, KE within 20% of truth, smoothed positions populated, metadata converged, `AtFittedTrack` round-trips.

6. **Write usage macro** `macro/tests/UKF/UKFTask.C`.

## Key constraints (from CLAUDE.md)
- Tests must NOT access external files — hardcode data; VMCWORKDIR repo files are OK
- `clang-format-17` all new files
- `cmake --build build -j10`
- `cd build && ctest -V -R OpenKFTests`
