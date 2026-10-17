# Units and Conventions

Current unit conventions and known inconsistencies. Survey date: 2026-09.

## Rules

- **Code after the Geant4 boundary uses mm and MeV.** This includes `AtELossModel` ("internal units are MeV/mm"), `AtSimpleSimulation`, `AtKinematics`, clusterization output, and new data classes.
- **Parameter files use each quantity's natural units, by design:** drift velocity in cm/µs, diffusion in cm²/µs, time-bucket width in ns, ionization energy in eV, pressure in torr. They are for people to read and edit; code converts on the backend. Do not change parameter-file units to match internal code units. Table of keys and units: [../reference/parameters.md](../reference/parameters.md).
- **Geant4/VMC data uses cm, GeV and ns.** This covers `AtMCPoint` and `AtMCTrack`. Convert to mm and MeV at the boundary, not deeper in the pipeline.

## Known Inconsistencies

- **Double conversion in the simple simulation.** `AtSimpleSimulation` works in mm and MeV but writes `AtMCPoint` in cm and GeV (`AddHit`: `/10.`, `/1000.`). `AtClusterize` then converts straight back to mm. The only reason is sharing `AtMCPoint` with the Geant4 path.
- **The z axis flips between simulation and analysis.** Simulation puts z = 0 at the entrance window and the pad plane at z = 1000 mm; data analysis measures z from the pad plane. It is handled by hardcoded `1000 - z` in several places, e.g. `AtSimpleSimulation.cxx` (space-charge block) and `AtMCFission.cxx` (vertex parameter). New code should state which convention it uses.
- **Coordinates are sometimes time.** `AtSimulatedPoint` stores (mm, mm, µs), and `AtHit` position units are not documented.
- **The fitter hardcodes detector constants.** `AtMCFission.cxx` computes one time bucket as `0.320 * .815 * 10` mm instead of reading `AtDigiPar`.
- **The mass per nucleon differs by call site.**
  - `AtKinematics.cxx` uses 931.49401, 931.494 and 931.5 (`AtoE()`).
  - `AtTpc.cxx` uses 0.93149401 GeV.
  - Macros sometimes use 939, the nucleon mass, which is about 0.8% higher than the atomic mass unit.

## Suggested Cleanup (not scheduled)

- Use a single mass constant, e.g. `AtTools::Kinematics::AtoE()`.
- Document the units of `AtHit` position and the z convention in `AtHit.h`.
- Read detector constants in the fitter from `AtDigiPar`.
