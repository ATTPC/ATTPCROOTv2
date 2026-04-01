# Simulation Pipeline

The simulation pipeline converts a generated reaction into MC truth, simulated detector responses, and finally `AtRawEvent` output that can be passed into reconstruction.

There are currently two ways to generate the simulation-side `AtMCPoint` input:

- the full FairRoot/VMC transport path used by most detector simulations
- a lighter-weight model-driven path built around `AtSimpleSimulation`

## Flow

The overall simulation flow is shared downstream of `AtMCPoint` generation:

```
FairPrimaryGenerator + AtReactionGenerator   AtSimpleSimulation
                 │                                  │
                 ▼                                  ▼
        Geant4 / VMC transport              direct AtMCPoint generation
                 │                                  │
                 └───────────────┬──────────────────┘
                                 ▼
                             AtMCPoint
        ▼
AtClusterizeTask
        │  converts MC-point deposits -> ionization electron clusters
        ▼
AtPulseTask
        │  drifts electrons and produces simulated pad traces
        └─ output branch: AtRawEvent -> TClonesArray[AtRawEvent]
```

The only difference between the two paths is how `AtMCPoint` objects are generated. After that, the downstream digitization flow is the same.

### AtMCPoint Generation

- **Primary path:** Geant4/VMC transport through the detector geometry produces `AtMCPoint` objects from the generated particles.
- **Secondary path:** `AtDigitization/AtSimpleSimulation.h` advances particles through the active volume in fixed steps, applies an `AtTools::AtELossModel`, and writes `AtMCPoint` objects directly.

In this tree, `AtSimpleSimulation` uses straight-line propagation. Future versions may extend this step to non-linear tracks and more general transport models.

### Shared Downstream Stages

Once `AtMCPoint` objects exist, the remaining stages are shared:

- `AtClusterizeTask` converts energy deposits into ionization-electron clusters represented as `AtSimulatedPoint`
- `AtPulseTask` drifts those electrons to the pad plane and produces simulated traces in `AtRawEvent`

## Key Runtime Objects

- `AtVertexPropagator`
  shared simulation-side state between generators and downstream logic
- `AtMCTrack`
  simulated particle tracks
- `AtMCPoint`
  MC-point type used by digitization logic; also the concrete hit type written by `AtSimpleSimulation`
- `AtRawEvent`
  final simulated raw traces used by reconstruction

See [data-model.md](../reference/data-model.md) for the object-level view and [branch-io-contracts.md](../reference/branch-io-contracts.md) for task branch names.

## Event Structure

The FairRoot/VMC generator path represents each physical beam-induced event as two consecutive FairRoot events:

- Even-indexed event (0, 2, 4, …): beam phase — the beam particle traverses the detector
- Odd-indexed event (1, 3, 5, …): reaction phase — the reaction products are transported

Events 0+1 form one complete beam-induced event; events 2+3 form the next, and so on. Code that loops over events or checks event indices must account for this pairing.

**Track IDs:** Within each FairRoot event the beam particle always has `fTrackID = 0`. Reaction products receive subsequent IDs.

## Required Pieces

For the FairRoot/VMC path, a simulation run needs:

- detector geometry
- a configured `FairPrimaryGenerator` with one or more `AtReactionGenerator` subclasses
- the digitization stages `AtClusterizeTask` and `AtPulseTask`
- the experiment parameter set used by digitization

For the `AtSimpleSimulation` path, the run replaces VMC transport with:

- an `AtSimpleSimulation` instance
- one or more configured `AtTools::AtELossModel` instances
- the same downstream digitization stages if `AtRawEvent` output is needed

See [generators.md](generators.md) for generator behavior and [energy-loss.md](energy-loss.md) for the model layer used by `AtSimpleSimulation`.
