# Event Generators

ATTPCROOT uses FairRoot's generator framework. All reaction generators subclass `AtReactionGenerator`, which itself subclasses `FairGenerator`.

## AtReactionGenerator

`AtReactionGenerator` is the abstract base for all physics reaction generators in `AtGenerators/`.

**What to implement:** Subclasses implement `GenerateReaction(FairPrimaryGenerator*)`. This method is responsible for computing the kinematics of the reaction and adding primaries to the stack.

**What not to touch:** `ReadEvent()` is declared `final`. It handles the beam/reaction event alternation via `AtVertexPropagator` and calls `GenerateReaction()` internally. Do not override it.

## AtVertexPropagator

`AtVertexPropagator` is a singleton that passes vertex position, beam momentum, and kinematics between generators and downstream tasks (e.g., digitization tasks that need beam parameters).

```cpp
AtVertexPropagator::Instance()->GetVertex();   // read vertex
AtVertexPropagator::Instance()->EndEvent();    // called at end of each event to alternate beam/reaction
```

**In unit tests:** Call `AtVertexPropagator::Instance()->ResetForTesting()` in `SetUp()` to start from a clean state between tests.

## Sequential Decays

Generators can be chained for multi-step decays. Enable on the upstream generator:

```cpp
generator->SetSequentialDecay(true);
```

The second generator in the chain reads the vertex and momentum from `AtVertexPropagator` rather than sampling a fresh beam.

## Beam/Reaction Alternation

By default, `ReadEvent()` alternates between inserting a beam event (no reaction) and a full reaction event. This models the AT-TPC's operation where beam tracks are recorded alongside reaction tracks for calibration. The alternation is controlled via `AtVertexPropagator::EndEvent()`.
