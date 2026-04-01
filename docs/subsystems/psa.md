# Pulse Shape Analysis (PSA)

PSA extracts hit positions and charge information from raw pad traces (`AtRawEvent`) to produce an `AtEvent` containing `AtHit` objects.

## Class Hierarchy

`AtPSA` is the abstract base class in `AtReconstruction/AtPulseAnalyzer/`. All concrete PSA implementations inherit from it.

| Class | Algorithm |
|-------|-----------|
| `AtPSAMax` | Peak-finding: hit at the time bucket of maximum signal |
| `AtPSAFull` | Integrates the full trace; produces one hit per pad |
| `AtPSADeconv` | Deconvolution-based hit finding; can find multiple hits per pad |

## Running PSA

PSA is applied by `AtPSAtask`, which wraps any `AtPSA` subclass:

```cpp
auto psa = std::make_unique<AtPSAMax>();
auto psaTask = new AtPSAtask(std::move(psa));
fRun->AddTask(psaTask);
```

## Chaining PSA Methods

`AtPSAComposite` allows running multiple PSA methods in sequence on the same event, for example to handle different pad types with different algorithms:

```cpp
auto composite = std::make_unique<AtPSAComposite>();
composite->AddPSA(std::make_unique<AtPSAMax>());
composite->AddPSA(std::make_unique<AtPSADeconv>());
```

## Output

PSA produces an `AtEvent` stored as a branch in the output TTree. Each `AtHit` in the event carries:
- 3D position (x, y, time-bucket → z via drift velocity)
- Integrated charge
- Pad number and other metadata
