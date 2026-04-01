# Pulse Shape Analysis (PSA)

PSA extracts hit positions and charge information from raw pad traces (`AtRawEvent`) to produce an `AtEvent` containing `AtHit` objects.

## Class Hierarchy

`AtPSA` is the abstract base class in `AtReconstruction/AtPulseAnalyzer/`. All concrete PSA implementations inherit from it.

| Class | Hits per pad | Description |
|-------|-------------|-------------|
| `AtPSAMax` | 1 | Places one hit at the time bucket of peak ADC. Charge stored is the peak ADC value; total trace integral also stored on the hit. Optional sub-TB time correction via charge-weighted average of peak region. |
| `AtPSAFull` | 1 or many | Designed for long traces (e.g. beam tracks). Traces with a signal window <50 TBs get one hit at the peak; traces ≥50 TBs are split into 50-TB windows with one hit per window and charge = average ADC in that window. |
| `AtPSASpectrum` | 1 or many | Uses ROOT `TSpectrum` for peak finding; can find multiple peaks per pad. Supports optional background subtraction and interpolation. |
| `AtPSAHitPerTB` | many | One hit per time bucket above threshold. Produces a dense 3D charge distribution with no peak-finding assumptions. Configurable TB range. |
| `AtPSATBAvg` | many | Divides the trace into windows of N TBs (default 5) and creates one hit per window whose average charge exceeds threshold. Hit Z is at window center. Skips saturated pads when a max threshold is set. Can operate on a pad augment instead of the raw ADC. |
| `AtPSADeconv` | 1 | FFT-deconvolves the electronics response function from the raw trace and applies a Butterworth low-pass filter to recover the ionization current. Reconstructed charge saved as `"Qreco"` augment on the pad. Hit placed at charge-distribution weighted average. Requires a response function (`SetResponse`). |
| `AtPSADeconvFit` | 1 | Extends `AtPSADeconv`; instead of the weighted average, fits the deconvolved charge distribution with a Gaussian using the longitudinal diffusion coefficient for improved Z resolution. |
| `AtPSAIterDeconv` | 1 | Extends `AtPSADeconv` with iterative corrections to the reconstructed current. |
| `AtPSASimple2` | — | **Deprecated.** Use `AtPSASpectrum` or `AtPSAMax` instead. |

## Running PSA

PSA is applied by `AtPSAtask`, which wraps any `AtPSA` subclass:

```cpp
auto psa = std::make_unique<AtPSAMax>();
psa->SetThreshold(45);                          // ADC threshold; optional
auto psaTask = new AtPSAtask(std::move(psa));
psaTask->SetInputBranch("AtRawEventFiltered");  // default: "AtRawEvent"
psaTask->SetPersistence(true);
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
- Charge (meaning depends on algorithm — peak ADC, window average, etc.)
- Trace integral (sum of all ADC values over the trace)
- Pad number and timestamp metadata
