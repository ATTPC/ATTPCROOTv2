# Simulation, Digitization and MC Fit Speed

Results of the 2026-10 study of the CPU cost and thread scaling of digitization (`AtClusterize`, `AtPulse`, `AtPulseLine`) and of the MC fitter (`AtMCFitter`), without changing the physics. How the fitter parallelizes and its known bugs are in [mc-fitter-threading.md](mc-fitter-threading.md).

## Summary

- **Digitization** of Geant4 AT-TPC events is 15–25% faster and statistically equivalent to before. See [Geant4 simulation and digitization](#geant4-simulation-and-digitization).
- **The MC fit** costs about 1.4 times less CPU per event at 4 threads from the library changes alone. With the existing `AtPulse::SetDoConvolution(false)` (see [Usage](#usage)), the measured total is 4.9 times less, with the same fit results.
- **Fitter thread scaling.** The fitter no longer serializes on ROOT's global lock, so wall time per event keeps falling up to 50–64 threads: 3.85 s/event at 64 threads, against 27.7 s before and 10.7 s at the old best (16 threads).
- **The fit is reproducible.** It can be seeded, and each worker thread draws from its own generator.

## What Changed

| Change | Where | Effect on output |
|---|---|---|
| Seedable per-thread RNG | `AtTools::Random`, `AtMCFitter::SetSeed`, parameter distributions, `AtClusterize`, `AtPulse`, `AtPulseLine`, `AtPulseLineTask` | none outside the fitter; `Random::Get()` is `gRandom` unless a generator is installed |
| Pad lookup grid in front of `TH2Poly::FindBin` | `AtMap::GetPadNum`, `AtMap/AtPadLookup` | bit-identical; `AtMap::SetUsePadLookup(false)` turns it off |
| Pad inhibition looked up once per pad, not per time bucket | `AtPulse::GetGain`, `AtPulseLine` | bit-identical |
| Bin centers hoisted out of the time-bucket loop | `AtPulseLine` | bit-identical |
| One exact Gamma deviate for the gain of 1–10 electrons | `AtPulse::MeanPolyaGain` | statistically equivalent, different random stream |
| Minuit2 built directly instead of through `ROOT::Fit::Fitter` | `AtTools::MinimizeMigrad`, `AtPSADeconvFit`, `AtMCFission` | bit-identical |
| Plain Gaussian (`AtTools::GaussianTB`) instead of per-hit `TF1`s | `AtE12014`, `AtPSADeconvFit`, `AtMCFission` | bit-identical |
| "Energy converged" logged at debug level | `AtELossTable` | none; logs were 5 MB per fitted event |

Details:

- **Pad lookup grid.** One immutable `std::vector<int16_t>` over the pad plane at 0.1 mm, 60 MB for the AT-TPC plane. It is built once per process on the first lookup (about 5 s single-threaded) under `std::call_once`, and shared read-only by all threads.
  - A cell stores a pad only if its center and four inset corners agree and no pad edge crosses it. Otherwise the lookup calls `FindBin`, which happens for 10% of the points in the AT-TPC disk.
  - The grid agrees with `FindBin` at every point tested: a 150 M-point scan of the whole plane, and unit tests on the AT-TPC and GADGET II planes. The tests include points 1e-7 mm inside every cell side of the bottom 5 mm of the AT-TPC plane, where `FindBin` changes across internal `TH2Poly` partition lines.
- **Gamma gain.**
  - **The identity.** The Polya gain with shape `b` and mean `G` is a Gamma distribution with shape `b+1` and scale `G/(b+1)`. The mean gain of `n` electrons is therefore one Gamma(`n(b+1)`, `G/(b+1)`) deviate divided by `n`.
  - **How it is drawn.** As an Erlang deviate (minus the log of a product of uniforms) for integer shapes up to 16, and with `std::gamma_distribution` above. This replaces up to ten `TF1::GetRandom` table look-ups per deposit, for deposits of 10 or fewer electrons with the default fast gain and for every deposit with `UseFastGain(false)`.
  - **Accuracy.** It is exact. The old table draw was cut off at 5 times the gain, so its mean was 0.2% low and its variance 2% low. `MeanPolyaGain`'s comment derives the identity, and gtests compare the new draw with the old one and with the exact moments. The derivation and the Polya model itself should get a careful human review.
- **Direct Minuit2.**
  - **The problem.** `ROOT::Fit::Fitter` creates its minimizer through ROOT's plugin manager, which takes ROOT's global write lock (11–16 µs per fit). With about 250 PSA pad fits per fitter iteration, threads queued on that lock: system time was 9.6, 48 and 138 s/event at 16, 32 and 64 threads.
  - **The fix.** `MinimizeMigrad` builds a `Minuit2Minimizer` on a `Chi2FCN` with `Fitter`'s step sizes, call limit (1000 + 100n + 5n²) and default options, and takes no lock. System time is now 1.3, 3.9 and 14.4 s/event at 16, 32 and 64 threads.
- **Thread safety fixes found on the way.**
  - `ContainerManip::CreateHistFromData` no longer toggles the process-wide `TH1::AddDirectory` flag from worker threads.
  - `AtStudentDistribution::SetDoF` now reaches the sampler.
  - `AtPSADeconv` stops with a fatal error when `SetUseSimCharge(true)` meets a pad without a "Q" augment, instead of running its shared, unprotected FFT path.

## Geant4 Simulation and Digitization

**Pending:** timing of the Geant4 simulation and digitization (`macro/tests/AT-TPC`) before and after this branch.

**Equivalence check.** One 500-event Geant4 AT-TPC simulation (`macro/tests/AT-TPC/run_sim_attpc.C`) was digitized with `run_digi_attpc.C` by the old and new code, each with two seeds.

- **Why it is compared statistically.** The new gain draw uses a different number of random numbers, so the same seed gives a different event. Old against new is therefore compared like two seeds of one build.
- **Distributions.** Every distribution agrees with KS p ≥ 0.80 between old and new, and with p ≥ 0.60 over all pairs including seed against seed. The distributions are: fired pads, total ADC and Q, per-pad maximum ADC and its time bucket, PSA hits, hit charge, z and r, and RANSAC track candidates.
- **Fired pads.** The per-event Jaccard overlap of the fired-pad sets is 0.869–0.872 for every pair, seed against seed included.
- **Total charge.** The one systematic difference is +0.04% (±0.003%). That is about four times the seed-to-seed shift and far below the 0.06% per-event spread. It matches the old draw's 0.2% low single-electron mean, diluted because that draw is used only for deposits of 10 or fewer electrons.
- **Speed.** The new code digitizes 15–25% faster.

## MC Fit

**Setup.** Measured with the e12014 fission fit:
- `AtMCFission`, `AtClusterizeLine`, `AtPulseLine`, and `AtPSADeconvFit` with `SetUseSimCharge(true)`.
- 100 iterations, 2 rounds, 4 threads, on 300 simulated events.

The cluster's nodes have 64 cores with two hardware threads each.

**Gate.** Each version is checked against the reference on:
- the mean and σ of ΔZ (fitted minus true fragment Z), within 2σ;
- the fraction of fitted events;
- the two-sample KS p of the ΔZ and objective distributions, above 0.05.

Each fit ran twice (`a` / `b`), and the chunk-to-chunk spread of CPU-s/event is about 9%. ΔZ errors are about ±0.28 on the mean and ±0.20 on σ.

| Code | CPU-s/event (a / b) | Speedup | ΔZ mean | ΔZ σ | KS p (ΔZ, obj) | Gate |
|---|---|---|---|---|---|---|
| reference (library as before this branch) | 533 / 534 | 1.00 | −0.96 / −0.89 | 3.71 / 3.71 | 1.00, 1.00 | ref |
| + per-thread RNG only | 527 / 518 | 1.02 | −0.91 / −0.86 | 3.66 / 3.75 | 1.00, 1.00 | PASS |
| + `SetDoConvolution(false)` only | 227 / 224 | 2.37 | −0.94 / −0.90 | 3.65 / 3.74 | 1.00, 1.00 | PASS |
| + Gamma gain only | 509 / 508 | 1.05 | −0.79 / −0.92 | 3.71 / 3.69 | 1.00, 0.98 | PASS |
| this branch + `SetDoConvolution(false)` | 108 / 107 | 4.9 | −0.93 / −0.98 | 3.69 / 3.69 | 1.00, 1.00 | PASS |

**Per-change savings.** Out of the reference's 533 CPU-s/event, measured change by change on top of the RNG change and scaled from single-thread profiles:

| Change | CPU-s/event saved |
|---|---|
| `SetDoConvolution(false)` | ~310 |
| pad inhibition once per pad | ~50 |
| pad lookup grid | ~45 |
| per-hit `TF1`s removed | ~35 |
| Gamma gain | ~25 |
| per-thread RNG | ~10 |
| "Energy converged" log level | ~2 |

The library changes therefore save about 160 CPU-s/event, about 1.4x at 4 threads, and they add with the convolution skip. Wall time per event at 4 threads went from 136 s to 28 s with both.

### Thread Scaling

One 30-event digitization was fit with 1 to 64 threads, with 100 iterations, 2 rounds, a fixed seed, and `SetDoConvolution(false)` in both versions. "Before" has every change above except direct Minuit2, the plain Gaussian and the prebuilt grid; "after" is this branch. Up to 8 threads, two fitter threads share each core; from 16 threads up, the job had the whole node with unbound threads.

| Threads | Before: wall-s/ev | CPU-s/ev (sys) | After: wall-s/ev | CPU-s/ev (sys) |
|---|---|---|---|---|
| 1 | 70.09 | 69.8 (0.6) | 68.17 | 67.8 (0.6) |
| 2 | 48.55 | 95.5 (0.7) | 47.93 | 94.0 (0.6) |
| 4 | 24.69 | 95.4 (0.8) | 25.35 | 96.9 (0.7) |
| 8 | 13.01 | 94.3 (0.8) | 13.35 | 94.6 (0.6) |
| 16 | 10.73 | 102.0 (9.6) | 8.06 | 96.9 (1.3) |
| 32 | 15.84 | 138.0 (48.1) | 5.21 | 90.9 (3.9) |
| 50 | — | — | 3.69 | 95.3 (8.3) |
| 64 | 27.68 | 231.6 (137.7) | 3.85 | 102.5 (14.4) |

"Before" values are the mean of two repeats, which differ by 1–4% in wall time; "after" values are single runs.

- Up to 8 threads the two agree within 5%. The lock fix matters from 16 threads up.
- **CPU per thread, relative to one thread:**
  - With one thread per core, it is 1.01 at 2 threads and 1.17–1.19 at 4 and 8 threads.
  - With two threads sharing a core, it is 1.39–1.43. That still gives 1.42 times the events per core of one thread alone.
  - At 16 threads and up (unbound), it is 1.34–1.52.
- **Process control.** Sixteen 1-thread fits run at once, each on its own core of one node, used 67.7 CPU-s/event each, the same as one fit alone. The CPU inflation comes from threads in one process or from shared cores, not from a busy node.

### Serial Work per Event

The main thread reads each input event and writes the results and saved events while the workers wait. Wall-s/event, with main-thread seconds per event in parentheses:

| Threads | all input branches read, 10 events saved | unused input `AtRawEvent` branch off, 10 saved | branch off, `SetNumEventsToSave(0)` |
|---|---|---|---|
| 1 | 68.17 (0.57) | 68.29 (0.44) | 69.25 (0.17) |
| 16 | 8.08 (0.53) | 7.54 (0.52) | 7.66 (0.10) |
| 32 | 5.27 (0.58) | 5.12 (0.48) | 4.82 (0.14) |
| 64 | 3.84 (0.55) | 3.75 (0.46) | 3.44 (0.12) |

Not reading the digitized traces saves about 0.1 s/event, and saving no simulated events about 0.35 s more. That is about 10% of the wall time at 32–64 threads, and nothing at 1–4 threads. The 16-thread values carry about ±0.3 s of run-to-run noise.

## Usage

- **Skip the convolution in the fit.** When the PSA reads the simulated charge (`AtPSADeconv::SetUseSimCharge(true)` with `AtPulse::SetSaveCharge(true)`), call `SetDoConvolution(false)` on the fitter's pulse. The convolution was 38% of every iteration and nothing reads its traces. Fit results are bit-identical. The saved simulated events then carry charge but zero traces, and fit files shrink from about 23 to 3.5 MB per event.
- **Throughput (many chunks).** Use few threads per job, two per allocated core. Two threads on one core cost 47.9 core-s/event, against 68.2 for one thread alone on a core.
- **Turnaround on one chunk.** Use 32–50 threads on a whole node: 5.21–3.69 s/event. At 50 threads this holds the node for about 236 core-s/event.
- **Iterations per thread.** Keep `SetNumIter` a multiple of `SetNumThreads`. Each round splits the iterations into contiguous blocks, so 100 iterations on 32 or 64 threads leave at least 22% of the threads idle.
- **Bisecting a digitization difference.** `AtMap::SetUsePadLookup(false)` goes back to `FindBin` alone.

## Open Items

- **Per-thread CPU inflation of 1.34–1.52 at 16 threads and up.** The cause is untested. Candidates are two threads sharing a core (the threads were unbound) and memory effects within the process. To settle it, run 16 and 32 threads bound one per core and compare with the process control.
- **Start-up and the first event.** Init takes 9–12 s per job. At 16 threads and up, the first event takes 10–14 s longer than the rest, of which the pad grid build is about 5 s. The rest is untested; per-round thread and pulse-clone setup are candidates.
- **Tied objectives.** They are dropped, and which result survives depends on thread timing (bug 2 in [mc-fitter-threading.md](mc-fitter-threading.md)). A multi-threaded fit is therefore reproducible except for those results.
- **The digitization stage is not reproducible under load, and never was.** `AtPSADeconv` creates its FFT plans with FFTW's measure option (`"R2C M K"`), which picks the algorithm by timing it, so hit positions differ at the 1e-9 level between runs on a busy machine. The estimate option (`"ES"`) would make it deterministic. The fit, which reads the simulated charge, is unaffected.
- **Cheaper diffusion draws** in `AtPulseLine::generateIntegrationMap` were not built. They are estimated at about 9% of the fit CPU.
- **Peak memory** was not measured. The pad grid adds 60 MB per process.
