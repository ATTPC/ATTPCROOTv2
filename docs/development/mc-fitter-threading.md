# MC Fitter Threading

How `MCFitter::AtMCFitter` parallelizes, the bugs found in a read-through, and where its time goes. Measurements are in [fit-speed-study.md](fit-speed-study.md). Code is referred to by function name; bugs that can be checked in output have a check.

The configuration studied is the e12014 fission fit: `AtMCFission` with `AtClusterizeLine`, `AtPulseLine`, `AtPSADeconvFit` with `SetUseSimCharge(true)`, two rounds (`SetNumRounds(2)`), space charge off.

## Parallel Model

`AtMCFitter` (`AtReconstruction/AtFitter/AtMCFitter.cxx`) is the only threaded code in the fitter, simulation or digitization path. Events are processed serially by `AtMCFitterTask`. Within one event, `Exec`:

1. Clears `fResults` and resizes `fRawEventArray`/`fEventArray` to `fNumIter`.
2. Runs `fNumRounds` rounds. Each round (`RunRound`) splits `fNumIter` iterations into contiguous blocks, spawns `fNumThreads` new `std::thread`s (even when `fNumThreads == 1`), and joins them. Each thread installs its own random generator for the round (see the table below).
3. After each round, including the last, `RecenterParamDistributions` moves every parameter distribution's mean to the best result so far and narrows it (`AtUniformDistribution` ×0.8, `AtStudentDistribution` ×0.5).
4. `FillResultArrays`, called by the task, copies results into `TClonesArray`s in objective order and saves the first `fNumEventsToSave` simulated events.

One iteration (`RunIterRange`) samples parameters, simulates (`AtMCFission::SimulateEvent`), clusterizes, pulses, runs PSA, and computes the objective (a Minuit2 fit inside `AtMCFission`). The result is inserted into `fResults` under `fResultMutex`. Digitized events are written to `fRawEventArray[idx]`/`fEventArray[idx]` without a lock, which is safe within a round because each thread owns its index range.

### Shared vs. per-thread state

| Object | Per thread? | Notes |
|---|---|---|
| `AtPulse` / `AtPulseLine` | Yes | Cloned into `fThPulse` in `Init`. Each clone builds a `TH1F` per pad. The response function is shared read-only. |
| `AtMap` | Shared | One map for the whole fit. Its pad lookup grid (`AtPadLookup`, 60 MB for the AT-TPC plane) is built once under `std::call_once` on the first lookup and is read-only afterwards. |
| `AtSimpleSimulation` | Shared | Point array and track ID are `static thread_local`. Geometry lookups (`GetVolume`) go through `fGeoMutex`. |
| Space-charge model | Shared, **written per iteration** | See bug 3. |
| `AtClusterize` / `AtClusterizeLine` | Shared | `thread_local` statics. |
| PSA | Shared | `AtPSADeconvFit` uses a `thread_local` histogram. Its fits build a `Minuit2Minimizer` directly (`AtTools::MinimizeMigrad`), not through `ROOT::Fit::Fitter`, whose minimizer creation takes ROOT's global lock. The FFT objects and response cache in `AtPSADeconv` are shared and unprotected; with `SetUseSimCharge(true)` they are never used, and a pad without a `"Q"` augment is a fatal error (the pulse needs `SetSaveCharge(true)`). |
| Hit functions | — | `AtE12014` and `AtPSADeconvFit` evaluate hits with `AtTools::GaussianTB`, a plain struct; there are no per-thread `TF1`s. |
| Parameter distributions | Shared | Mean and spread are read-only during a round. |
| Random generator | Yes | Each thread of each round installs a `TRandom3` (`AtTools::Random::ScopedGenerator`) seeded from the fitter seed (`SetSeed`, else drawn in `Init`), the event's entry number, the round and the thread index. Parameter sampling, clusterize and pulse draw from it through `AtTools::Random::Get()`, which is `gRandom` outside the fitter. With the static split, a fit is reproducible for a given seed, `fNumThreads` and `fNumIter` (but see bug 2). |
| `AtDigiPar` | Cached in `fPar` | `FairRun::Instance()` is thread-local. |

Knobs: `SetSeed`, `SetNumThreads`, `SetNumIter`, `SetNumRounds`, `SetNumEventsToSave`, `SetTimeEvent`. `SetNumThreads` calls `ROOT::EnableThreadSafety()` when > 1 and must be called before `Init()`, which sizes `fThPulse`; raising it afterwards indexes past the end. `fNumThreads <= 0` divides by zero in `RunRound`.

## Where the time goes

Measured on the cluster (see [fit-speed-study.md](fit-speed-study.md)):

- Before the speed study, 94% of an iteration was in `AtPulseLine::GenerateEvent`. The geometry lock, the static split, thread creation per round, logging and result copies were each at most 2%. Per-thread `TGeoNavigator`s in place of the geometry lock crashed in a test, so that lock stays.
- `ROOT::Fit::Fitter` created each PSA pad fit's minimizer under ROOT's global lock, which made more than 16 threads slower than 16. The PSA and `AtMCFission` now build Minuit2 directly, and wall time per event falls up to 50–64 threads.
- What remains at high thread counts: the static split (at least 22% idle at 32 and 64 threads with `fNumIter = 100`, none when the thread count divides `fNumIter`), a per-thread CPU inflation of 1.3–1.5x of unknown cause, a slow first event per job, and about 0.1–0.5 s per event of main-thread work between rounds. The study doc's "What remains at high thread counts" lists the experiments.

For throughput, run many processes with few threads (two per allocated core) rather than one process with many threads.

## Bugs

Ordered by impact on current runs.

### Live, affects saved events or counts

1. **Saved events are paired with the wrong result after round 1.** Live whenever `SetNumRounds` is more than 1.
   - `fResults` is cleared once per event, but every round reuses indices `0..fNumIter-1` in `RunIterRange`, so `fRawEventArray[idx]` and `fEventArray[idx]` are overwritten.
   - A round-1 result with `fIterNum = 17` ends up pointing at round 2's event 17, and `FillResultArrays` saves that event next to it.
   - If two saved results share an `fIterNum` (one per round), `FillResultArrays` moves from the same slot twice, so the second gets an empty event.
   - Parameters and objectives are correct, so histograms built from `AtMCResult` are unaffected. Anything that inspects the saved best-fit `AtEvent`/`AtRawEvent` may be looking at the wrong simulation.
   - Check: in a fit output with two rounds, look for repeated `fIterNum` among the saved results of one event, or for saved events with no hits.
   - Fix: index by `round * fNumIter + idx`, or store the event with the result.
2. **Tied objectives are dropped.**
   - `fResults` is a `std::set` ordered by `fObjective` (comparator in `AtMCFitter`'s constructor), so results with equal objectives count as duplicates.
   - `AtMCFission::ObjectiveCharge` returns `std::numeric_limits<double>::max()` when the charge fit fails and when the experimental and simulated charge do not overlap. `ObjectiveChargePads` does too, but its call is commented out. All such results but one vanish.
   - Result counts per event are therefore not `iter × rounds`, and failure rates can't be measured.
   - With more than one thread, which tied result survives depends on thread timing, so such an event's results can differ between reruns with the same seed.
   - Check: count `AtMCResult` entries per event and compare with `iter × rounds`.
   - Fix: `std::multiset`.

### Dormant or latent

3. **The shared space-charge model is mutated per iteration.**
   - `AtMCFission::SimulateEvent` (existing TODO) calls `SetDistortionField` or `SetLambda` on the simulation's single model while other threads apply it.
   - `SetDistortionField` reassigns a `std::function` (`AtRadialChargeModel`) that other threads may be calling, which is undefined behaviour and can crash, not just use a stale lambda.
   - Dormant only because current macros leave space charge off; it becomes a race as soon as it is enabled.
4. **Parameter spreads shrink from event to event.** `TruncateSpace` is never undone, and `AtMCFission::SetParamDistributions` resets only `Z`'s spread. Today this affects only `thBeam`, which is sampled but unused (`GetBeamDirSample` is never called), and the uniform spreads, which are 0. It will matter as soon as a macro sets nonzero spreads.
5. **Smaller defects.**
   - `AtPulse::ApplyNoise` multiplies the signal by `Gaus(0, fNoiseSigma)` instead of adding noise; with nonzero noise the signal is scrambled.
   - `AtClusterize::getNumberOfElectronsGenerated` returns a `Gaus` draw as `uint64_t`; a negative draw is undefined.
   - `RecenterParamDistributions` dereferences `fResults.begin()` on an empty set when `fNumIter == 0`.

Until bug 3 is fixed, enable space charge only with `SetNumThreads(1)`.
