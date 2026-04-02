# Data Visualization

ATTPCROOT includes an interactive event display built on ROOT's TEve framework. The current API is `AtViewerManager` (in `AtEventDisplay/`). 

## Architecture

`AtViewerManager` is a singleton GUI/controller. It uses `FairRunAna::Instance()` in `AddTask()` and `Init()`, so the macro must create and configure `FairRunAna` before constructing the viewer. Attach **tabs** (display panels) and optionally **tasks** (FairTask subclasses for online re-analysis). Branch names for the three standard data streams — `AtRawEvent`, `AtEvent`, and `AtPatternEvent` — are selected at runtime from dropdown menus in the GUI sidebar.

The viewer requires a pad map (`AtMap`) loaded from an experiment-specific XML mapping file.

## Basic Setup

```cpp
// Create and configure the FairRoot run before constructing the viewer
FairRunAna *fRun = new FairRunAna();
fRun->SetSource(new FairFileSource(inputFile));
fRun->SetSink(new FairRootFileSink(outputFile));
fRun->SetGeomFile(geoFile);

// Load the pad mapping
TString mapDir = TString(getenv("VMCWORKDIR")) + "/scripts/e12014_pad_mapping.xml";
auto fMap = std::make_shared<AtTpcMap>();
fMap->ParseXMLMap(mapDir.Data());

// Create viewer and add tabs
AtViewerManager *eveMan = new AtViewerManager(fMap);

auto tabMain = std::make_unique<AtTabMain>();
tabMain->SetMultiHit(100);
eveMan->AddTab(std::move(tabMain));

eveMan->Init();
```

## Adding FairTasks for Online Re-analysis

`AddTask` registers a `FairTask` that runs each time an event is loaded or re-analyzed (e.g. when PSA parameters are changed interactively):

```cpp
eveMan->AddTask(new AtPSAtask(std::move(psa)));
```

Combined with the PSA sidebar addons (`AtSidebarPSA`, `AtSidebarPSADeconv`, etc.), this allows live adjustment of PSA parameters and immediate re-display without restarting the macro.

## Available Tabs

### `AtTabMain`
The primary view. Shows a 3D hit display (TEve), a pad plane heat map, and the waveform of the currently selected pad. Selecting a pad in the pad plane updates the waveform display.

```cpp
auto tab = std::make_unique<AtTabMain>();
tab->SetThreshold(20);       // minimum charge to draw a hit
tab->SetMultiHit(100);       // max hits in a single pad before that pad is suppressed
tab->SetDrawProjection(true); // draw AtPattern projections on the pad plane
tab->SetHitAttributes(TAttMarker(kPink, kFullDotMedium, 1));
```

### `AtTabBraggCurve`
Extends `AtTabMain`. Adds a Bragg curve panel (dE/dx vs. range) alongside the 3D view, populated from `AtTrack::BraggCurve` data.

### `AtTabPad`
A configurable grid of pad trace panels. Each cell in the grid can independently display a different data source for the currently selected pad.

```cpp
// 2-row × 4-column grid
auto tab = std::make_unique<AtTabPad>(2, 4, "MyTraces");
tab->DrawADC(0, 0);                    // pedestal-subtracted ADC in cell (0,0)
tab->DrawRawADC(0, 1);                 // raw ADC
tab->DrawArrayAug("Qreco", 0, 2);      // named array augment (e.g. deconvolved charge)
tab->DrawAuxADC("MeshSignal", 0, 3);   // auxiliary pad by name
tab->DrawFPN(1, 0);                    // fixed pattern noise trace
tab->DrawGenTrace(3, 1, 1);            // generic trace by ID
tab->DrawHits(0, 0);                   // overlay hit markers on the ADC trace
eveMan->AddTab(std::move(tab));
```

### `AtTabMacro`
A canvas-based tab driven entirely by user-supplied lambda functions. Callbacks fire at three granularities:

```cpp
auto tab = std::make_unique<AtTabMacro>(1, 2, "Custom");

// Called once when the tree is opened
tab->SetDrawTreeFunction([](AtTabInfo *info) { /* fill histograms */ }, 0, 0);

// Called each time an event is loaded
tab->SetDrawEventFunction([](AtTabInfo *info) { /* update per-event plots */ }, 0, 1);

// Called each time the selected pad changes
tab->SetDrawPadFunction([](AtTabInfo *info, Int_t padNum) { /* pad-level plots */ }, 0, 0);

eveMan->AddTab(std::move(tab));
```

### `AtTabFission`
Extends `AtTabMain`. Adds hit overlays for fission fragment analysis (uncorrected and corrected hit sets). Exposes `GetFissionBranch()` for branch selection.

### `AtTabEnergyLoss`
Canvas-based tab that calculates dE/dx for two fission fragments from an `AtFissionEvent`. Stacked histograms, charge-sum and Gaussian-fit methods, and the ratio between the two fragments.

```cpp
AtTabFission *fissionTab = ...; // add fission tab first
auto elTab = std::make_unique<AtTabEnergyLoss>(fissionTab->GetFissionBranch());
eveMan->AddTab(std::move(elTab));
```

## Notes

- **Branch selection is in the GUI**, not in the macro. The sidebar dropdowns populate from whatever branches are present in the input file.
- `AtViewerManager::Instance()` returns the singleton after construction.
- Requires a display (X11 or native windowing); cannot run headlessly.
- Pad mapping XML files live in `scripts/`. The correct file is experiment-specific.
