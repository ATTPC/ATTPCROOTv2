#ifndef ATTAB3DBRAGGCURVE_H
#define ATTAB3DBRAGGCURVE_H

#include "At3DBraggFitter.h"
#include "AtDataObserver.h"         // for AtObserver
#include "AtTabBase.h"              // for AtTabBase
#include "AtViewerManagerSubject.h" // for ?????????????????

#include <Rtypes.h>           // for Int_t
#include <TAttMarker.h>       // for TAttMarker, kFullDotMedium
#include <TEveElement.h>      // for TEveElement
#include <TEveEventManager.h> // for TEveEventManager
#include <TEvePointSet.h>     // for TEvePointSet
#include <TGraph.h>
#include <TMultiGraph.h>

#include <memory>  // for make_unique, unique_ptr
#include <utility> // for move
#include <vector>  // for vector

class AtHit;
class AtTrack;
class TEveWindowSlot;
class TCanvas;
class TH1F;
namespace DataHandling {
class AtSubject;
}

/*
   Tab for 3D visualizing of events together with the 3DBragg curves of straight tracks.
*/
class AtTab3DBraggCurve : public AtTabBase, public DataHandling::AtObserver {
protected:
   using TEvePointSetPtr = std::unique_ptr<TEvePointSet>;
   using TEveEventManagerPtr = std::unique_ptr<TEveEventManager>;

   TEveEventManagerPtr fEveEvent{std::make_unique<TEveEventManager>("AtEvent")};
   TEvePointSetPtr fHitSet{std::make_unique<TEvePointSet>("Hits")}; //< AtEvent Hit Set

   TEveEventManagerPtr fEvePatternEvent{std::make_unique<TEveEventManager>("AtPatternEvent")};
   TEvePointSetPtr fNoiseHitSet{std::make_unique<TEvePointSet>("Noise")}; //< AtPatternEvent Noise Set
   std::vector<TEvePointSetPtr> fPatternHitSets;
   std::vector<TEveElement> fPatterns;

   Int_t fThreshold{0};    //< Min charge to draw hit
   Int_t fMaxHitMulti{10}; //< Max hits in a pad for hit to be drawn

   TAttMarker fHitAttr{kPink, kFullDotMedium, 1};

   Int_t fTrackIdx{-1};
   TCanvas *fCvsDeDx{nullptr};
   TMultiGraph *fDeDxGraphs{nullptr};

   TCanvas *fCvsQvS{nullptr};
   TH1F *fHistQvS{nullptr};
   TGraph *fFittedBraggCurveGraph{nullptr};
   Double_t fDZ{1};
   Double_t fDS{1};

   std::unique_ptr<AtFITTER::At3DBraggFitter> fFitter{nullptr};

   DataHandling::AtBranch *fEventBranch;
   DataHandling::AtBranch *fPatternEventBranch;
   DataHandling::AtTreeEntry *fEntry;

public:
   AtTab3DBraggCurve();
   ~AtTab3DBraggCurve();
   void InitTab() override;

   void Exec() override{};
   void Update(DataHandling::AtSubject *sub) override;

   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(TAttMarker attr) { fHitAttr = std::move(attr); }
   void SetMultiHit(Int_t hitMax) { fMaxHitMulti = hitMax; }

   void SetDZ(Double_t DZ) { fDZ = DZ; }

   void Set3DBraggCurveFitter(std::unique_ptr<AtFITTER::At3DBraggFitter> fitter) { fFitter = std::move(fitter); }

protected:
   void MakeTab(TEveWindowSlot *slot) override;

   // Sets the default render state for TEveEventManagers
   virtual void UpdateRenderState();
   Color_t GetTrackColor(int i);
   void SetPointsFromHits(TEvePointSet &hitSet, const std::vector<std::unique_ptr<AtHit>> &hits);
   void SetPointsFromHits(TEvePointSet &hitSet, const std::vector<AtHit *> &hits);
   void SetPointsFromTrack(TEvePointSet &hitSet, const AtTrack &track);

private:
   // Functions to draw the initial canvases
   void DrawDeDxGraphs();
   void DrawHistQvS();

   // This function draws the 3D Bragg curve of a given track
   void Draw3DBraggCurve(AtTrack &track);

   // Update hit sets
   void UpdateEventElements();
   void UpdatePatternEventElements();

   void ExpandNumPatterns(int num);

   ClassDefOverride(AtTab3DBraggCurve, 1)
};

#endif
