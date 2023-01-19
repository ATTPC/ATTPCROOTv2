#ifndef ATTABMAIN_H
#define ATTABMAIN_H

#include "AtEvent.h"        // IWYU pragma: keep
#include "AtPatternEvent.h" // IWYU pragma: keep
#include "AtRawEvent.h"     // IWYU pragma: keep
#include "AtTabBase.h"

#include <Rtypes.h> // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TEvePointSet.h>
#include <TString.h> // for TString

#include <memory> // for shared_ptr
#include <set>
#include <string> // for string

class TBuffer;
class TClass;
class TMemberInspector;
class AtEventManagerNew;
class AtMap;
class TCanvas;
class TEvePointSet;
class TEveRGBAPalette;
class TH2Poly;
class TH1I;
class TPaletteAxis;
class AtTabInfo;
class AtPatternEvent;
class AtRawEvent;
class AtEvent;

class AtTabMain : public AtTabBase {
private:
   using TEvePointSetPtr = std::unique_ptr<TEvePointSet>;

protected:
   // Information for drawing 3D events
   Int_t fThreshold{0};    //< Min charge to draw hit
   Int_t fMaxHitMulti{10}; //< Max hits in a pad for hit to be drawn

   TAttMarker fHitAttr{kPink, 1, kFullDotMedium};

   TEvePointSetPtr fHitSet{std::make_unique<TEvePointSet>("Hit")}; //< AtEvent Hit Set
   std::vector<TEvePointSetPtr> fPatternHitSets;

   TCanvas *fCvsPadPlane{nullptr};
   TH2Poly *fPadPlane{nullptr};
   TCanvas *fCvsPadWave{nullptr};
   TH1I *fPadWave{nullptr};

public:
   AtTabMain() = default;
   void InitTab() override;
   void UpdateTab() override {}
   void Reset() override;
   void MakeTab() override;

   void DrawEvent() override;
   void DrawPad(Int_t PadNum) override;
   void DumpEvent(std::string file);

   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(TAttMarker attr) { fHitAttr = std::move(attr); }
   void SetMultiHit(Int_t hitMax) { fMaxHitMulti = hitMax; }

private:
   // Functions to draw the initial canvases
   void DrawPadPlane();
   void DrawPadWave();

   // Functions to update the data on the canvases
   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();

   // Functions for drawing hits
   void DrawHitPoints();

   void DrawPatternHitPoints();
   bool DrawWave(Int_t PadNum);
   // std::unique_ptr<TEvePointSet> GetPointsFromHits(const std::vector<std::unique_ptr<AtHit>> &hits);
   void SetPointsFromHits(TEvePointSet &hitSet, const std::vector<std::unique_ptr<AtHit>> &hits);
   void FillPadPlane(const std::vector<std::unique_ptr<AtHit>> &hits);

private:
   void AddPatternHitSet();

   ClassDefOverride(AtTabMain, 1)
};

#endif
