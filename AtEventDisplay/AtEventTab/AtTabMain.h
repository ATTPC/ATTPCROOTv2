#ifndef ATTABMAIN_H
#define ATTABMAIN_H

#include "AtEvent.h"    // IWYU pragma: keep
#include "AtRawEvent.h" // IWYU pragma: keep
#include "AtTabBase.h"

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
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

class AtTabMain : public AtTabBase {
protected:
   AtRawEvent *fRawEvent;
   AtEvent *fEvent;

   AtEventManagerNew *fEventManager;

   std::shared_ptr<AtMap> fDetmap;

   Int_t fThreshold;
   TString fMap;

   TEvePointSet *fHitSet;

   TPaletteAxis *fPadPlanePal;

   Color_t fHitColor;
   Size_t fHitSize;
   Style_t fHitStyle;

   TCanvas *fCvsPadPlane;
   TH2Poly *fPadPlane;
   TCanvas *fCvsPadWave;
   TH1I *fPadWave;

   Int_t fMultiHit{10};
   Bool_t fIsRawData;

   TString fDefaultEventBranch{"AtEventH"};
   TString fDefaultRawEventBranch{"AtRawEvent"};

   std::string fInfoEventName{"AtEvent"};
   std::string fInfoRawEventName{"AtRawEvent"};

   TEveRGBAPalette *fRGBAPalette;

public:
   AtTabMain();
   void InitTab() override;
   void UpdateTab() override;
   void Reset() override;
   void MakeTab() override;
   void DrawTree() override{};
   void DrawEvent() override;
   void DrawPad(Int_t PadNum) override;

   void SetMap(std::shared_ptr<AtMap> map) { fDetmap = map; }
   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(Color_t, Size_t, Style_t);
   void SetMultiHit(Int_t hitMax);

   void SetEventBranch(TString name) { fDefaultEventBranch = name; }
   void SetRawEventBranch(TString name) { fDefaultRawEventBranch = name; }

private:
   void DrawPadPlane();
   void DrawPadWave();

   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();

   // Functions for drawing hits
   void DrawHitPoints();
   void DrawWave(Int_t PadNum);

   ClassDefOverride(AtTabMain, 1)
};

#endif
