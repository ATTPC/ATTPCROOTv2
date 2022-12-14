#ifndef ATEVENTTABMAIN_H
#define ATEVENTTABMAIN_H

#include "AtEventTab.h"

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

class AtEvent;
class AtEventManagerNew;
class AtRawEvent;
class AtMap;
class TCanvas;
class TEvePointSet;
class TEveRGBAPalette;
class TH2Poly;
class TH1I;
class TPaletteAxis;

class AtEventTabMain : public AtEventTab {
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

   TEveRGBAPalette *fRGBAPalette;

public:
   AtEventTabMain();
   void Init() override;
   void Reset() override;
   void MakeTab() override;
   void SetMap(std::shared_ptr<AtMap> map) { fDetmap = map; }
   void SetThreshold(Int_t val) { fThreshold = val; }
   void SetHitAttributes(Color_t, Size_t, Style_t);
   void SetRawEventBranch(TString branchName);
   void SetEventBranch(TString branchName);
   static void SelectPad(const char *rawevt);
   void DrawEvent(AtRawEvent *rawEvent, AtEvent *event) override;
   void DrawPad(Int_t PadNum) override;
   void SetMultiHit(Int_t hitMax);

private:
   void DrawPadPlane();
   void DrawPadWave();

   void UpdateCvsPadPlane();
   void UpdateCvsPadWave();

   // Functions for drawing hits
   void DrawHitPoints();
   void DrawWave(Int_t PadNum);

   ClassDefOverride(AtEventTabMain, 1)
};

#endif
