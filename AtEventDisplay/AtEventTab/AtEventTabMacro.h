#ifndef ATEVENTTABMACRO_H
#define ATEVENTTABMACRO_H

#include "AtEventTab.h"

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <vector>
#include <unordered_map>

class AtEvent;
class AtEventManagerNew;
class AtRawEvent;
class AtMap;
class AtPad;
class TCanvas;
class TEveRGBAPalette;
class TH1D;
class TPaletteAxis;

class AtEventTabMacro : public AtEventTab {
protected:
   AtRawEvent *fRawEvent;
   AtEvent *fEvent;

   AtEventManagerNew *fEventManager;

   std::shared_ptr<AtMap> fDetmap;

   TString fMap;

   TCanvas *fCvsMacro;

   Int_t fRows;
   Int_t fCols;

   std::unordered_map<Int_t, std::function<void(AtRawEvent (*), AtEvent(*))> > fDrawEventMap;
   std::unordered_map<Int_t, std::function<void(AtRawEvent (*), AtEvent (*), Int_t)> > fDrawPadMap;

   TString fTabName;

   TEveRGBAPalette *fRGBAPalette;

public:
   AtEventTabMacro();
   void Init() override;
   void Reset() override;
   void MakeTab() override;
   void SetDrawEventFunction(Int_t pos, std::function<void(AtRawEvent (*), AtEvent (*))> function);
   void SetDrawPadFunction(Int_t pos, std::function<void(AtRawEvent (*), AtEvent (*), Int_t)> function);
   void SetMap(std::shared_ptr<AtMap> map) { fDetmap = map; }
   void SetColumns(Int_t cols) { fCols = cols; }
   void SetRows(Int_t rows) {fRows = rows;}
   void SetTabName(TString tabName) { fTabName = tabName; }
   void DrawEvent(AtRawEvent *rawEvent, AtEvent *event) override;
   void DrawPad(Int_t padNum) override;

private:
   void UpdateCvsMacro();

   // Functions for drawing hits

   ClassDefOverride(AtEventTabMacro, 1)
};

#endif
