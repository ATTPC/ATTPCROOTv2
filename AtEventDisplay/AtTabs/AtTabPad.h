#ifndef ATTABPAD_H
#define ATTABPAD_H

#include "AtTabBase.h"

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <memory> // for shared_ptr
#include <string> // for string
#include <unordered_map>
#include <utility> // for pair

class TBuffer;
class TClass;
class TMemberInspector;
class AtEvent;
class AtEventManagerNew;
class AtRawEvent;
class AtMap;
class AtPad;
class TCanvas;
class TEveRGBAPalette;
class TH1D;

/**
 * Class for drawing traces from pads in an AtRawEvent.
 *
 */
class AtTabPad : public AtTabBase {
protected:
   enum class PadDrawType { kADC, kRawADC, kArrAug };
   TCanvas *fCvsPad{nullptr};

   Int_t fRows;
   Int_t fCols;

   /// <location, <type, histo>
   /// location is row * nCols + col
   std::unordered_map<Int_t, std::pair<PadDrawType, TH1D *>> fDrawMap; //! Let root handle hist memory
   std::unordered_map<Int_t, TString> fAugNames;

   TString fTabName;

   TEveRGBAPalette *fRGBAPalette{nullptr};

public:
   AtTabPad(int nRow = 1, int nCol = 1, TString name = "AtPad");
   void InitTab() override;
   void Exec() override {}

   void DrawADC(int row = 0, int col = 0);                       //< Draw adc in current pad
   void DrawRawADC(int row = 0, int col = 0);                    //< Draw raw adc in current pad
   void DrawArrayAug(TString augName, int row = 0, int col = 0); //< Draw an array augment current pad

   void SetTabName(TString tabName) { fTabName = tabName; }

   void DrawPad(Int_t padNum) override;

protected:
   void MakeTab(TEveWindowSlot *) override;

private:
   void SetDraw(Int_t pos, PadDrawType type);
   void DrawAdc(TH1D *hist, const AtPad &pad);
   void DrawRawAdc(TH1D *hist, const AtPad &pad);
   void DrawArrayAug(TH1D *hist, const AtPad &pad, TString augName);

   void UpdateCvsPad();

   // Functions for drawing hits

   ClassDefOverride(AtTabPad, 1)
};

#endif
