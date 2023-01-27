#ifndef ATTABPAD_H
#define ATTABPAD_H

#include "AtDataObserver.h"
#include "AtTabCanvas.h"
#include "AtViewerManagerSubject.h" // for AtPadNum

#include <Rtypes.h>  // for Int_t, Bool_t, THashConsistencyHolder, Color_t
#include <TString.h> // for TString

#include <string> // for string
#include <unordered_map>
#include <utility> // for pair

class TEveWindowSlot;
class TBuffer;
class TClass;
class TMemberInspector;
class AtPad;
class TH1D;
namespace DataHandling {
class AtSubject;
}

/**
 * Class for drawing traces from pads in an AtRawEvent.
 *
 */
class AtTabPad : public AtTabCanvas, public DataHandling::AtObserver {
protected:
   enum class PadDrawType { kADC, kRawADC, kArrAug, kAuxPad };

   /// <location, <type, histo>
   /// location is row * nCols + col
   std::unordered_map<Int_t, std::pair<PadDrawType, TH1D *>> fDrawMap; //! Let root handle hist memory
   std::unordered_map<Int_t, std::string> fAugNames;                   //< Augment and Aux pad names
   DataHandling::AtPadNum *fPadNum;

public:
   AtTabPad(int nRow = 1, int nCol = 1, TString name = "AtPad");
   ~AtTabPad();
   void InitTab() override;
   void Exec() override;
   void Update(DataHandling::AtSubject *sub) override;

   void DrawADC(int row = 0, int col = 0);                       //< Draw adc in current pad
   void DrawRawADC(int row = 0, int col = 0);                    //< Draw raw adc in current pad
   void DrawArrayAug(TString augName, int row = 0, int col = 0); //< Draw an array augment current pad
   void DrawAuxADC(TString auxName, int row = 0, int col = 0);   //< Draw an aux pad

protected:
   void MakeTab(TEveWindowSlot *) override;

private:
   void SetDraw(Int_t pos, PadDrawType type);
   void DrawPad();
   void DrawAdc(TH1D *hist, const AtPad &pad);
   void DrawRawAdc(TH1D *hist, const AtPad &pad);
   void DrawArrayAug(TH1D *hist, const AtPad &pad, TString augName);

   void UpdateCvsPad();

   // Functions for drawing hits

   ClassDefOverride(AtTabPad, 1)
};

#endif
