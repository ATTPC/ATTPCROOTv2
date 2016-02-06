// =================================================
//  ATPedestal Class
//
//  Description:
//    This class is used for calculating pedestal
//    values of each channel using FPN channels.
//
//  Author :Genie Jhang ( geniejhang@majimak.com )
//  Modified by Y. Ayyad for the ATTPCROOT
//
// =================================================

#ifndef ATPEDESTAL
#define ATPEDESTAL

#include "TObject.h"

#include "GETMath2.hh"

#include <fstream>

class ATPedestal : public TObject {
  public:
    ATPedestal();

    Bool_t SubtractPedestal(Int_t numTbs, Int_t *fpn, Int_t *rawADC, Double_t *dest, Double_t rmsCut = 5, Bool_t signalNegativePolarity = kFALSE, Int_t startTb = 3, Int_t averageTbs = 10);

  private:
    GETMath2 *fMath;

  ClassDef(ATPedestal, 1);
};

#endif
