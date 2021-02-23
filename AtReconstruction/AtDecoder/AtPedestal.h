// =================================================
//  AtPedestal Class
//
//  Description:
//    This class is used for calculating pedestal
//    values of each channel using FPN channels.
//
//  Author :Genie Jhang ( geniejhang@majimak.com )
//  Modified by Y. Ayyad for the AtTPCROOT
//
// =================================================

#ifndef AtPEDESTAL
#define AtPEDESTAL

#include "TObject.h"

#include "GETMath2.h"

#include <fstream>

class AtPedestal : public TObject {
public:
   AtPedestal();

   Bool_t SubtractPedestal(Int_t numTbs, Int_t *fpn, Int_t *rawADC, Double_t *dest, Double_t rmsCut = 5,
                           Bool_t signalNegativePolarity = kFALSE, Int_t startTb = 3, Int_t averageTbs = 10);

private:
   GETMath2 *fMath;

   ClassDef(AtPedestal, 1);
};

#endif
