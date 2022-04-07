/*********************************************************************
 *   AtTPC Pad Class	AtPad                                         *
 *   Author: Y. Ayyad                                                 *
 *   Log: 05-03-2015 19:24 JST                                        *
 *                                                                    *
 *                                                                    *
 *********************************************************************/

#include <FairLogger.h>
#include <memory>

#include "AtPad.h"

ClassImp(AtPad);

AtPad::AtPad(Int_t PadNum) : fPadNum(PadNum)
{
   // fIsAux = kFALSE;
   // fAuxName = "noname";
}

const trace &AtPad::GetADC() const
{
   if (!fIsPedestalSubtracted)
      LOG(fatal) << "Pedestal subtraction was not done on pad " << fPadNum;

   return fAdc;
}

Double_t AtPad::GetADC(Int_t idx) const
{
   return GetADC()[idx];
}
