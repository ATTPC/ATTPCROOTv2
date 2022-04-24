/*********************************************************************
 *   AtTPC Pad Class	AtPad                                         *
 *   Author: Y. Ayyad                                                 *
 *   Log: 05-03-2015 19:24 JST                                        *
 *                                                                    *
 *                                                                    *
 *********************************************************************/

#include "AtPad.h"

#include <FairLogger.h>

#include <TH1.h>

#include <memory>
#include <string>

ClassImp(AtPad);

AtPad::AtPad(Int_t PadNum) : fPadNum(PadNum) {}

std::unique_ptr<AtPad> AtPad::Clone()
{
   return std::make_unique<AtPad>(*this);
}

const AtPad::trace &AtPad::GetADC() const
{
   if (!fIsPedestalSubtracted)
      LOG(fatal) << "Pedestal subtraction was not done on pad " << fPadNum;

   return fAdc;
}

Double_t AtPad::GetADC(Int_t idx) const
{
   return GetADC()[idx];
}

std::unique_ptr<TH1D> AtPad::GetADCHistrogram() const
{
   auto histName = "adc" + std::to_string(GetPadNum());
   auto histTitle = "ADC " + std::to_string(GetPadNum());
   auto hist = std::make_unique<TH1D>(histName.data(), histTitle.data(), fAdc.size(), 0, fAdc.size() - 1);
   hist->SetDirectory(nullptr); // Pass ownership to the pointer instead of current ROOT directory
   for (int i = 0; i < fAdc.size(); ++i)
      hist->SetBinContent(i + 1, fAdc[i]);
   return hist;
}
