#include "AtPadCharge.h"

std::unique_ptr<AtPad> AtPadCharge::Clone()
{
   return std::make_unique<AtPadCharge>(*this);
}

const AtPadCharge::traceElec &AtPadCharge::GetElectrons() const
{
   return fElectrons;
}

Double_t AtPadCharge::GetElectrons(Int_t idx) const
{
   return GetElectrons()[idx];
}

ClassImp(AtPadCharge);
