#include "AtPadCharge.h"

std::unique_ptr<AtPad> AtPadCharge::Clone()
{
   return std::make_unique<AtPadCharge>(*this);
}

ClassImp(AtPadCharge);
