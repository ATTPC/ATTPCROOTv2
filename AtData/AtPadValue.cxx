#include "AtPadValue.h"

AtPadValue::AtPadValue(Double_t val) : AtPadBase(), fValue(val) {}

std::unique_ptr<AtPadBase> AtPadValue::Clone() const
{
   return std::make_unique<AtPadValue>(*this);
}

ClassImp(AtPadValue);
