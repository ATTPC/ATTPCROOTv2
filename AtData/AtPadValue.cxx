#include "AtPadValue.h"

std::unique_ptr<AtPadBase> AtPadValue::Clone() const
{
   return std::make_unique<AtPadValue>(*this);
}

ClassImp(AtPadValue);
