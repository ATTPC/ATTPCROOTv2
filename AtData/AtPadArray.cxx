#include "AtPadArray.h"

std::unique_ptr<AtPadBase> AtPadArray::Clone() const
{
   return std::make_unique<AtPadArray>(*this);
}

ClassImp(AtPadArray);
