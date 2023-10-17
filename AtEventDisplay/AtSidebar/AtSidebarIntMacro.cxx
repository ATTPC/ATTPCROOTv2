#include "AtSidebarIntMacro.h"

#include <FairLogger.h> // for endl, basic_ostream, cout, ostream

ClassImp(AtSidebarIntMacro);

void AtSidebarIntMacro::FillFrame()
{
   AddIntBox(fLabel, "RunFunction()", fMin, fMax);

   SetIntNumber(fLabel, fStart);
}

void AtSidebarIntMacro::RunFunction()
{
   auto value = GetIntNumber(fLabel);
   fFunction(value);
}
