#include "AtSidebarIntMacro.h"

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
