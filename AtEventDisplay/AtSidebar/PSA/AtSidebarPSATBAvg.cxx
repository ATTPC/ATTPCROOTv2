#include "AtSidebarPSATBAvg.h"

#include "AtPSA.h" // for AtPSA
#include "AtPSATBAvg.h"

#include <FairLogger.h>

ClassImp(AtSidebarPSATBAvg);

void AtSidebarPSATBAvg::FillFrame()
{
   auto tPSA = dynamic_cast<AtPSATBAvg *>(fPSA);
   if (tPSA == nullptr)
      return;

   AtSidebarPSA::FillFrame();

   AddIntBox(fTBtoAvg, "SetTBtoAvg()", 0, 16);
   AddIntBox(fMaxThreshold, "SetMaxThreshold()", 0, 512);

   SetIntNumber(fTBtoAvg, tPSA->GetNumTBToAvg());
   SetIntNumber(fMaxThreshold, tPSA->GetMaxThreshold());
}

void AtSidebarPSATBAvg::SetTBtoAvg()
{
   auto tPSA = dynamic_cast<AtPSATBAvg *>(fPSA);
   if (tPSA == nullptr)
      return;

   auto value = GetIntNumber(fTBtoAvg);
   tPSA->SetNumTBToAvg(value);
   LOG(debug) << fTBtoAvg << " set: " << value;
}

void AtSidebarPSATBAvg::SetMaxThreshold()
{
   auto tPSA = dynamic_cast<AtPSATBAvg *>(fPSA);
   if (tPSA == nullptr)
      return;

   auto value = GetIntNumber(fMaxThreshold);
   tPSA->SetMaxThreshold(value);
   LOG(debug) << fMaxThreshold << " set: " << value;
}
