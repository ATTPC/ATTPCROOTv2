#include "AtSidebarPSATBAvg.h"

#include "AtPSA.h" // for AtPSA
#include "AtPSATBAvg.h"

#include <FairLogger.h>

ClassImp(AtSidebarPSATBAvg);

void AtSidebarPSATBAvg::FillFrame()
{
   AtSidebarPSA::FillFrame();

   AddIntBox(fTBtoAvg, "SetTBtoAvg()", 0, 16);
   AddIntBox(fMaxThreshold, "SetMaxThreshold()", 0, 512);

   SetIntNumber(fTBtoAvg, dynamic_cast<AtPSATBAvg *>(fPSA)->GetNumTBToAvg());
   SetIntNumber(fMaxThreshold, dynamic_cast<AtPSATBAvg *>(fPSA)->GetMaxThreshold());
}

void AtSidebarPSATBAvg::SetTBtoAvg()
{
   auto value = GetIntNumber(fTBtoAvg);
   dynamic_cast<AtPSATBAvg *>(fPSA)->SetNumTBToAvg(value);
   LOG(debug) << fTBtoAvg << " set: " << value;
}

void AtSidebarPSATBAvg::SetMaxThreshold()
{
   auto value = GetIntNumber(fMaxThreshold);
   dynamic_cast<AtPSATBAvg *>(fPSA)->SetMaxThreshold(value);
   LOG(debug) << fMaxThreshold << " set: " << value;
}
