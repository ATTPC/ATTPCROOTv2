#include "AtSidebarPSADeconv.h"

#include "AtPSA.h" // for AtPSA
#include "AtPSADeconv.h"

#include <FairLogger.h>

ClassImp(AtSidebarPSADeconv);

void AtSidebarPSADeconv::FillFrame()
{
   AtSidebarPSA::FillFrame();

   AddIntBox(fOrder, "SetFilterOrder()", 0, 16);
   AddIntBox(fCutoff, "SetCutoffFreq()", 0, 512);

   SetIntNumber(fOrder, dynamic_cast<AtPSADeconv *>(fPSA)->GetFilterOrder());
   SetIntNumber(fCutoff, dynamic_cast<AtPSADeconv *>(fPSA)->GetCutoffFreq());
}

void AtSidebarPSADeconv::SetFilterOrder()
{
   auto value = GetIntNumber(fOrder);
   dynamic_cast<AtPSADeconv *>(fPSA)->SetFilterOrder(value);
   LOG(debug) << fOrder << " set: " << value;
}

void AtSidebarPSADeconv::SetCutoffFreq()
{
   auto value = GetIntNumber(fCutoff);
   dynamic_cast<AtPSADeconv *>(fPSA)->SetCutoffFreq(value);
   LOG(debug) << fCutoff << " set: " << value;
}
