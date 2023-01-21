#include "AtSidebarPSAIterDeconv.h"

#include "AtPSA.h"           // for AtPSA
#include "AtPSAIterDeconv.h" // for AtPSAIterDeconv

#include <FairLogger.h>

#include <Rtypes.h> // for TGenericClassInfo

ClassImp(AtSidebarPSAIterDeconv);

void AtSidebarPSAIterDeconv::FillFrame()
{
   AtSidebarPSADeconv::FillFrame();

   AddIntBox(fIterations, "SetIterations()", 0, 10);

   SetIntNumber(fIterations, dynamic_cast<AtPSAIterDeconv *>(fPSA)->GetIterations());
}

void AtSidebarPSAIterDeconv::SetIterations()
{
   auto value = GetIntNumber(fIterations);
   dynamic_cast<AtPSAIterDeconv *>(fPSA)->SetIterations(value);
   LOG(debug) << fIterations << " set: " << value;
}
