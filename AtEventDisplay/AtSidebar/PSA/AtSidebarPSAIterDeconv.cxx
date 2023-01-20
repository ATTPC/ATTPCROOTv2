#include "AtSidebarPSAIterDeconv.h"

#include "AtPSAIterDeconv.h"

ClassImp(AtSidebarPSAIterDeconv);

void AtSidebarPSAIterDeconv::FillFrame()
{
   AtSidebarPSADeconv::FillFrame();

   AddIntBox(fIterations, "SetIterations()", 0, 10);

   fNumbers.find(fIterations)
      ->second->GetNumberEntry()
      ->SetIntNumber(dynamic_cast<AtPSAIterDeconv *>(fPSA)->GetIterations());
}

void AtSidebarPSAIterDeconv::SetIterations()
{
   if (fNumbers.find(fIterations) == fNumbers.end())
      std::cout << fIterations << " not defined!" << std::endl;
   else {
      auto value = fNumbers.find(fIterations)->second->GetNumberEntry()->GetIntNumber();
      dynamic_cast<AtPSAIterDeconv *>(fPSA)->SetIterations(value);
      std::cout << fIterations << " set: " << value << std::endl;
   }
}
