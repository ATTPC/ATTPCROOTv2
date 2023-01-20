#include "AtSidebarPSADeconv.h"

#include "AtPSADeconv.h"

ClassImp(AtSidebarPSADeconv);

void AtSidebarPSADeconv::FillFrame()
{
   AtSidebarPSA::FillFrame();

   AddIntBox(fOrder, "SetFilterOrder()", 0, 16);
   AddIntBox(fCutoff, "SetCutoffFreq()", 0, 512);

   fNumbers.find(fOrder)->second->GetNumberEntry()->SetIntNumber(dynamic_cast<AtPSADeconv *>(fPSA)->GetFilterOrder());
   fNumbers.find(fCutoff)->second->GetNumberEntry()->SetIntNumber(dynamic_cast<AtPSADeconv *>(fPSA)->GetCutoffFreq());
}

void AtSidebarPSADeconv::SetFilterOrder()
{
   if (fNumbers.find(fOrder) == fNumbers.end())
      std::cout << fOrder << " not defined!" << std::endl;
   else {
      auto value = fNumbers.find(fOrder)->second->GetNumberEntry()->GetIntNumber();
      dynamic_cast<AtPSADeconv *>(fPSA)->SetFilterOrder(value);
      std::cout << fOrder << " set: " << value << std::endl;
   }
}

void AtSidebarPSADeconv::SetCutoffFreq()
{
   if (fNumbers.find(fCutoff) == fNumbers.end())
      std::cout << fCutoff << " not defined!" << std::endl;
   else {
      auto value = fNumbers.find(fCutoff)->second->GetNumberEntry()->GetIntNumber();
      dynamic_cast<AtPSADeconv *>(fPSA)->SetCutoffFreq(value);
      std::cout << fCutoff << " set: " << value << std::endl;
   }
}