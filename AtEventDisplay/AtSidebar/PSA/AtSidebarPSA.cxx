#include "AtSidebarPSA.h"

#include "AtPSA.h"

ClassImp(AtSidebarPSA);

void AtSidebarPSA::FillFrame()
{
   AddIntBox(fThreshold, "SetThreshold()", 0, 4000);

   fNumbers.find(fThreshold)->second->GetNumberEntry()->SetIntNumber(fPSA->GetThreshold());
}

void AtSidebarPSA::SetThreshold()
{
   if (fNumbers.find(fThreshold) == fNumbers.end())
      std::cout << "Threshold not defined!" << std::endl;
   else {
      auto value = fNumbers.find(fThreshold)->second->GetNumberEntry()->GetIntNumber();
      fPSA->SetThreshold(value);
      std::cout << "Threshold set: " << value << std::endl;
   }
}