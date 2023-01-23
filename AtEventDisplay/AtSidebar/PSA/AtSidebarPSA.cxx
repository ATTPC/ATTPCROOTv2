#include "AtSidebarPSA.h"

#include "AtPSA.h"

#include <TGNumberEntry.h> // for TGNumberEntry, TGNumberEntryField

#include <iostream> // for operator<<, endl, basic_ostream, cout
#include <map>      // for map, operator==, _Rb_tree_iterator, map<>...
#include <utility>  // for paiAr

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
