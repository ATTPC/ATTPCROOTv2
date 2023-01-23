#include "AtSidebarPSAIterDeconv.h"

#include "AtPSA.h"           // for AtPSA
#include "AtPSAIterDeconv.h" // for AtPSAIterDeconv

#include <TGNumberEntry.h> // for TGNumberEntry, TGNumberEntryField

#include "Rtypes.h" // for TGenericClassInfo

#include <iostream> // for operator<<, basic_ostream, endl, cout
#include <map>      // for map, operator==, _Rb_tree_iterator, map...
#include <utility>  // for pair

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
