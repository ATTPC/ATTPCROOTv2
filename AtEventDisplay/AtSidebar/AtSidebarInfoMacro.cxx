#include "AtSidebarInfoMacro.h"

#include <FairLogger.h> // for endl, basic_ostream, cout, ostream
namespace DataHandling {
class AtSubject;
}

ClassImp(AtSidebarInfoMacro);

void AtSidebarInfoMacro::FillFrame()
{
   AddInfoBox(fLabel);
}

void AtSidebarInfoMacro::Update(DataHandling::AtSubject *changedSubject)
{
   if (changedSubject == &fEntryNumber) {
      LOG(info) << fFunction();
      SetInfoString(fLabel, fFunction());
   }
}
