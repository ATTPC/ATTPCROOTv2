#include "AtSidebarInfoMacro.h"

ClassImp(AtSidebarInfoMacro);

void AtSidebarInfoMacro::FillFrame()
{
   AddInfoBox(fLabel);
}

void AtSidebarInfoMacro::Update(DataHandling::AtSubject *changedSubject)
{
   if (changedSubject == &fEntryNumber) {
      std::cout << fFunction() << std::endl;
      SetInfoString(fLabel, fFunction());
   }
}