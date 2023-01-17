#include "AtTabBase.h"

#include "AtTabInfo.h"

ClassImp(AtTabBase);

/// Init both the tab (InitTab()) and then init the AtTabInfo class
void AtTabBase::Init()
{
   InitTab();
   fTabInfo->Init();
}

/// Update both the tab (UpdateTab()) and then update the AtTabInfo class
void AtTabBase::Update()
{
   LOG(debug2) << std::endl << "Updating tab info" << fTabNumber;
   fTabInfo->Update();
   LOG(debug2) << std::endl << "Updating tab" << fTabNumber;
   UpdateTab();
   LOG(debug2) << std::endl << "Done updating tab" << fTabNumber << std::endl;
}
