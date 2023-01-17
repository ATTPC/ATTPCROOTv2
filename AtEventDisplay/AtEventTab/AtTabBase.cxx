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
    fTabInfo->Update();
    UpdateTab();
}
