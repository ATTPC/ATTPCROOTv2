#include "AtTabBase.h"
#include "AtTabInfo.h"

ClassImp(AtTabBase);

AtTabBase::AtTabBase()
: fTabNumber(0)
{
    fTabInfo = new AtTabInfo();
}

void AtTabBase::Init()
{
    InitTab();
    fTabInfo->Init();
}

void AtTabBase::Update()
{
    fTabInfo->Update();
    UpdateTab();
}
