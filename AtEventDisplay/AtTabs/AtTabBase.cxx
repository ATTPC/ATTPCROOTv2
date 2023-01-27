#include "AtTabBase.h"

#include <TEveBrowser.h>
#include <TEveManager.h>
#include <TEveWindow.h>

ClassImp(AtTabBase);
int AtTabBase::fNumTabs = 0;

AtTabBase::AtTabBase(TString name) : fTabId(fNumTabs), fTabName(std::move(name))
{
   fNumTabs++;
}

/// Init both the tab (InitTab()) and then init the AtTabInfo class
void AtTabBase::Init()
{
   InitTab();
   MakeTab(TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight()));
}
