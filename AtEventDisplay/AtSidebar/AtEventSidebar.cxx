#include "AtEventSidebar.h"

#include <FairLogger.h>

#include <TGLabel.h>
void AtEventSidebar::FillFrames()
{
   for (auto frame : fFrames)
      frame->FillFrame();
}

AtEventSidebar::AtEventSidebar() : TGMainFrame(gClient->GetRoot(), 1000, 600)
{
   SetWindowName("XX GUI");
   SetCleanup(kDeepCleanup);

   // Add frame components that are always there
   auto runInfo = new AtSidebarRunInfo(this);
   auto runControl = new AtSidebarEventControl(this);
   AddSidebarFrame(runInfo);
   AddSidebarFrame(runControl);
}

void AtEventSidebar::AddSidebarFrame(AtSidebarFrame *frame)
{
   if (frame->GetParent() != this)
      LOG(fatal) << "Cannot pass a sidebar frame whose parent isn't the sidebar";

   fFrames.push_back(frame);
   TGMainFrame::AddFrame(frame);
}
