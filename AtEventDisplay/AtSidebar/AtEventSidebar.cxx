#include "AtEventSidebar.h"

#include "AtDataObserver.h"

#include <FairLogger.h>

#include <TGLabel.h>
void AtEventSidebar::FillFrames()
{
   for (auto frame : fFrames)
      frame->FillFrame();
}

AtEventSidebar::AtEventSidebar(DataHandling::AtEntryNumber *entryNum,
                               DataHandling::AtBranchName &rawEvent,
                               DataHandling::AtBranchName &event,
                               DataHandling::AtBranchName &patternEvent)
   : TGMainFrame(gClient->GetRoot(), 1000, 600)
{
   SetWindowName("XX GUI");
   SetCleanup(kDeepCleanup);

   // Add frame components that are always there
   auto runInfo = new AtSidebarRunInfo(this);
   auto runControl = new AtSidebarEventControl(entryNum, this);
   auto branchControl = new AtSidebarBranchControl(rawEvent, event, patternEvent, this);
   AddSidebarFrame(runInfo);
   AddSidebarFrame(runControl);
   AddSidebarFrame(branchControl);
}

void AtEventSidebar::AddSidebarFrame(AtSidebarFrame *frame)
{
   if (frame->GetParent() != this)
      LOG(fatal) << "Cannot pass a sidebar frame whose parent isn't the sidebar";

   fFrames.push_back(frame);
   TGMainFrame::AddFrame(frame, new TGLayoutHints(kLHintsExpandX));
}
