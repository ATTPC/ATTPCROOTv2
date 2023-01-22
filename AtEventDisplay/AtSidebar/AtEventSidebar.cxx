#include "AtEventSidebar.h"

#include "AtDataObserver.h"
#include "AtViewerManager.h"

#include <FairLogger.h>

#include <TGLabel.h>
void AtEventSidebar::FillFrames()
{
   for (auto frame : fFrames)
      frame->FillFrame();

   MapSubwindows();
   Resize();
   MapWindow();
}

AtEventSidebar::AtEventSidebar(DataHandling::AtTreeEntry &entryNum, DataHandling::AtBranch &rawEvent,
                               DataHandling::AtBranch &event, DataHandling::AtBranch &patternEvent)
   : TGMainFrame(gClient->GetRoot(), 1000, 600)
{
   SetWindowName("XX GUI");
   SetCleanup(kDeepCleanup);

   // Add frame components that are always there
   auto runInfo = new AtSidebarRunInfo(this);
   auto runControl = new AtSidebarEventControl(entryNum, AtViewerManager::Instance()->GetPadNum(), this);
   auto branchControl = new AtSidebarBranchControl(rawEvent, event, patternEvent, this);

   AddSidebarFrame(runInfo);
   AddSidebarFrame(runControl);
   AddSidebarFrame(branchControl);
}

void AtEventSidebar::AddSidebarFrame(AtSidebarFrame *frame)
{
   if (frame->GetParent() != this)
      LOG(fatal) << "Cannot pass a sidebar frame whose parent (" << frame->GetParent() << ") isn't the sidebar ("
                 << this << ")";

   fFrames.push_back(frame);
   TGMainFrame::AddFrame(frame, new TGLayoutHints(kLHintsExpandX));
   frame->Layout();
}

void AtEventSidebar::UsePictureButtons(bool val)
{
   for (auto frame : fFrames)
      frame->UsePictureButtons(val);
}
