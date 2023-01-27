#include "AtEventSidebar.h"

#include "AtSidebarFrames.h" // for AtSidebarFrame, AtSidebarBranchControl

#include <FairLogger.h>

#include <TGClient.h> // for TGClient, gClient
#include <TGLayout.h> // for TGLayoutHints, kLHintsExpandX

#include <algorithm> // for max
void AtEventSidebar::FillFrames()
{
   for (auto frame : fFrames)
      frame->FillFrame();

   MapSubwindows();
   Resize();
   MapWindow();
}

AtEventSidebar::AtEventSidebar(UInt_t options) : TGMainFrame(gClient->GetRoot(), 1000, 600, options)
{
   SetWindowName("XX GUI");
   SetCleanup(kDeepCleanup);
   fExpandX = options & kVerticalFrame;
}

void AtEventSidebar::AddSidebarFrame(AtSidebarFrame *frame)
{
   if (frame->GetParent() != this)
      LOG(fatal) << "Cannot pass a sidebar frame whose parent (" << frame->GetParent() << ") isn't the sidebar ("
                 << this << ")";

   fFrames.push_back(frame);
   if (fExpandX)
      TGMainFrame::AddFrame(frame, new TGLayoutHints(kLHintsExpandX));
   else
      TGMainFrame::AddFrame(frame, new TGLayoutHints(kLHintsExpandY));
   frame->Layout();
}

void AtEventSidebar::UsePictureButtons(bool val)
{
   for (auto frame : fFrames)
      frame->UsePictureButtons(val);
}
