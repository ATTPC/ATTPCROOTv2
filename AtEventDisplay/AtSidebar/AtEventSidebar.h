#ifndef AtEVENTSIDEBAR_H
#define AtEVENTSIDEBAR_H
#include "AtSidebarFrames.h"

#include <TGFrame.h>
namespace DataHandling {
class AtEntryNumber;
}

/**
 * Sidebar class containings frames that also can broadcast data as a DataHandling::Subject.
 */
class AtEventSidebar : public TGMainFrame {
private:
   std::vector<AtSidebarFrame *> fFrames;

public:
   AtEventSidebar(DataHandling::AtEntryNumber *entryNum, DataHandling::AtBranchName &rawEvent,
                  DataHandling::AtBranchName &event, DataHandling::AtBranchName &patternEvent);

   /// Actually generate content of the frames once Init has run
   void FillFrames();
   void AddSidebarFrame(AtSidebarFrame *frame);
};

#endif
