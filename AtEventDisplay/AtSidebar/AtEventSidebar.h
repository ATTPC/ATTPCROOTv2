#ifndef AtEVENTSIDEBAR_H
#define AtEVENTSIDEBAR_H
#include "AtSidebarFrames.h"

#include <TGFrame.h>

/** This is the parent frame of all Sidebar frames
 */
class AtEventSidebar : public TGMainFrame {
private:
   std::vector<AtSidebarFrame *> fFrames;

public:
   AtEventSidebar();

   /// Actually generate content of the frames once Init has run
   void FillFrames();
   void AddSidebarFrame(AtSidebarFrame *frame);
};

#endif
