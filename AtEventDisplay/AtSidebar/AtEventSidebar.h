#ifndef AtEVENTSIDEBAR_H
#define AtEVENTSIDEBAR_H

#include <Rtypes.h> // for UInt_t
#include <TGFrame.h>

#include <GuiTypes.h> // for kVerticalFrame

#include <vector> // for vector
class AtSidebarFrame;

/**
 * Side or basebar class containings frames.
 */
class AtEventSidebar : public TGMainFrame {
private:
   std::vector<AtSidebarFrame *> fFrames;
   bool fExpandX;

public:
   AtEventSidebar(UInt_t options = kVerticalFrame);

   /// Actually generate content of the frames once Init has run
   void FillFrames();
   void AddSidebarFrame(AtSidebarFrame *frame);
   void UsePictureButtons(bool val);
};

#endif
