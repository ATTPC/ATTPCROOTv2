#ifndef AtEVENTSIDEBAR_H
#define AtEVENTSIDEBAR_H
#include "AtSidebarFrames.h"

#include <TGFrame.h>
#include <TGLabel.h>

/** This is the parent frame of all Sidebar frames
 */
class AtEventSidebar : public TGMainFrame {
private:
   std::vector<AtSidebarFrame *> fFrames;

public:
   enum class FrameTypes { kRunInfo, kEventControl };

   AtEventSidebar() : TGMainFrame(gClient->GetRoot(), 1000, 600)
   {
      SetWindowName("XX GUI");
      SetCleanup(kDeepCleanup);

      // Add frame components that are always there
      AddSidebarFrame(FrameTypes::kRunInfo);
      AddSidebarFrame(FrameTypes::kEventControl);
   }

   /// Actually generate content of the frames once Init has run
   void FillFrames()
   {
      for (auto frame : fFrames)
         frame->FillFrame();
   }

   /**
    * @brief Add a frame to the sidebar
    *
    * Because the frames need to know who their parent is (this) they have to be instantiated after
    * the sidebar object and know the address of the sidebar object. To make life easier this factory method handles the
    * parent of the AtSidebarFrame and simple forwards every other passed parameter to the constructor of the correct
    * frame type.
    *
    * Because this is a TGMainFrame as well you can always construct additional frames ussing the normal AddFrame
    * method.
    * The order this is called also controls the order in which frames are added to the sidebar.
    */
   template <typename... Ts>
   AtSidebarFrame *AddSidebarFrame(FrameTypes type, Ts &&...params)
   {
      AtSidebarFrame *frame = nullptr;
      switch (type) {
      case FrameTypes::kRunInfo: frame = new AtSidebarRunInfo(this, std::forward<Ts>(params)...); break;
      case FrameTypes::kEventControl: frame = new AtSidebarEventControl(this, std::forward<Ts>(params)...); break;
      }
      fFrames.push_back(frame);
      this->AddFrame(frame);
      return frame;
   }
};

#endif
