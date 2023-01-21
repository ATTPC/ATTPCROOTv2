#ifndef ATTABBASE_H
#define ATTABBASE_H

#include "AtTabInfo.h" // IWYU pragma: keep

#include <Rtypes.h>

#include <memory>
class TClass;

/**
 * @brief Base class for all tabs that can be added to the event viewer.
 *
 * Each tab owns an AtTabInfo object that is responsible for knowning what information is needed
 * to draw or display whatever is in the tab. This is Attached to every DataHandling::Subject that
 * is in the sidebar of the event viewer (Or Subject that has been added to the AtEventManagerNew).
 *
 * @defgroup Tabs Viewer Tabs
 */
class AtTabBase {
protected:
   Int_t fTabNumber{0};

   std::unique_ptr<AtTabInfo> fTabInfo{std::make_unique<AtTabInfo>()};

public:
   AtTabBase() = default;
   virtual ~AtTabBase() = default;

   /// Called in the init stage of the run.
   void Init();
   /**
    * @brief Create the gui components of the tab
    * Called after Init() in init run stage
    */
   virtual void MakeTab() = 0;

   /// Called in each Exec()
   void Update();
   /// Called at the end of each Exec()
   virtual void DrawEvent() = 0;

   AtTabInfo *GetTabInfo() { return fTabInfo.get(); }

   // Update tab with newly selected pad
   virtual void DrawPad(Int_t padNum) = 0;
   void SetTabNumber(Int_t tabNum) { fTabNumber = tabNum; }

protected:
   /// Responsible for creating the fTabInfo object that will be updated on each event.
   virtual void InitTab() = 0;
   virtual void UpdateTab() = 0;

   /**
    * Returns the instance of T (usually an event type) associated with the AtTabInfoFairRoot
    * augment named infoName. If infoName is not passed assume its name is "T".
    */
   template <typename T>
   T *GetFairRootInfo(std::string infoName = T::Class_Name())
   {
      return dynamic_cast<AtTabInfoFairRoot<T> *>(fTabInfo->GetAugment(infoName))->GetInfo();
   }

   ClassDef(AtTabBase, 1)
};

#endif
