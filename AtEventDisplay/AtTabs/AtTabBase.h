#ifndef ATTABBASE_H
#define ATTABBASE_H

#include "AtTabInfo.h" // IWYU pragma: keep

#include <Rtypes.h>
#include <TString.h> // for TString

#include <memory>
#include <string>  // for string
#include <utility> // for move

class TBuffer;
class TMemberInspector;
class TClass;
class TEveWindowSlot;

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
   static int fNumTabs; //< Number of tab objects created
   Int_t fTabId{0};     //< Unique ID for tab
   TString fTabName;    //< Name for the tab

   std::unique_ptr<AtTabInfo> fTabInfo{std::make_unique<AtTabInfo>()};

public:
   AtTabBase(TString tabName);
   virtual ~AtTabBase() = default;

   /// Called in the init stage of the run.
   void Init();

   /// Called after the run's `Exec()` to update tab
   virtual void Exec() = 0;

   AtTabInfo *GetTabInfo() { return fTabInfo.get(); }
   void SetTabName(TString name) { fTabName = std::move(name); }

protected:
   /// Responsible for creating the fTabInfo object that will be updated on each event.
   /// That fTabInfo object will be initialized without user input though
   virtual void InitTab() = 0;

   /**
    * @brief Create the gui components of the tab in the passed window slot.
    */
   virtual void MakeTab(TEveWindowSlot *slot) = 0;

   /**
    * Returns the instance of T (usually an event type) associated with the AtTabInfoFairRoot
    * augment named infoName. If infoName is not passed assume its name is "T".
    */
   template <typename T>
   T *GetFairRootInfo(std::string infoName = T::Class_Name())
   {
      return dynamic_cast<AtTabInfoFairRoot<T> *>(fTabInfo->GetAugment(infoName).get())->GetInfo();
   }

   ClassDef(AtTabBase, 1)
};

#endif
