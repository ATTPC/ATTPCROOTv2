#ifndef ATVIEWERMANAGER_H
#define ATVIEWERMANAGER_H

#include "AtDataObserver.h"
#include "AtDataSubject.h"
#include "AtEventSidebar.h"
#include "AtViewerManagerSubject.h"

#include <FairRunAna.h>

#include <Rtypes.h>
#include <TEveEventManager.h>
#include <TEveManager.h>

#include <vector>

class AtTabTask;
class FairRootManager;
class FairTask;
class TBuffer;
class TCanvas;
class TH2F;
class TClass;
class TGNumberEntry;
class TGTextButton;
class TMemberInspector;
class TGListTreeItem;
class TList;
class AtMap;
class TGComboBox;
class AtTabBase;

/**
 * Event manager than allows for the addition arbitrary pads to expand behavior.
 * Operates as a singleton who needs to be instatiated once with the detector map in use.
 *
 * Tabs are added directly to AtEventManager (which creates a task used to update the pads as events
 * are loaded or re-analyzed)
 */
class AtViewerManager final : DataHandling::Observer {
private:
   using TabVec = std::vector<std::unique_ptr<AtTabBase>>;

   /** Data subjects we own and send around on update **/
   DataHandling::AtTreeEntry fEntry{0};          //< Observing
   DataHandling::AtBranch fRawEventBranch{};     //< Not observing
   DataHandling::AtBranch fEventBranch{};        //< Not observing
   DataHandling::AtBranch fPatternEventBranch{}; //< Not observing
   DataHandling::AtPadNum fPadNum{-1};           //< Not Observing

   AtEventSidebar *fSidebar;

   std::shared_ptr<AtMap> fMap;
   std::map<TString, std::vector<TString>> fBranchNames; //< fBranchNames[type] = {list of branches}
   TabVec fTabs;

   static AtViewerManager *fInstance;

public:
   AtViewerManager(std::shared_ptr<AtMap> map);
   ~AtViewerManager();

   void AddTask(FairTask *task);
   void AddTab(std::unique_ptr<AtTabBase> tab);

   void Init();
   virtual void Update(DataHandling::Subject *) override;

   AtMap *GetMap() { return fMap.get(); }
   AtEventSidebar *GetSidebar() { return fSidebar; }
   std::map<TString, std::vector<TString>> const &GetBranchNames() const { return fBranchNames; }

   /*** Data handlers owned by AtViewerManager ***/
   DataHandling::AtBranch &GetRawEventName() { return fRawEventBranch; }
   DataHandling::AtBranch &GetEventName() { return fEventBranch; }
   DataHandling::AtBranch &GetPatternEventName() { return fPatternEventBranch; }
   DataHandling::AtTreeEntry &GetCurrentEntry() { return fEntry; }
   DataHandling::AtPadNum &GetPadNum() { return fPadNum; }

   /**
    * Main function for navigating to an event. Everything that changes event number should end up
    * here
    */
   virtual void GotoEvent(Int_t event) { fEntry.Set(event); }
   void NextEvent() { GotoEvent(fEntry.Get() + 1); }
   void PrevEvent() { GotoEvent(fEntry.Get() - 1); }

   static AtViewerManager *Instance();

private:
   void GenerateBranchLists();
   void GotoEventImpl();

   ClassDef(AtViewerManager, 1);
};

#endif
