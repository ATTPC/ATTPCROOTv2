#ifndef ATVIEWERMANAGER_H
#define ATVIEWERMANAGER_H
#include "AtDataObserver.h"
#include "AtViewerManagerSubject.h"

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TString.h> // for TString

#include <map>    // for map
#include <memory> // for shared_ptr, unique_ptr
#include <vector>

class AtEventSidebar;
class AtMap;            // lines 29-29
class AtRawEvent;
class AtTabBase;        // lines 31-31
class FairTask;         // lines 19-19
class TBuffer;          // lines 20-20
class TClass;           // lines 23-23
class TMemberInspector; // lines 26-26
namespace DataHandling {
class AtSubject;
}
/**
 * Event manager than allows for the addition arbitrary pads to expand behavior.
 * Operates as a singleton who needs to be instatiated once with the detector map in use.
 *
 * Tabs are added directly to AtEventManager (which creates a task used to update the pads as events
 * are loaded or re-analyzed)
 */
class AtViewerManager final : DataHandling::AtObserver {
private:
   using TabVec = std::vector<std::unique_ptr<AtTabBase>>;

   /** Data subjects we own and send around on update **/
   DataHandling::AtTreeEntry fEntry{0};          //< Observing
   DataHandling::AtBranch fRawEventBranch{};     //< Not observing
   DataHandling::AtBranch fEventBranch{};        //< Not observing
   DataHandling::AtBranch fPatternEventBranch{}; //< Not observing
   DataHandling::AtPadNum fPadNum{-1};           //< Not Observing

   AtEventSidebar *fSidebar;
   AtEventSidebar *fBasebar;

   std::shared_ptr<AtMap> fMap;
   std::map<TString, std::vector<TString>> fBranchNames; //< fBranchNames[type] = {list of branches}
   TabVec fTabs;

   bool fCheckGood{false}; //< Check if the event is good and skip if not when using next and prev
   TClonesArray *fCheckEventArray{nullptr}; //< AtRawEvent branch object
   TString fCheckBranchName{"AtRawEvent"};  //< Name of AtRawEvent branch to check as good/not good
   AtRawEvent *fCheckRawEvent{};                 //< Event to check as good

   static AtViewerManager *fInstance;

public:
   AtViewerManager(std::shared_ptr<AtMap> map);
   ~AtViewerManager();

   void AddTask(FairTask *task);
   void AddTab(std::unique_ptr<AtTabBase> tab);

   void Init();
   virtual void Update(DataHandling::AtSubject *) override;

   AtMap *GetMap() { return fMap.get(); }
   AtEventSidebar *GetSidebar() { return fSidebar; }
   std::map<TString, std::vector<TString>> const &GetBranchNames() const { return fBranchNames; }

   /*** Data handlers owned by AtViewerManager ***/
   DataHandling::AtBranch &GetRawEventName() { return fRawEventBranch; }
   DataHandling::AtBranch &GetEventName() { return fEventBranch; }
   DataHandling::AtBranch &GetPatternEventName() { return fPatternEventBranch; }
   DataHandling::AtTreeEntry &GetCurrentEntry() { return fEntry; }
   DataHandling::AtPadNum &GetPadNum() { return fPadNum; }

   void SetCheckGood() { fCheckGood = true; }
   void SetCheckBranch(TString name) { fCheckBranchName = name; }

   /**
    * Main function for navigating to an event. Everything that changes event number should end up
    * here
    */
   virtual void GotoEvent(Int_t event) { fEntry.Set(event); }
   void NextEvent();
   void PrevEvent();

   static AtViewerManager *Instance();

private:
   void GenerateBranchLists();
   void GotoEventImpl();

   ClassDef(AtViewerManager, 1);
};

#endif
