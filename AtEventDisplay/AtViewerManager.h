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
   static AtViewerManager *fInstance;

   AtEventSidebar *fSidebar;

   // TGListTreeItem *fEvent;
   /*** Sidebar info - Event # and event selection ***/
   // TGNumberEntry *fCurrentEvent{nullptr};
   TGNumberEntry *f3DThresDisplay;

   std::unique_ptr<AtTabTask> fTabTask;
   std::shared_ptr<AtMap> fMap;
   Int_t fPadNum;

   /** Data subjects we own and send around on update **/
   DataHandling::AtEntryNumber fEntry{0};
   DataHandling::AtBranchName fRawEventBranch{};
   DataHandling::AtBranchName fEventBranch{};
   DataHandling::AtBranchName fPatternEventBranch{};

   /** Should have the list of branch names **/
   std::map<std::string, std::vector<TString>> fBranchNames;

   /*** Sidebar info - Branch selection***/

public:
   static AtViewerManager *Instance();
   AtViewerManager(std::shared_ptr<AtMap> map);
   ~AtViewerManager();

   void AddTask(FairTask *task);
   void AddTab(std::unique_ptr<AtTabBase> tab);

   void Init();
   virtual void Update(DataHandling::Subject *) override;

   AtMap *GetMap() { return fMap.get(); }
   AtEventSidebar *GetSidebar() { return fSidebar; }
   std::map<std::string, std::vector<TString>> const &GetBranchNames() const
   {
      return fBranchNames;
   }

   /*** Data handlers owned by AtViewerManager ***/
   DataHandling::AtBranchName &GetRawEventName() { return fRawEventBranch; }
   DataHandling::AtBranchName &GetEventName() { return fEventBranch; }
   DataHandling::AtBranchName &GetPatternEventName() { return fPatternEventBranch; }

   Int_t GetCurrentEvent() { return fEntry.Get(); }

   /*
    * Functions used for event control
    * "*MENU*" adds these function to the right click menue in the eve viewer
    */

   /**
    * Main function for navigating to an event. Everything that changes event number should end up
    * here
    */
   virtual void GotoEvent(Int_t event);              ///< *MENU*
   void NextEvent() { GotoEvent(fEntry.Get() + 1); } ///< *MENU*
   void PrevEvent() { GotoEvent(fEntry.Get() - 1); } ///< *MENU*
   void RedrawEvent();                               ///< *MENU*

   static void SelectPad();

private:
   void DrawPad(Int_t padNum);
   void RegisterDataHandles();

   /*** Functions used by sidebar - Branch control ***/
   void SelectEventBranch(int, int);
   void GenerateBranchLists();

   void make_gui();

   ClassDef(AtViewerManager, 1);
};

#endif