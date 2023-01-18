#ifndef AtEVENTMANAGERNEW_H
#define AtEVENTMANAGERNEW_H

#include "AtDataSubject.h"
#include "AtEventSidebar.h"
#include "AtSubjectEventViewer.h"

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
 * Tabs are added directly to AtEventManager (which creates a task used to update the pads as events are loaded or
 * re-analyzed)
 */
class AtEventManagerNew : public TEveEventManager {
private:
   static AtEventManagerNew *fInstance;
   Int_t fEntry;

   AtEventSidebar *fSidebar;

   // TGListTreeItem *fEvent;
   /*** Sidebar info - Event # and event selection ***/
   TGNumberEntry *fCurrentEvent{nullptr};
   TGNumberEntry *f3DThresDisplay;

   std::unique_ptr<AtTabTask> fTabTask;
   std::shared_ptr<AtMap> fMap;
   Int_t fPadNum;

   /*** Sidebar info - Branch selection***/
   std::array<std::string, 3> fBranchTypes{"AtRawEvent", "AtEvent", "AtPatternEvent"};
   std::array<std::unique_ptr<DataHandling::BranchName>, 3> fSubjectBranchNames;
   std::array<TGComboBox *, 3> fBranchBoxes;
   std::array<std::vector<TString>, 3> fBranchNames;

public:
   static AtEventManagerNew *Instance();
   AtEventManagerNew(std::shared_ptr<AtMap> map);
   virtual ~AtEventManagerNew();

   void AddTask(FairTask *task);
   void AddTab(std::unique_ptr<AtTabBase> tab);

   virtual void Init();

   AtMap *GetMap() { return fMap.get(); }
   AtEventSidebar *GetSidebar() { return fSidebar; }
   Int_t GetCurrentEvent() { return fEntry; }

   /*** Functions used for event control
   "*MENU*" adds these function to the right click menue in the eve viewer
   */
   /// Main function for navigating to an event. Everything that changes event number should end up here
   virtual void GotoEvent(Int_t event);        ///< *MENU*
   void NextEvent() { GotoEvent(fEntry + 1); } ///< *MENU*
   void PrevEvent() { GotoEvent(fEntry - 1); } ///< *MENU*

   virtual void SelectEvent(); //< Goto event in fCurrentEvent (callback for fCurrentEvent)
   void SetCurrentEvent(TGNumberEntry *num) { fCurrentEvent = num; }

   /*** Functions for sidebar - Branch selection ***/
   void SelectAtRawEvent(Int_t);
   void SelectAtEvent(Int_t);
   void SelectAtPatternEvent(Int_t);
   void RedrawEvent();

   static void SelectPad();

private:
   void DrawPad(Int_t padNum);
   void RegisterDataHandles();

   /*** Functions used by sidebar - Branch control ***/
   void SelectEventBranch(int, int);
   void GenerateBranchLists();

   void make_gui();

   ClassDef(AtEventManagerNew, 1);
};

#endif
