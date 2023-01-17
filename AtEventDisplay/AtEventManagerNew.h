#ifndef AtEVENTMANAGERNEW_H
#define AtEVENTMANAGERNEW_H

#include "AtDataSubject.h"
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
class AtEventManagerNew : public TEveEventManager {
private:
   static AtEventManagerNew *fInstance;
   Int_t fEntry;

   /*** Sidebar info ***/
   std::array<std::string, 3> fBranchTypes{"AtRawEvent", "AtEvent", "AtPatternEvent"};
   std::array<std::unique_ptr<DataHandling::BranchName>, 3> fSubjectBranchNames;
   std::array<TGComboBox *, 3> fBranchBoxes;
   std::array<std::vector<TString>, 3> fBranchNames;

   // TGListTreeItem *fEvent;
   TGNumberEntry *fCurrentEvent;
   TGNumberEntry *f3DThresDisplay;

   Int_t fEntries;
   AtTabTask *fTabTask;
   std::shared_ptr<AtMap> fMap;
   Int_t fPadNum;

public:
   static AtEventManagerNew *Instance();
   AtEventManagerNew(std::shared_ptr<AtMap> map);
   virtual ~AtEventManagerNew();
   AtMap *GetMap() { return fMap.get(); }

   virtual void GotoEvent(Int_t event); ///< *MENU*
   virtual void NextEvent();            ///< *MENU*
   virtual void PrevEvent();            ///< *MENU*
   virtual void SelectEvent();

   void SelectAtRawEvent(Int_t);
   void SelectAtEvent(Int_t);
   void SelectAtPatternEvent(Int_t);

   void RedrawEvent();
   static void SelectPad();
   void DrawPad(Int_t padNum);

   void AddTask(FairTask *task);

   virtual void Init();

   virtual Int_t GetCurrentEvent() { return fEntry; }

   void RunEvent();

   AtEventManagerNew(const AtEventManagerNew &);
   AtEventManagerNew &operator=(const AtEventManagerNew &);

private:
   void SelectEventBranch(int, int);
   void GenerateBranchLists();
   void RegisterDataHandles();
   void AddTabTask(AtTabTask *task);
   void make_gui();

   ClassDef(AtEventManagerNew, 1);
};

#endif
