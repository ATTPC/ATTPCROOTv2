#ifndef AtEVENTMANAGERNEW_H
#define AtEVENTMANAGERNEW_H

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

class AtEventManagerNew : public TEveEventManager {
private:
   Int_t fEntry;
   // TGListTreeItem *fEvent;
   TGNumberEntry *fCurrentEvent;
   TGNumberEntry *f3DThresDisplay;

   Bool_t kToggleData;

   Int_t fEntries;

   AtTabTask *fTabTask;
   std::shared_ptr<AtMap> fMap;
   static AtEventManagerNew *fInstance;

public:
   static AtEventManagerNew *Instance();
   AtEventManagerNew(std::shared_ptr<AtMap> map);
   virtual ~AtEventManagerNew();
   AtMap *GetMap() { return fMap.get(); }

   virtual void GotoEvent(Int_t event); ///< *MENU*
   virtual void NextEvent();            ///< *MENU*
   virtual void PrevEvent();            ///< *MENU*
   virtual void SelectEvent();

   static void SelectPad();
   void DrawPad(Int_t padNum);

   void AddTask(FairTask *task);

   virtual void Init();

   virtual Int_t GetCurrentEvent() { return fEntry; }

   void RunEvent();

   AtEventManagerNew(const AtEventManagerNew &);
   AtEventManagerNew &operator=(const AtEventManagerNew &);

private:
   void AddTabTask(AtTabTask *task);
   void make_gui();

   ClassDef(AtEventManagerNew, 1);
};

#endif
