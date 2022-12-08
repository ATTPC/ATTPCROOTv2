#ifndef AtEVENTMANAGERNEW_H
#define AtEVENTMANAGERNEW_H

#include <FairRunAna.h>

#include <Rtypes.h>
#include <TEveEventManager.h>

#include <vector>

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
class AtEvent;
class TClonesArray;

class AtEventManagerNew : public TEveEventManager {
private:
   FairRootManager *fRootManager;
   FairRunAna *fRunAna;
   TClonesArray *cArray;
   AtEvent *cevent;

   Int_t fEntry;
   TGListTreeItem *fEvent;
   TGNumberEntry *fCurrentEvent;
   TGNumberEntry *f3DThresDisplay;
   TCanvas *fCvsPadPlane;
   TCanvas *fPadWave;

   Bool_t kToggleData;

   Int_t fEntries;

   static AtEventManagerNew *fInstance;

public:
   static AtEventManagerNew *Instance();
   AtEventManagerNew();
   virtual ~AtEventManagerNew();

   virtual void GotoEvent(Int_t event); ///< *MENU*
   virtual void NextEvent();            ///< *MENU*
   virtual void PrevEvent();            ///< *MENU*
   virtual void make_gui();
   virtual void SelectEvent();

   static void DrawWave();

   void AddTask(FairTask *task) { fRunAna->AddTask(task); }
   // virtual void InitRiemann(Int_t option=1, Int_t level=3, Int_t nNodes=10000);
   virtual void Init(Int_t option = 1, Int_t level = 3, Int_t nNodes = 10000);

   virtual Int_t GetCurrentEvent() { return fEntry; }

   TCanvas *GetCvsPadPlane() { return fCvsPadPlane; }
   TCanvas *GetCvsPadWave() { return fPadWave; }

   void RunEvent();

   AtEventManagerNew(const AtEventManagerNew &);
   AtEventManagerNew &operator=(const AtEventManagerNew &);

   ClassDef(AtEventManagerNew, 1);
};

#endif
