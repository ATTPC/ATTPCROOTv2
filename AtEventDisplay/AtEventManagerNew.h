#ifndef AtEVENTMANAGERNEW_H
#define AtEVENTMANAGERNEW_H

#include <FairRunAna.h>

#include <Rtypes.h>
#include <TEveEventManager.h>

#include "S800Ana.h"

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

   TGTextButton *drawallpad;
   TGTextButton *eraseQevent;
   TGTextButton *drawReconstruction;
   TGTextButton *saveASCIIevent;
   TGTextButton *toggleCorr;

   Bool_t kDrawAllOn;
   Bool_t kEraseQ;
   Bool_t kDrawReconstruction;
   Bool_t kDraw3DGeo;
   Bool_t kDraw3DHist;
   Bool_t kToggleData;
   Float_t k3DThreshold;

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
   virtual void Select3DThres();

   static void DrawWave();
   void ChangeDrawAllPads();
   void EnableDrawReconstruction();
   void EraseQEvent();
   void Draw3DGeo();
   void Draw3DHist();
   void ToggleCorrData();

   void AddTask(FairTask *task) { fRunAna->AddTask(task); }
   // virtual void InitRiemann(Int_t option=1, Int_t level=3, Int_t nNodes=10000);
   virtual void Init(Int_t option = 1, Int_t level = 3, Int_t nNodes = 10000);

   virtual Int_t GetCurrentEvent() { return fEntry; }

   TCanvas *GetCvsPadPlane() { return fCvsPadPlane; }
   TCanvas *GetCvsPadWave() { return fPadWave; }

   void SetTofObjCorr(std::vector<Double_t> vec) { fTofObjCorr = vec; }
   void SetMTDCObjRange(std::vector<Double_t> vec) { fMTDCObjRange = vec; }
   void SetMTDCXfRange(std::vector<Double_t> vec) { fMTDCXfRange = vec; }

   S800Ana fS800Ana;
   std::vector<Double_t> fTofObjCorr;
   std::vector<Double_t> fMTDCObjRange;
   std::vector<Double_t> fMTDCXfRange;

   Bool_t GetDrawAllPad() { return kDrawAllOn; }
   Bool_t GetDrawReconstruction() { return kDrawReconstruction; }
   Bool_t GetEraseQEvent()
   {
      Bool_t EraseBuff = kEraseQ;
      kEraseQ = kFALSE;
      return EraseBuff;
   }
   Float_t Get3DThreshold() { return k3DThreshold; }
   Bool_t GetToggleCorrData() { return kToggleData; }

   void RunEvent();

   void SaveASCIIEvent();

   AtEventManagerNew(const AtEventManagerNew &);
   AtEventManagerNew &operator=(const AtEventManagerNew &);

   ClassDef(AtEventManagerNew, 1);
};

#endif
