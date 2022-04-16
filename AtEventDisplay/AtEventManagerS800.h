#ifndef AtEVENTMANAGERS800_H
#define AtEVENTMANAGERS800_H

#include <FairRunAna.h>

#include <Rtypes.h>
#include <TEveEventManager.h>

class TH2F;
class AtEvent;
class FairRootManager;
class FairTask;
class S800Calc;
class TBuffer;
class TCanvas;
class TClass;
class TClonesArray;
class TGNumberEntry;
class TGTextButton;
class TMemberInspector;
class TGListTreeItem;

class AtEventManagerS800 : public TEveEventManager {
public:
   static AtEventManagerS800 *Instance();
   AtEventManagerS800();
   virtual ~AtEventManagerS800();

   virtual void GotoEvent(Int_t event); ///< *MENU*
   virtual void NextEvent();            ///< *MENU*
   virtual void PrevEvent();            ///< *MENU*
   virtual void make_gui();
   virtual void SelectEvent();
   virtual void Select3DThres();

   static void DrawWave();
   void ChangeDrawAllPads();
   void EnableDrawHoughSpace();
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
   TCanvas *GetCvsPadAll() { return fPadAll; }
   TCanvas *GetCvsQEvent() { return fCvsQEvent; }
   TCanvas *GetCvsHoughSpace() { return fCvsHough; }
   TCanvas *GetCvsPhi() { return fCvsPhi; }
   TCanvas *GetCvsMesh() { return fCvsMesh; }
   TCanvas *GetCvs3DHist() { return fCvs3DHist; }
   TCanvas *GetCvsRad() { return fCvsRad; }
   TCanvas *GetCvsTheta() { return fCvsTheta; }
   TCanvas *GetCvsThetaxPhi() { return fCvsThetaxPhi; }
   TCanvas *GetCvsQuadrant1() { return fCvsQuadrant1; }
   TCanvas *GetCvsQuadrant2() { return fCvsQuadrant2; }
   TCanvas *GetCvsQuadrant3() { return fCvsQuadrant3; }
   TCanvas *GetCvsQuadrant4() { return fCvsQuadrant4; }
   TCanvas *GetCvsMC_XY() { return fCvsMC_XY; }
   TCanvas *GetCvsMC_Z() { return fCvsMC_Z; }
   TCanvas *GetCvsLvsTheta() { return fCvsLvsTheta; }
   TCanvas *GetCvsPID() { return fCvsPID; }
   TCanvas *GetCvsPID2() { return fCvsPID2; }

   Bool_t GetDrawAllPad() { return kDrawAllOn; }
   Bool_t GetDrawHoughSpace() { return kDrawHoughOn; }
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

   Int_t Entries{};
   TCanvas *fCvsPIDFull;
   TH2F *fPIDFull{};
   virtual void DrawPIDFull();
   virtual void FillPIDFull();
   TCanvas *fCvsPID2Full;
   TH2F *fPID2Full{};
   virtual void DrawPID2Full();

private:
   FairRootManager *fRootManager;
   FairRunAna *fRunAna;
   TClonesArray *cArray{};
   TClonesArray *cS800Array{};
   AtEvent *cevent{};
   S800Calc *cS800Calc{};

   Int_t fEntry;
   TGListTreeItem *fEvent;
   TGNumberEntry *fCurrentEvent;
   TGNumberEntry *f3DThresDisplay;
   TCanvas *fCvsPadPlane;
   TCanvas *fPadWave;
   TCanvas *fPadAll;
   TCanvas *fCvsQEvent;
   TCanvas *fCvsHough;
   TCanvas *fCvsPhi{};
   TCanvas *fCvsMesh;
   TCanvas *fCvs3DHist{};
   TCanvas *fCvsRad;
   TCanvas *fCvsTheta{};
   TCanvas *fCvsThetaxPhi{};
   TCanvas *fCvsQuadrant1{};
   TCanvas *fCvsQuadrant2{};
   TCanvas *fCvsQuadrant3{};
   TCanvas *fCvsQuadrant4{};
   TCanvas *fCvsMC_XY{};
   TCanvas *fCvsMC_Z{};
   TCanvas *fCvsLvsTheta;
   TCanvas *fCvsPID;
   TCanvas *fCvsPID2;

   TGTextButton *drawallpad;
   TGTextButton *eraseQevent;
   TGTextButton *drawHoughSpace;
   TGTextButton *saveASCIIevent;
   TGTextButton *toggleCorr;
   // TGCanvas *InfoCvs;
   Bool_t kDrawAllOn;
   Bool_t kDrawAllOff;
   Bool_t kEraseQ;
   Bool_t kDrawHoughOn;
   Bool_t kDraw3DGeo;
   Bool_t kDraw3DHist;
   Bool_t kToggleData;
   Float_t k3DThreshold;

   static AtEventManagerS800 *fInstance;

   AtEventManagerS800(const AtEventManagerS800 &);
   AtEventManagerS800 &operator=(const AtEventManagerS800 &);

   ClassDef(AtEventManagerS800, 1);
};

#endif
