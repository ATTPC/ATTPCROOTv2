#ifndef AtEVENTMANAGERPROTO_H
#define AtEVENTMANAGERPROTO_H

#include <FairRunAna.h>

#include <Rtypes.h>
#include <TEveEventManager.h>

class FairRootManager;
class FairTask;
class TBuffer;
class TCanvas;
class TClass;
class TGNumberEntry;
class TGTextButton;
class TMemberInspector;
class TGListTreeItem;

class AtEventManagerProto : public TEveEventManager {
public:
   static AtEventManagerProto *Instance();
   AtEventManagerProto();
   virtual ~AtEventManagerProto();

   virtual void GotoEvent(Int_t event); ///< *MENU*
   virtual void NextEvent();            ///< *MENU*
   virtual void PrevEvent();            ///< *MENU*
   virtual void make_gui();
   virtual void SelectEvent();

   static void DrawWave();
   void EnableDrawPatternRecognition();

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
   TCanvas *GetCvsQuadrant1() { return fCvsQuadrant1; }
   TCanvas *GetCvsQuadrant2() { return fCvsQuadrant2; }
   TCanvas *GetCvsQuadrant3() { return fCvsQuadrant3; }
   TCanvas *GetCvsQuadrant4() { return fCvsQuadrant4; }
   TCanvas *GetCvsELQuadrant1() { return fCvsELQuadrant1; }
   TCanvas *GetCvsELQuadrant2() { return fCvsELQuadrant2; }
   TCanvas *GetCvsELQuadrant3() { return fCvsELQuadrant3; }
   TCanvas *GetCvsELQuadrant4() { return fCvsELQuadrant4; }
   TCanvas *GetCvsVertex() { return fCvsVertex; }
   TCanvas *GetCvsKineAA() { return fCvsKineAA; }
   TCanvas *GetCvsAux() { return fCvsAux; }

   Bool_t GetDrawPatternRecognition() { return kDrawPROn; }

   void RunEvent();
   void SaveASCIIEvent();

private:
   FairRootManager *fRootManager;
   FairRunAna *fRunAna;

   Bool_t kEraseQ{};
   Bool_t kDrawPROn;

   Int_t fEntry;
   TGListTreeItem *fEvent;
   TGNumberEntry *fCurrentEvent;
   TGTextButton *drawPatternRecognition;
   TGTextButton *saveASCIIevent;

   TCanvas *fCvsPadPlane{};
   TCanvas *fPadWave{};
   TCanvas *fPadAll{};
   TCanvas *fCvsQEvent{};
   TCanvas *fCvsHough{};
   TCanvas *fCvsPhi{};
   TCanvas *fCvsMesh{};
   TCanvas *fCvs3DHist{};
   TCanvas *fCvsQuadrant1{};
   TCanvas *fCvsQuadrant2{};
   TCanvas *fCvsQuadrant3{};
   TCanvas *fCvsQuadrant4{};
   TCanvas *fCvsELQuadrant1{};
   TCanvas *fCvsELQuadrant2{};
   TCanvas *fCvsELQuadrant3{};
   TCanvas *fCvsELQuadrant4{};
   TCanvas *fCvsVertex{};
   TCanvas *fCvsKineAA{};
   TCanvas *fCvsAux{};

   static AtEventManagerProto *fInstance;

   AtEventManagerProto(const AtEventManagerProto &);
   AtEventManagerProto &operator=(const AtEventManagerProto &);

   ClassDef(AtEventManagerProto, 1);
};

#endif
