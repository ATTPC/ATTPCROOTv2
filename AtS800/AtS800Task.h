#ifndef AtS800TASK_H
#define AtS800TASK_H

#include <Rtypes.h>
// FAIRROOT classes
#include <FairTask.h>

// AtTPCROOT classes

class FairLogger;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtS800Task : public FairTask {

public:
   AtS800Task() = default;
   ~AtS800Task() = default;

   void SetPersistence(Bool_t value = kTRUE);

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);

private:
   TClonesArray *fEventHArray{};
   TClonesArray *fS800Array{};

   Bool_t kIsPersistence = false;
   Bool_t kIsFullMode = false;
   int fS800Model{};
   Float_t fS800Threshold{};
   Int_t fMinHitsLine{}; // Minimum number of hits
   Double_t fTiltAngle{};
   Int_t fNumItera{};
   Int_t fS800Alg{};
   Int_t fRandSamplMode{};

   ClassDef(AtS800Task, 1);
};

#endif
