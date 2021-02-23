#ifndef AtPHIRECOTASK_H
#define AtPHIRECOTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// AtTPCTROOT classes
#include "AtEvent.h"
#include "AtProtoEvent.h"
#include "AtDigiPar.h"
#include "AtHoughSpaceLine.h"
#include "AtHoughSpace.h"
#include "AtPhiReco.h"
#include "AtPhiRecoSimple.h"
#include "AtPhiRecoTriple.h"


// ROOT classes
#include "TClonesArray.h" 

class AtPhiRecoTask : public FairTask {
  public:
    AtPhiRecoTask();
    ~AtPhiRecoTask();

   
    void SetPersistence(Bool_t value = kTRUE);
    void SetThreshold(Double_t threshold);
    void SetPhiRecoMode(Int_t value = 0);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
    
    AtDigiPar *fPar;
    
    TClonesArray *fEventHArray;
    TClonesArray *fPEventArray; 

    AtPhiReco *fPhiReco;
    Int_t fPhiRecoMode;

    Bool_t fIsPersistence;
    
    Double_t fThreshold;

  ClassDef(AtPhiRecoTask, 1);
};

#endif
