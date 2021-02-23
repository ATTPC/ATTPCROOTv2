#ifndef AtS800TASK_H
#define AtS800TASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// AtTPCROOT classes


// ROOT classes
#include "TClonesArray.h"

class AtS800Task : public FairTask {

  public:
    AtS800Task();
    ~AtS800Task();

    void SetPersistence(Bool_t value = kTRUE);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;

    TClonesArray *fEventHArray;
    TClonesArray *fS800Array;

    

    Bool_t kIsPersistence;
    Bool_t kIsFullMode;
    int fS800Model;
    Float_t fS800Threshold;
    Int_t fMinHitsLine; // Minimum number of hits
    Double_t fTiltAngle;
    Int_t fNumItera;
    Int_t fS800Alg;
    Int_t fRandSamplMode;



  ClassDef(AtS800Task, 1);
};

#endif
