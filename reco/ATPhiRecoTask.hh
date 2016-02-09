#ifndef ATPHIRECOTASK_H
#define ATPHIRECOTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCTROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATDigiPar.hh"
#include "ATHoughSpaceLine.hh"
#include "ATHoughSpace.hh"
#include "ATPhiReco.hh"
#include "ATPhiRecoSimple.hh"
#include "ATPhiRecoTriple.hh"


// ROOT classes
#include "TClonesArray.h" 

class ATPhiRecoTask : public FairTask {
  public:
    ATPhiRecoTask();
    ~ATPhiRecoTask();

   
    void SetPersistence(Bool_t value = kTRUE);
    void SetThreshold(Double_t threshold);
    void SetPhiRecoMode(Int_t value = 0);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
    
    ATDigiPar *fPar;
    
    TClonesArray *fEventHArray;
    TClonesArray *fPEventArray; 

    ATPhiReco *fPhiReco;
    Int_t fPhiRecoMode;

    Bool_t fIsPersistence;
    
    Double_t fThreshold;

  ClassDef(ATPhiRecoTask, 1);
};

#endif
