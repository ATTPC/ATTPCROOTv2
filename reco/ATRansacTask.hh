#ifndef ATRANSACTASK_H
#define ATRANSACTASK_H



// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATDigiPar.hh"
#include "ATRansac.hh"

// ROOT classes
#include "TClonesArray.h"

class ATRansacTask : public FairTask {

  public:
    ATRansacTask();
    ~ATRansacTask();

    void SetPersistence(Bool_t value = kTRUE);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
    ATDigiPar *fPar;
    TClonesArray *fEventHArray;
    TClonesArray *fRansacArray;

    ATEvent *fEvent;

    Bool_t fIsPersistence;


  ClassDef(ATRansacTask, 1);
};

#endif
