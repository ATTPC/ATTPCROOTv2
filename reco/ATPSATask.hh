#ifndef ATPSATASK_H
#define ATPSATASK_H

// FAIRROOT classes
#include "FairTask.h"
class FairLogger;

// ATTPCROOT classes
class ATPSA;

// ROOT classes
class TClonesArray;

class ATPSATask:public FairTask {
 public:
    ATPSATask(ATPSA * psaMethod);
    ~ATPSATask();

    void SetPersistence(Bool_t value);
    virtual InitStatus Init();
    virtual void Exec(Option_t * opt);

 private:
     FairLogger * fLogger;

    TClonesArray *fRawEventArray;
    TClonesArray *fEventHArray;
    TClonesArray *fMCPointArray;

    ATPSA *fPSA;

    Bool_t fIsPersistence;

     ClassDef(ATPSATask, 2);
};

#endif
