#ifndef AtPSAtASK_H
#define AtPSAtASK_H

// FAIRROOT classes
#include "FairTask.h"
class FairLogger;

// AtTPCROOT classes
class AtPSA;

// ROOT classes
class TClonesArray;

class AtPSAtask:public FairTask {
 public:
    AtPSAtask(AtPSA * psaMethod);
    ~AtPSAtask();

    void SetPersistence(Bool_t value);
    virtual InitStatus Init();
    virtual void Exec(Option_t * opt);

 private:

    TClonesArray *fRawEventArray;
    TClonesArray *fEventHArray;
    TClonesArray *fMCPointArray;

    AtPSA *fPSA;

    Bool_t fIsPersistence;

     ClassDef(AtPSAtask, 2);
};

#endif
