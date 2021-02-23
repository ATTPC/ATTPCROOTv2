#ifndef AtVMEUNPACKTASK_H
#define AtVMEUNPACKTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

#include "AtDigiPar.h"
#include "VMECore.h"
#include "VMERawEvent.h"
#include "AtRawIC.h"

#include "TClonesArray.h"
#include "TString.h"

#include <vector>

class AtVMEUnpackTask : public FairTask {
  public:
    
    AtVMEUnpackTask();
    ~AtVMEUnpackTask();

    void AddData(TString filename);
    void SetData(Int_t value);
    void SetPersistence(Bool_t value = kTRUE);
    void SetICChannel(Int_t value);
    void SetMeshChannel(Int_t value);
    void SetTriggerChannel(Int_t value);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

   

   private:
  
    VMECore *fVMEDecoder;
    std::vector<TString> fDataList;    
    Int_t fDataNum;
    Bool_t fIsPersistence;

    Int_t fICChannel;
    Int_t fMeshChannel;
    Int_t fTriggerChannel;
  
    FairLogger *fLogger; 
    AtDigiPar *fPar;   

    Int_t fEventID;
  
    TClonesArray *fVMERawEventArray;

       
   ClassDef(AtVMEUnpackTask, 1);
};

#endif
