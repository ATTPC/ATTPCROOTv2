#ifndef ATVMEUNPACKTASK_H
#define ATVMEUNPACKTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

#include "ATDigiPar.hh"
#include "VMECore.hh"
#include "VMERawEvent.hh"
#include "ATRawIC.hh"

#include "TClonesArray.h"
#include "TString.h"

#include <vector>

class ATVMEUnpackTask : public FairTask {
  public:
    
    ATVMEUnpackTask();
    ~ATVMEUnpackTask();

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
    ATDigiPar *fPar;   

    Int_t fEventID;
  
    TClonesArray *fVMERawEventArray;

       
   ClassDef(ATVMEUnpackTask, 1);
};

#endif
