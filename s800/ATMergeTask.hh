#ifndef ATMergeTask_H
#define ATMergeTask_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCROOT classes
#include "ATEvent.hh"

// ROOT classes
#include "TClonesArray.h"
#include "ATRawEvent.hh"

// S800 Classes
#include "S800Calc.hh"
#include "TCutG.h"

class ATMergeTask : public FairTask {

  public:
    ATMergeTask();
    ~ATMergeTask();

    void SetPersistence(Bool_t value = kTRUE);
    //void SetS800FileType(int model);
    void SetS800File(TString file);
    void SetGlom(Double_t glom);
    void SetOptiEvtDelta(Int_t EvtDelta);
    void SetPIDcut(TString file);

    Bool_t isInGlom(Double_t ts1, Double_t ts2);
    Bool_t isInPID(S800Calc *s800calc);


    virtual InitStatus Init();
//    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
//    ATDigiPar *fPar;
//    TClonesArray *fEventHArray;
    TClonesArray *fRawEventArray;
//    TClonesArray *fRansacArray;
//    S800Event *fS800Event;
//    S800Calc *fS800CalcBr;
//    S800 *fS800Br;
    TClonesArray *fS800CalcBr;
    //TClonesArray *fS800Br;
    TFile *fS800file;
    TString fS800strReader[2];
    TString fS800strTs[2];
    TString fcutPIDFile;

    //Int_t fS800FileType;
    Int_t fTsEvtS800Size,fEvtDelta;
    Double_t fGlom;
    TString fS800File;
    vector <Double_t> fS800Ts;
    vector <Double_t> fS800Evt;

    TF1 *fOptiFit;
    TCutG *fcutPID;

    ATEvent *fEvent;

    Bool_t fIsPersistence;



  ClassDef(ATMergeTask, 1);
};

#endif
