#ifndef ATMergeTask_H
#define ATMergeTask_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCROOT classes

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
  void SetS800File(TString file);
  void SetGlom(Double_t glom);
  void SetOptiEvtDelta(Int_t EvtDelta);
  void SetPIDcut(TString file);

  Bool_t isInGlom(Long64_t ts1, Long64_t ts2);
  Bool_t isInPID(S800Calc *s800calc);


  virtual InitStatus Init();
  //    virtual void SetParContainers();
  virtual void Exec(Option_t *opt);

private:
  FairLogger *fLogger;
  
  TClonesArray *fRawEventArray;
  S800Calc *fS800CalcBr;
  TFile *fS800file;

  Int_t fTsEvtS800Size,fEvtDelta;
  TString fS800File;
  vector <Long64_t> fS800Ts;
  vector <Double_t> fS800Evt;

  TF1 *fOptiFit;
  Double_t fGlom;
  TCutG *fcutPID;
  TString fcutPIDFile;

  Bool_t fIsPersistence;



  ClassDef(ATMergeTask, 1);
};

#endif
