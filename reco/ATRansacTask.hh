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
#include "ATRansacMod.hh"
#include "ATMlesacMod.hh"
#include "ATLmedsMod.hh"

// ROOT classes
#include "TClonesArray.h"

class ATRansacTask : public FairTask {

  public:
    ATRansacTask();
    ~ATRansacTask();

    void SetIsReprocess(Bool_t value = kFALSE);
    void SetPersistence(Bool_t value = kTRUE);
    void SetModelType(int model);
    void SetDistanceThreshold(Float_t threshold);
    void SetNumItera(Int_t niterations);
    void SetFullMode(); //Mode that calculates the RANSAC method for every potential line
    void SetMinHitsLine(Int_t nhits); //Set Mininum number of hits per line
    void SetTiltAngle(Double_t val);
    void SetAlgorithm(Int_t val);
    void SetRanSamMode(Int_t mode);
    void SetChargeThreshold(Double_t value);
    void SetVertexMode(Int_t value);

    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;
    ATDigiPar *fPar;
    TClonesArray *fEventHArray;
    TClonesArray *fRansacArray;
    TClonesArray *fS800CalcBr;

    ATEvent *fEvent;

    Bool_t kIsReprocess;
    Bool_t kIsPersistence;
    Bool_t kIsFullMode;
    int fRANSACModel;
    Float_t fRANSACThreshold;
    Int_t fMinHitsLine; // Minimum number of hits
    Double_t fTiltAngle;
    Int_t fNumItera;
    Int_t fRANSACAlg;
    Int_t fRandSamplMode;
    Bool_t fCharThres;
    Int_t fVertexMode;


  ClassDef(ATRansacTask, 1);
};

#endif
