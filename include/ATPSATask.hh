#ifndef ATPSATASK_H
#define ATPSATASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// ATTPCROOT classes
#include "ATRawEvent.hh"
#include "ATDigiPar.hh"
#include "ATPSA.hh"

// ROOT classes
#include "TClonesArray.h"

class ATPSATask : public FairTask {
  public:
    ATPSATask();
    ~ATPSATask();

    void SetPSAMode(Int_t value = 0);
    void SetPersistence(Bool_t value = kTRUE);
    void SetThreshold(Double_t threshold);
    void SetBackGroundPeakFinder(Bool_t value);
    void SetPeakFinder();
    void SetMaxFinder();
    void SetBaseCorrection(Bool_t value);
    void SetTimeCorrection(Bool_t value);
    void SetGainCalibration(TString gainFile);
    void SetJitterCalibration(TString jitterFile);
    //void SetAuxChannel(std::vector<Int_t> AuxCh);
    //void EnableAuxChannels(std::vector<Int_t> AuxCh);

    //For PCL Outlier Removal
    void SetMeanK(Int_t value); //Number of neighbors
    void SetStddevMulThresh(Double_t value);
    void SetTBLimits(std::pair<Int_t,Int_t> limits);


    virtual InitStatus Init();
    virtual void SetParContainers();
    virtual void Exec(Option_t *opt);

  private:
    FairLogger *fLogger;

    ATDigiPar *fPar;
    TClonesArray *fRawEventArray;
    TClonesArray *fEventHArray;

    ATPSA *fPSA;
    Int_t fPSAMode;

    Bool_t fIsPersistence;
    Bool_t fIsBGPK;
    Bool_t fIsPeakFinder;
    Bool_t fIsMaxFinder;
    Bool_t fIsBaseCorr;
    Bool_t fIsTimeCorr;

    TString fGainFile;
    TString fJitterFile;

    Double_t fThreshold;
    //std::vector<Int_t> fAuxChannels;
    Int_t fMeanK;
    Double_t fStdDev;
    std::pair<Int_t,Int_t> fTBRange;

  ClassDef(ATPSATask, 1);
};

#endif
