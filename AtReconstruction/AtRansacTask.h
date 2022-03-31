#ifndef AtRANSACTASK_H
#define AtRANSACTASK_H

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtProtoEvent.h"
#include "AtDigiPar.h"
#include "AtRansac.h"
#include "AtRansacMod.h"
#include "AtMlesacMod.h"
#include "AtLmedsMod.h"

// ROOT classes
#include "TClonesArray.h"

class AtRansacTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;

   TClonesArray *fEventArray;
   TClonesArray *fRansacArray;

   AtEvent *fEvent;

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

public:
   AtRansacTask();
   ~AtRansacTask();

   void SetIsReprocess(Bool_t value = kFALSE);
   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetPersistence(Bool_t value = kTRUE);

   void SetModelType(int model);
   void SetDistanceThreshold(Float_t threshold);
   void SetNumItera(Int_t niterations);
   void SetFullMode();               // Mode that calculates the RANSAC method for every potential line
   void SetMinHitsLine(Int_t nhits); // Set Mininum number of hits per line
   void SetTiltAngle(Double_t val);
   void SetAlgorithm(Int_t val);
   void SetRanSamMode(Int_t mode);
   void SetChargeThreshold(Double_t value);
   void SetVertexMode(Int_t value);
   void SetInputBranchName(TString inputName);
   void SetOutputBranchName(TString outputName);

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(AtRansacTask, 2);
};

#endif
