#ifndef AtRANSACTASK_H
#define AtRANSACTASK_H

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>  // for Int_t, Bool_t, Double_t, THashConsistencyHolder
#include <TString.h> // for TString
class AtEvent;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

class AtRansacTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;

   TClonesArray *fEventArray{};
   TClonesArray *fRansacArray{};

   AtEvent *fEvent{};

   Bool_t kIsReprocess;
   Bool_t kIsPersistence;
   Bool_t kIsFullMode;
   int fRANSACModel{-1};
   Float_t fRANSACThreshold{5.0};
   Int_t fMinHitsLine{5}; // Minimum number of hits
   Double_t fTiltAngle{};
   Int_t fNumItera{500};
   Int_t fRANSACAlg{0};
   Int_t fRandSamplMode{0};
   Bool_t fCharThres{false};
   Int_t fVertexMode{0};

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
