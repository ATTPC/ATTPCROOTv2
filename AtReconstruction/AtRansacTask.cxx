#include "AtRansac.h"
#include "AtRansacTask.h"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>

ClassImp(AtRansacTask);

AtRansacTask::AtRansacTask() : fInputBranchName("AtEventH"), fOutputBranchName("AtRansac")
{

   kIsPersistence = kFALSE;
   kIsFullMode = kFALSE;
   kIsReprocess = kFALSE;

   fRANSACModel = pcl::SACMODEL_LINE;
   fRANSACThreshold = 5.0;
   fMinHitsLine = 5;
   fNumItera = 500;
   fRANSACAlg = 0;
   fRandSamplMode = 0;
   fCharThres = 0;
   fVertexMode = 0;
}

AtRansacTask::~AtRansacTask() {}

void AtRansacTask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}
void AtRansacTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtRansacTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

void AtRansacTask::SetModelType(int model)
{
   fRANSACModel = model;
}
void AtRansacTask::SetDistanceThreshold(Float_t threshold)
{
   fRANSACThreshold = threshold;
}
void AtRansacTask::SetFullMode()
{
   kIsFullMode = kTRUE;
}
void AtRansacTask::SetMinHitsLine(Int_t nhits)
{
   fMinHitsLine = nhits;
}
void AtRansacTask::SetTiltAngle(Double_t val)
{
   fTiltAngle = val;
}
void AtRansacTask::SetNumItera(Int_t niterations)
{
   fNumItera = niterations;
}
void AtRansacTask::SetAlgorithm(Int_t val)
{
   fRANSACAlg = val;
}
void AtRansacTask::SetRanSamMode(Int_t mode)
{
   fRandSamplMode = mode;
};
void AtRansacTask::SetIsReprocess(Bool_t value)
{
   kIsReprocess = value;
}
void AtRansacTask::SetChargeThreshold(Double_t value)
{
   fCharThres = value;
}
void AtRansacTask::SetVertexMode(Int_t value)
{
   fVertexMode = value;
}

InitStatus AtRansacTask::Init()
{

   if (fRANSACAlg == 0)
      fRansacArray = new TClonesArray("AtRANSACN::AtRansac");
   else if (fRANSACAlg == 1)
      fRansacArray = new TClonesArray("AtRansacMod");
   else if (fRANSACAlg == 2)
      fRansacArray = new TClonesArray("AtMlesacMod");
   else if (fRANSACAlg == 3)
      fRansacArray = new TClonesArray("AtLmedsMod");
   else {
      LOG(error) << "Cannot find Ransac algorithm!";
      return kERROR;
   }

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventArray = (TClonesArray *)ioMan->GetObject(fInputBranchName);
   if (fEventArray == nullptr) {
      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", fRansacArray, kIsPersistence);

   if (kIsReprocess) {
      ioMan->Register("AtEventH", "AtTPC", fEventArray, kIsPersistence);
   }

   return kSUCCESS;
}

void AtRansacTask::Exec(Option_t *opt)
{

   fRansacArray->Delete();

   if (fEventArray->GetEntriesFast() == 0)
      return;

   fEvent = (AtEvent *)fEventArray->At(0);

   LOG(debug) << "Running RANSAC with " << fEvent->GetNumHits() << " hits.";

   if (fRANSACAlg == 0) {
      LOG(debug) << "Running RANSAC algorithm AtRANSACN::AtRansac";
      AtRANSACN::AtRansac *Ransac = (AtRANSACN::AtRansac *)new ((*fRansacArray)[0]) AtRANSACN::AtRansac();
      Ransac->SetTiltAngle(fTiltAngle);
      Ransac->SetModelType(fRANSACModel);
      Ransac->SetDistanceThreshold(fRANSACThreshold);
      Ransac->SetMinHitsLine(fMinHitsLine);
      if (kIsFullMode)
         Ransac->CalcRANSACFull(fEvent);
      else
         Ransac->CalcRANSAC(fEvent);
   }

   if (fRANSACAlg == 1) {
      LOG(debug) << "Running RANSAC algorithm AtRansacMod";
      AtRansacMod *Rantest = (AtRansacMod *)new ((*fRansacArray)[0]) AtRansacMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcRANSACMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }

   if (fRANSACAlg == 2) {
      LOG(debug) << "Running RANSAC algorithm AtMlesacMod";
      AtMlesacMod *Rantest = (AtMlesacMod *)new ((*fRansacArray)[0]) AtMlesacMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcMlesacMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }

   if (fRANSACAlg == 3) {
      LOG(debug) << "Running RANSAC algorithm AtLmedsMod";
      AtLmedsMod *Rantest = (AtLmedsMod *)new ((*fRansacArray)[0]) AtLmedsMod();
      Rantest->SetDistanceThreshold(fRANSACThreshold);
      Rantest->SetMinHitsLine(fMinHitsLine);
      Rantest->SetNumItera(fNumItera);
      Rantest->SetRanSamMode(fRandSamplMode);
      Rantest->CalcLmedsMod(fEvent);
      Rantest->SetChargeThres(fCharThres);
      Rantest->SetVertexMod(fVertexMode);
   }
}
