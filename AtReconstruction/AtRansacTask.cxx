#include "AtRansacTask.h"

#include "AtEstimatorMethods.h"
#include "AtEvent.h" // for AtEvent
#include "AtPatternEvent.h"
#include "AtPatternTypes.h"
#include "AtSampleConsensus.h"
#include "AtSampleMethods.h"

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>      // for TObject

#include <memory> // for allocator

ClassImp(AtRansacTask);

AtRansacTask::AtRansacTask()
   : fInputBranchName("AtEventH"), fOutputBranchName("AtPatternEvent"), kIsPersistence(kFALSE), kIsReprocess(kFALSE),
     fPatternEventArray("AtPatternEvent", 1)
{
}

AtRansacTask::~AtRansacTask() = default;

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
void AtRansacTask::SetMinHitsLine(Int_t nhits)
{
   fMinHitsLine = nhits;
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
void AtRansacTask::SetInputBranchName(TString inputName)
{
   fInputBranchName = inputName;
}
void AtRansacTask::SetOutputBranchName(TString outputName)
{
   fOutputBranchName = outputName;
}

InitStatus AtRansacTask::Init()
{

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fEventArray == nullptr) {

      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fPatternEventArray, kIsPersistence);

   if (kIsReprocess) {

      ioMan->Register("AtEventH", "AtTPC", fEventArray, kIsPersistence);
   }

   return kSUCCESS;
}

void AtRansacTask::Exec(Option_t *opt)
{

   if (fEventArray->GetEntriesFast() == 0)
      return;

   fEvent = dynamic_cast<AtEvent *>(fEventArray->At(0));

   LOG(debug) << "Running RANSAC with " << fEvent->GetNumHits() << " hits.";
   LOG(debug) << "Running Unified RANSAC";

   auto sampleMethod = static_cast<RandomSample::SampleMethod>(fRandSamplMode);
   auto patternType = AtPatterns::PatternType::kLine;
   auto estimator = SampleConsensus::Estimators::kChi2;
   if (fRANSACAlg == 2)
      estimator = SampleConsensus::Estimators::kMLESAC;
   if (fRANSACAlg == 3)
      estimator = SampleConsensus::Estimators::kLMedS;
   if (fRANSACAlg == 4)
      estimator = SampleConsensus::Estimators::kWRANSAC;
   SampleConsensus::AtSampleConsensus ransac(estimator, patternType, sampleMethod);

   ransac.SetDistanceThreshold(fRANSACThreshold);
   ransac.SetMinHitsPattern(fMinHitsLine);
   ransac.SetNumIterations(fNumItera);
   ransac.SetChargeThreshold(fChargeThres);
   fPatternEventArray.Delete();
   auto patternEvent = ransac.Solve(fEvent);
   new (fPatternEventArray[0]) AtPatternEvent(patternEvent);
}
