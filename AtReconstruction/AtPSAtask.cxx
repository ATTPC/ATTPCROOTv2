#include "AtPSAtask.h"

// FairRoot Classes
#include <FairLogger.h>

// Root Classes
#include "AtEvent.h"
#include "AtPSA.h"
#include "AtRawEvent.h"

#include <FairRootManager.h> // for FairRootManager

#include <TObject.h> // for TObject

#include <map> // for allocator, operator!=, _Rb_tree_const_i...

// AtTPCRoot Classes
#include <TClonesArray.h>

/*
#ifdef _OPENMP
#include <omp.h>
#endif
*/

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

ClassImp(AtPSAtask);

AtPSAtask::AtPSAtask(AtPSA *psa) : AtPSAtask(psa->Clone()) {}
AtPSAtask::AtPSAtask(std::unique_ptr<AtPSA> psa)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtEventH"), fSimulatedPointBranchName("AtTpcPoint"),
     fEventArray(TClonesArray("AtEvent", 1)), fPSA(std::move(psa)), fIsPersistence(false)
{
}

void AtPSAtask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtPSAtask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtPSAtask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}
void AtPSAtask::SetSimlulatedPointBranch(TString branchName)
{
   fSimulatedPointBranchName = branchName;
}
InitStatus AtPSAtask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(ERROR) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fRawEventArray == nullptr) {
      LOG(ERROR) << "Cannot find AtRawEvent array in branch " << fInputBranchName << "!";
      return kERROR;
   }

   fPSA->Init();

   // Retrieving simulated points, if available
   fMCPointArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fSimulatedPointBranchName));
   if (fMCPointArray != nullptr) {
      LOG(INFO) << " Simulated points found (simulation analysis) in branch " << fSimulatedPointBranchName;
      fPSA->SetSimulatedEvent(fMCPointArray);
   } else {
      LOG(INFO) << " Simulated points not found (experimental data analysis) looking at branch "
                << fSimulatedPointBranchName;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtPSAtask::Exec(Option_t *opt)
{
   fEventArray.Clear();

   if (fRawEventArray->GetEntriesFast() == 0) {
      LOG(debug) << "Skipping PSA because raw event array is empty";
      return;
   }

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   auto *event = dynamic_cast<AtEvent *>(new (fEventArray[0]) AtEvent(*rawEvent));

   if (!rawEvent->IsGood()) {
      LOG(debug) << "Event " << rawEvent->GetEventID() << " is not good, skipping PSA";
      return;
   }

   LOG(debug) << "Staring PSA on event Number: " << rawEvent->GetEventID() << " with " << rawEvent->GetNumPads()
              << " valid pads";

   fPSA->Analyze(rawEvent, event);

   LOG(debug) << "Finished running PSA";
}
