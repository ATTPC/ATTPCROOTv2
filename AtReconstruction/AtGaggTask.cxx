#include "AtGaggTask.h"

#include "AtGaggEvent.h"
#include "AtGenericTrace.h"
#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h>
#include <TObject.h> // for TObject

#include <map>
#include <string>
#include <utility> // for move

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

AtGaggTask::AtGaggTask(std::unique_ptr<AtPSASi> psa)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtGaggEvent"), fGaggEventArray(TClonesArray("AtGaggEvent", 1)),
     fPSA(std::move(psa)), fIsPersistence(kFALSE)
{
}

void AtGaggTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtGaggTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtGaggTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

InitStatus AtGaggTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fRawEventArray == nullptr) {
      LOG(error) << "Cannot find AtRawEvent array in branch " << fInputBranchName << "!";
      return kERROR;
   }

   fPSA->Init();

   ioMan->Register(fOutputBranchName, "AtTPC", &fGaggEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtGaggTask::Exec(Option_t *opt)
{
   if (fRawEventArray->GetEntriesFast() == 0) {
      LOG(debug) << "Skipping Gagg analysis because raw event array is empty";
      return;
   }

   fGaggEventArray.Clear("C");

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   auto *gaggEvent = dynamic_cast<AtGaggEvent *>(fGaggEventArray.ConstructedAt(0));
   *gaggEvent = *rawEvent;

   if (!rawEvent->IsGood()) {
      LOG(debug) << "Event " << rawEvent->GetEventID() << " is not good, skipping GAGG analysis";
      return;
   }

   LOG(info) << "Staring GAGG analysis on event Number: " << rawEvent->GetEventID() << " with "
             << rawEvent->GetNumPads() << " valid pads";

   // Get the AtGenTrace that contain the GAGG data and get the trace integrals.
   auto &genTraces = rawEvent->GetGenTraces();
   int idx1{};
   int idx2{};
   for (auto &genTrace : genTraces) {

      auto pseudoHits = fPSA->AnalyzeGenTrace(genTrace.get());
      double traceCharge{};
      double maxADC{};
      if (pseudoHits.size()) {
         traceCharge = pseudoHits[0]->GetTraceIntegral();
         maxADC = pseudoHits[0]->GetCharge();
      } else
         continue;

      if (idx1 < 25) {
         gaggEvent->SetE1(idx1, traceCharge);
         gaggEvent->SetADCMax1(idx1++, maxADC);
      } else if (idx2 < 16) {
         gaggEvent->SetE2(idx2, traceCharge);
         gaggEvent->SetADCMax2(idx2++, maxADC);
      }

      if (idx1 >= 25 && idx2 >= 16)
         break;
   }
   gaggEvent->SetMultiplicity1(idx1);
   gaggEvent->SetMultiplicity2(idx2);

   LOG(debug) << "Finished running GAGG analysis";
}
