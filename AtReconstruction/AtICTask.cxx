#include "AtICTask.h"

#include "AtAuxPad.h"
#include "AtICEvent.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h>
#include <TObject.h> // for TObject

#include <algorithm>
#include <map>
#include <numeric>
#include <string>
#include <utility> // for move

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

AtICTask::AtICTask()
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtICEvent"), fICEventArray(TClonesArray("AtICEvent", 1)),
     fIsPersistence(kFALSE)
{
}

void AtICTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtICTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtICTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

InitStatus AtICTask::Init()
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

   ioMan->Register(fOutputBranchName, "AtTPC", &fICEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtICTask::Exec(Option_t *opt)
{
   if (fRawEventArray->GetEntriesFast() == 0) {
      LOG(debug) << "Skipping IC analysis because raw event array is empty";
      return;
   }

   fICEventArray.Clear("C");

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   if (!rawEvent || !rawEvent->IsGood())
      return;

   auto *ICEvent = dynamic_cast<AtICEvent *>(fICEventArray.ConstructedAt(0));

   if (!rawEvent->IsGood()) {
      LOG(debug) << "Event " << rawEvent->GetEventID() << " is not good, skipping IC analysis";
      return;
   }

   LOG(debug) << " --------------------> Staring IC analysis on event Number: " << rawEvent->GetEventID() << " with "
              << rawEvent->GetNumPads() << " valid pads";

   // Get the AtAuxPads that contain the IC data and get the trace integrals.
   const auto &traces = rawEvent->GetGenTraces();
   int i_channel = -1;
   for (auto &traceEntry : traces) {
      i_channel++;
      if (i_channel == 1) {
         const auto &trace_rawADC = traceEntry->GetRawADC();
         auto maxIt = std::max_element(trace_rawADC.begin() + 20, trace_rawADC.end() - 12);

         ICEvent->SetRawADC(*maxIt);

         double baseline = std::accumulate(trace_rawADC.begin(), trace_rawADC.begin() + 20, 0.0) / 20.0;
         if (baseline <= 0.) {
            LOG(debug) << "    ---> IC analysis : error baseline";
            return;
         }

         ICEvent->SetPedestalSubtracted(kTRUE);
         ICEvent->SetADC(*maxIt - baseline);
      }
   }

   LOG(debug) << "Finished running IC analysis";
}
