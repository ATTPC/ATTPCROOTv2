#include "AtSiTask.h"

#include "AtAuxPad.h"
#include "AtPad.h"
#include "AtRawEvent.h"
#include "AtSiEvent.h"

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

AtSiTask::AtSiTask(std::unique_ptr<AtPSA> psa)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtSiEvent"), fSiEventArray(TClonesArray("AtSiEvent", 1)),
     fPSA(std::move(psa)), fIsPersistence(kFALSE)
{
}

void AtSiTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtSiTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtSiTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

InitStatus AtSiTask::Init()
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

   ioMan->Register(fOutputBranchName, "AtTPC", &fSiEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtSiTask::Exec(Option_t *opt)
{
   if (fRawEventArray->GetEntriesFast() == 0) {
      LOG(debug) << "Skipping Si analysis because raw event array is empty";
      return;
   }

   fSiEventArray.Clear("C");

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   auto *siEvent = dynamic_cast<AtSiEvent *>(fSiEventArray.ConstructedAt(0));
   *siEvent = *rawEvent;

   if (!rawEvent->IsGood()) {
      LOG(debug) << "Event " << rawEvent->GetEventID() << " is not good, skipping Si analysis";
      return;
   }

   LOG(debug) << "Staring Si analysis on event Number: " << rawEvent->GetEventID() << " with " << rawEvent->GetNumPads()
              << " valid pads";

   // Get the AtAuxPads that contain the Si data and get the trace integrals.
   auto &auxPadsMap = rawEvent->GetAuxPads();
   int idx1{};
   int idx2{};
   for (auto &auxPadMapEntry : auxPadsMap) {
      auto auxPadName = auxPadMapEntry.first;
      auto auxPad = auxPadMapEntry.second;

      int cobo = std::stoi(auxPadName.substr(3, 2));
      int asad = std::stoi(auxPadName.substr(6, 1));
      int aget = std::stoi(auxPadName.substr(8, 1));
      int channel = std::stoi(auxPadName.substr(10, 2));

      // Focus on positive traces for now.
      if (asad == 0)
         continue;

      auto pseudoHits = fPSA->AnalyzePad(&auxPad);
      double traceCharge{};
      double maxADC{};
      if (pseudoHits.size()) {
         traceCharge = pseudoHits[0]->GetTraceIntegral();
         maxADC = pseudoHits[0]->GetCharge();
      } else
         continue;

      if (channel < 11 || (22 < channel && channel < 45) || 56 < channel) {
         if (channel % 2 == 0) {
            siEvent->SetEFront1(idx1, traceCharge);
            siEvent->SetADCMaxFront1(idx1++, maxADC);
         } else {
            siEvent->SetEFront2(idx2, traceCharge);
            siEvent->SetADCMaxFront2(idx2++, maxADC);
         }
      } else if ((11 < channel && channel < 22) || (45 < channel && channel < 56)) {
         if (channel % 2 == 0) {
            siEvent->SetEFront2(idx2, traceCharge);
            siEvent->SetADCMaxFront2(idx2++, maxADC);
         } else {
            siEvent->SetEFront1(idx1, traceCharge);
            siEvent->SetADCMaxFront1(idx1++, maxADC);
         }
      }
      if (idx1 >= 4 || idx2 >= 4)
         break;
   }
   siEvent->SetMultiplicityFront1(idx1);
   siEvent->SetMultiplicityFront2(idx2);

   LOG(debug) << "Finished running Si analysis";
}
