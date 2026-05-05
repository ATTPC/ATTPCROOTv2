#include "AtSiTask.h"

#include "AtAuxPad.h"
#include "AtPad.h"
#include "AtPadReference.h"
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

AtSiTask::AtSiTask(std::unique_ptr<AtPSASi> psa)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtSiEvent"), fSiEventArray(TClonesArray("AtSiEvent", 1)),
     fPSA(std::move(psa)), fIsPersistence(kFALSE)
{
   fSiMap = std::make_unique<AtSiMap>();
   TString dir = getenv("VMCWORKDIR");
   fSiMap->ParseXMLMap(dir + "scripts/rcnp_si_map.xml");
}

AtSiTask::AtSiTask(std::unique_ptr<AtPSASi> psa, std::unique_ptr<AtSiMap> simap)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtSiEvent"), fSiEventArray(TClonesArray("AtSiEvent", 1)),
     fPSA(std::move(psa)), fSiMap(std::move(simap)), fIsPersistence(kFALSE)
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
   int idx3{};
   int idx4{};

   for (auto &auxPadMapEntry : auxPadsMap) {
      auto auxPadName = auxPadMapEntry.first;
      auto auxPad = auxPadMapEntry.second;

      int cobo = std::stoi(auxPadName.substr(3, 2));
      int asad = std::stoi(auxPadName.substr(6, 1));
      int aget = std::stoi(auxPadName.substr(8, 1));
      int channel = std::stoi(auxPadName.substr(10, 2));

      AtPadReference padRef = {cobo, asad, aget, channel};

      int StripNum = fSiMap->GetPadNum(padRef);
      int SiFace = (StripNum / 128) % 2;
      int DetNum = (StripNum / 256) + 1;

      if (asad == 1) { // positive trace
         fPSA->SetPositivePolarity(true);
      }
      if (asad == 0) { // negative trace
         fPSA->SetPositivePolarity(false);
      }

      auto pseudoHits = fPSA->AnalyzePad(&auxPad);
      double traceCharge{};
      double maxADC{};
      int timestamp{};
      if (pseudoHits.size()) {
         traceCharge = pseudoHits[0]->GetTraceIntegral();
         maxADC = pseudoHits[0]->GetCharge();
         timestamp = pseudoHits[0]->GetTimeStamp();
      } else {
         continue;
      }

      if (DetNum == 1) {
         if (SiFace == 0) {
            siEvent->SetEFront1(idx1, traceCharge);
            siEvent->SetADCMaxFront1(idx1, maxADC);
            siEvent->SetStripFront1(idx1, StripNum);
            siEvent->SetTSFront1(idx1, timestamp);
            idx1++;
         } else if (SiFace == 1) {
            siEvent->SetEBack1(idx2, traceCharge);
            siEvent->SetADCMaxBack1(idx2, maxADC);
            siEvent->SetStripBack1(idx2, StripNum);
            siEvent->SetTSBack1(idx2, timestamp);
            idx2++;
         }
      } else if (DetNum == 2) {
         if (SiFace == 0) {
            siEvent->SetEFront2(idx3, traceCharge);
            siEvent->SetADCMaxFront2(idx3, maxADC);
            siEvent->SetStripFront2(idx3, StripNum);
            siEvent->SetTSFront2(idx3, timestamp);
            idx3++;
         } else if (SiFace == 1) {
            siEvent->SetEBack2(idx4, traceCharge);
            siEvent->SetADCMaxBack2(idx4, maxADC);
            siEvent->SetStripBack2(idx4, StripNum);
            siEvent->SetTSBack2(idx4, timestamp);
            idx4++;
         }
      }

      if ((idx1 >= 4) || (idx2 >= 4) || (idx3 >= 4) || (idx4 >= 4))
         break;
   }
   siEvent->SetMultiplicityFront1(idx1);
   siEvent->SetMultiplicityFront2(idx3);
   siEvent->SetMultiplicityBack1(idx2);
   siEvent->SetMultiplicityBack2(idx4);

   siEvent->BuildHits();

   LOG(debug) << "Finished running Si analysis";
}
