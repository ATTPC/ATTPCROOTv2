#include "AtPSAtask.h"

// FairRoot Classes
#include "FairLogger.h"

// Root Classes
#include "AtPSA.h"
#include "AtRawEvent.h"
#include "AtEvent.h"

// AtTPCRoot Classes
#include "TClonesArray.h"

// stdlib headers
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtPSAtask);

AtPSAtask::AtPSAtask(AtPSA *psa)
   : fInputBranchName("AtRawEvent"), fOutputBranchName("AtEventH"), fPSA(psa), fIsPersistence(false)
{
   fEventHArray = new TClonesArray("AtEvent");
}

AtPSAtask::~AtPSAtask() {}

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

InitStatus AtPSAtask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(ERROR) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = (TClonesArray *)ioMan->GetObject(fInputBranchName);
   if (fRawEventArray == 0) {
      LOG(ERROR) << "Cannot find AtRawEvent array in branch " << fInputBranchName << "!";
      return kERROR;
   }
   // Retrieving simulated points, if available
   fMCPointArray = (TClonesArray *)ioMan->GetObject("AtTpcPoint");
   if (fMCPointArray != 0) {
      LOG(INFO) << " Simulated points found (simulation analysis) ";
      fPSA->SetSimulatedEvent(fMCPointArray);
   } else if (fMCPointArray != 0) {
      LOG(INFO) << " Simulated points not found (experimental data analysis) ";
   }

   ioMan->Register(fOutputBranchName, "AtTPC", fEventHArray, fIsPersistence);

   fPSA->Init();

   return kSUCCESS;
}

void AtPSAtask::Exec(Option_t *opt)
{
   fEventHArray->Delete();

   if (fRawEventArray->GetEntriesFast() == 0)
      return;

   AtRawEvent *rawEvent = (AtRawEvent *)fRawEventArray->At(0);
   AtEvent *event = (AtEvent *)new ((*fEventHArray)[0]) AtEvent();

   event->SetIsGood(rawEvent->IsGood());
   event->SetEventID(rawEvent->GetEventID());
   event->SetTimestamp(rawEvent->GetTimestamp());
   event->SetIsExtGate(rawEvent->GetIsExtGate());

   if (!rawEvent->IsGood())
      return;
   LOG(debug) << " Event Number :  " << rawEvent->GetEventID() << " Valid pads : " << rawEvent->GetNumPads();

   fPSA->Analyze(rawEvent, event);
}
