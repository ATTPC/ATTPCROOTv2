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
{
   fPSA = psa;
   fIsPersistence = kFALSE;
   fEventHArray = new TClonesArray("AtEvent");
}

AtPSAtask::~AtPSAtask() {}

void AtPSAtask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

InitStatus AtPSAtask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(ERROR) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = (TClonesArray *)ioMan->GetObject("AtRawEvent");
   if (fRawEventArray == 0) {
      LOG(ERROR) << "Cannot find AtRawEvent array!";
      return kERROR;
   }
   // Retrieving simulated points, if available
   fMCPointArray = (TClonesArray *)ioMan->GetObject("AtTpcPoint");
   if (fMCPointArray != 0) {
      LOG(INFO) << " Simulated points found (simulation analysis) ";
   } else if (fMCPointArray != 0) {
      LOG(INFO) << " Simulated points not found (experimental data analysis) ";
   }

   ioMan->Register("AtEventH", "AtTPC", fEventHArray, fIsPersistence);

   return kSUCCESS;
}

void AtPSAtask::Exec(Option_t *opt)
{
   fEventHArray->Delete();

   if (fRawEventArray->GetEntriesFast() == 0)
      return;

   AtRawEvent *rawEvent = (AtRawEvent *)fRawEventArray->At(0);

   std::cout << " Event Number :  " << rawEvent->GetEventID() << " Valid pads : " << rawEvent->GetNumPads()
             << std::endl;

   AtEvent *event = (AtEvent *)new ((*fEventHArray)[0]) AtEvent();

   if (!(rawEvent->IsGood())) {
      event->SetIsGood(kFALSE);
   } else {
      if (fMCPointArray != 0) {
         std::cout << " point array size " << fMCPointArray->GetEntries() << "\n";
         fPSA->SetSimulatedEvent(fMCPointArray);
      }
      // Analyze the event
      fPSA->Analyze(rawEvent, event);

      // Copy parameters from raw event
      event->SetIsGood(kTRUE);
      event->SetEventID(rawEvent->GetEventID());
      event->SetTimestamp(rawEvent->GetTimestamp());
      event->SetIsExtGate(rawEvent->GetIsExtGate());
   }

   // std::cout << "PSA events  "<<event->GetNumHits()  << '\n';
}
