#include "AtFilterTask.h"

// FairRoot Classes
#include "FairLogger.h"

// Root Classes
#include "AtFilter.h"
#include "AtRawEvent.h"

// AtTPCRoot Classes
#include "TClonesArray.h"

// stdlib headers
#include <iostream>

AtFilterTask::AtFilterTask(AtFilter *filter) : fFilter(filter), fIsPersistent(false), fFilterAux(false)
{
   fOutputEventArray = new TClonesArray("AtRawEvent");
}

AtFilterTask::~AtFilterTask() {}

void AtFilterTask::SetPersistence(Bool_t value)
{
   fIsPersistent = value;
}
void AtFilterTask::SetFilterAux(Bool_t value)
{
   fFilterAux = value;
}

InitStatus AtFilterTask::Init()
{
   FairRootManager *ioManager = FairRootManager::Instance();

   if (ioManager == nullptr) {
      LOG(ERROR) << "Cannot find RootManager!" << std::endl;
      return kERROR;
   }

   // Get the old data from the io manager
   fInputEventArray = (TClonesArray *)ioManager->GetObject("AtRawEvent");
   if (fInputEventArray == nullptr) {
      LOG(ERROR) << "AtFilterTask: Cannot find AtRawEvent array!";
      return kERROR;
   }

   // Set the raw event array, and new output event array
   ioManager->Register("AtRawEventFiltered", "AtTPC", fOutputEventArray, fIsPersistent);

   fFilter->Init();

   return kSUCCESS;
}

void AtFilterTask::Exec(Option_t *opt)
{
   fOutputEventArray->Delete();

   if (fInputEventArray->GetEntriesFast() == 0)
      return;

   AtRawEvent *rawEvent = (AtRawEvent *)fInputEventArray->At(0);
   if (!rawEvent->IsGood())
      return;
   AtRawEvent *filteredEvent = (AtRawEvent *)new ((*fOutputEventArray)[0]) AtRawEvent(rawEvent);

   fFilter->InitEvent(filteredEvent);

   if (fFilterAux)
      for (auto &padIt : filteredEvent->GetAuxPads())
         fFilter->Filter(&(padIt.second));

   for (auto &pad : filteredEvent->GetPads())
      fFilter->Filter(&pad);

   auto isGood = filteredEvent->IsGood() && fFilter->IsGoodEvent();
   filteredEvent->SetIsGood(isGood);
}
