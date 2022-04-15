#include "AtFilterTask.h"

#include <FairRootManager.h>
#include <FairTask.h>
#include <fairlogger/Logger.h>
// stdlib headers
#include <iostream>
#include <map>
#include <memory>
#include <utility>
#include <vector>

// Root Classes
#include "AtFilter.h"
#include "AtRawEvent.h"
// AtTPCRoot Classes
#include <TClonesArray.h>
#include "AtAuxPad.h"

class AtPad;

AtFilterTask::AtFilterTask(AtFilter *filter)
   : fOutputEventArray(new TClonesArray("AtRawEvent")), fFilter(filter), fIsPersistent(false), fFilterAux(false),
     fInputBranchName("AtRawEvent"), fOutputBranchName("AtRawEventFiltered")
{
}

AtFilterTask::~AtFilterTask() = default;

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
   fInputEventArray = dynamic_cast<TClonesArray *>(ioManager->GetObject(fInputBranchName));
   if (fInputEventArray == nullptr) {
      LOG(fatal) << "AtFilterTask: Cannot find AtRawEvent array!";
      return kFATAL;
   }

   // Set the raw event array, and new output event array
   ioManager->Register(fOutputBranchName, "AtTPC", fOutputEventArray, fIsPersistent);

   fFilter->Init();

   return kSUCCESS;
}

void AtFilterTask::Exec(Option_t *opt)
{
   fOutputEventArray->Delete();

   if (fInputEventArray->GetEntriesFast() == 0)
      return;

   auto *rawEvent = dynamic_cast<AtRawEvent *>(fInputEventArray->At(0));
   auto *filteredEvent = (AtRawEvent *)new ((*fOutputEventArray)[0]) AtRawEvent(*rawEvent);

   if (!rawEvent->IsGood())
      return;

   fFilter->InitEvent(filteredEvent);

   if (fFilterAux)
      for (auto &padIt : filteredEvent->fAuxPadMap) {
         AtPad *pad = &(padIt.second);
         fFilter->Filter(pad);
      }

   for (auto &pad : filteredEvent->fPadList)
      fFilter->Filter(pad.get());

   auto isGood = filteredEvent->IsGood() && fFilter->IsGoodEvent();
   filteredEvent->SetIsGood(isGood);
}
