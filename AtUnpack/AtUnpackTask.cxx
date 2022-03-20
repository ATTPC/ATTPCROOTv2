#include "AtUnpackTask.h"

#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRunAna.h"

#include "AtRawEvent.h"
#include "TClonesArray.h"

#include <iostream>

ClassImp(AtUnpackTask);

AtUnpackTask::AtUnpackTask(unpackerPtr unpacker)
   : fRawEvent(new AtRawEvent()), fUnpacker(std::move(unpacker)), fOutputEventArray(TClonesArray("AtRawEvent", 1))
{
}

InitStatus AtUnpackTask::Init()
{
   if (FairRootManager::Instance() == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fUnpacker->Init();

   FairRootManager::Instance()->Register(fOuputBranchName.data(), "AtTpc", &fOutputEventArray, fIsPersistent);

   return kSUCCESS;
}

void AtUnpackTask::SetParContainers() {}

void AtUnpackTask::Exec(Option_t *opt)
{
   fOutputEventArray.Clear("C");
   auto rawEvent = dynamic_cast<AtRawEvent *>(fOutputEventArray.ConstructedAt(0));

   // Hacked solution to the fact that FinishEvent() is not normally called with how
   // we unpack runs.
   if (fUnpacker->GetNextEventID() < fUnpacker->GetNumEvents())
      fUnpacker->FillRawEvent(*rawEvent);
   else
      fRawEvent->SetIsGood(false);
}

void AtUnpackTask::FinishEvent()
{
   // Note this is only called if we are not running a "DummyRun"
   // ie FairRoot thinks there is some sort of source. This is, in general
   // not true so will never be called.
   if (fUnpacker->IsLastEvent()) {
      LOG(info) << "Unpacked last event. Terminating run.";
      FairRootManager::Instance()->SetFinishRun();
   }
}
