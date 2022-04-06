#include "AtUnpackTask.h"

#include <FairTask.h>
#include <TObject.h>
#include <fairlogger/Logger.h>
#include <utility>

#include <FairRootManager.h>
#include "AtRawEvent.h"
#include "TClonesArray.h"

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
   if (!fUnpacker->IsLastEvent())
      fUnpacker->FillRawEvent(*rawEvent);
   else {
      LOG(warn) << "Hit last event at: " << fUnpacker->GetNextEventID();
      fRawEvent->SetIsGood(false);
   }
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
