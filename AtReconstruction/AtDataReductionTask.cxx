#include "AtDataReductionTask.h"

#include <FairTask.h>
#include <TObject.h>
#include <fairlogger/Logger.h>
#include <memory>

#include "AtRawEvent.h"
#include "FairRunAna.h"
#include "FairRootManager.h"
#include "TClonesArray.h"

AtDataReductionTask::AtDataReductionTask() : reduceFunc(nullptr), fInputBranchName("AtRawEvent") {}

AtDataReductionTask::~AtDataReductionTask() {}

InitStatus AtDataReductionTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fInputEventArray = (TClonesArray *)ioMan->GetObject(fInputBranchName);
   if (fInputEventArray == 0) {
      LOG(fatal) << "Cannot find AtRawEvent array in branch " << fInputBranchName << "!";
      return kFATAL;
   }

   return kSUCCESS;
}

void AtDataReductionTask::Exec(Option_t *opt)
{
   // Get raw event
   if (fInputEventArray->GetEntriesFast() == 0)
      return;
   fRawEvent = dynamic_cast<AtRawEvent *>(fInputEventArray->At(0));

   // If we should skip this event
   if ((*reduceFunc)(fRawEvent))
      LOG(info) << "Keeping event " << fRawEvent->GetEventID() << " with " << fRawEvent->GetNumPads() << " pads";
   else {
      fRawEvent->SetIsGood(false);
      FairRunAna::Instance()->MarkFill(false);
   }
}
