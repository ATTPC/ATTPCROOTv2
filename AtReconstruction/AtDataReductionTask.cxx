#include "AtDataReductionTask.h"

#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>

#include <memory>

ClassImp(AtDataReductionTask);

AtDataReductionTask::AtDataReductionTask() : fReductionFunction(nullptr), fInputBranchName("AtRawEvent") {}

AtDataReductionTask::~AtDataReductionTask() = default;

InitStatus AtDataReductionTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fInputEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fInputEventArray == nullptr) {
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
   // if ((*reduceFunc)(fRawEvent))
   if (fReductionFunction(fRawEvent))
      LOG(info) << "Keeping event " << fRawEvent->GetEventID() << " with " << fRawEvent->GetNumPads() << " pads";
   else {
      fRawEvent->SetIsGood(false);
      FairRunAna::Instance()->MarkFill(false);
   }
}
