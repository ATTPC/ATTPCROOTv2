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

void AtDataReductionTask::SetReductionFunction(std::function<bool(AtRawEvent *)> func)
{
   // Bind the reduction function to the passed function, having it call that function with the current
   // AtRawEvent in the pointer this->fRawEvent.
   fReductionFunc = [this, func]() { return func(fRawEvent); };
}

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

   // If we should skip this event mark bad and don't fill tree
   if (fReductionFunc())
      LOG(info) << "Keeping event " << fRawEvent->GetEventID() << " with " << fRawEvent->GetNumPads() << " pads";
   else {
      fRawEvent->SetIsGood(false);
      FairRunAna::Instance()->MarkFill(false);
   }
}
