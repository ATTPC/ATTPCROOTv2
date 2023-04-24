#include "AtDataReductionTask.h"

#include "AtBaseEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>

ClassImp(AtDataReductionTask);

void AtDataReductionTask::SetReductionFunction(std::function<bool(AtBaseEvent *)> func)
{
   // Bind the reduction function to the passed function, having it call that function with the current
   // AtRawEvent in the pointer this->fRawEvent.
   fReductionFunc = [this, func]() { return func(fEvent); };
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
      LOG(fatal) << "Cannot find TClonesArray in branch " << fInputBranchName << "!";
      return kFATAL;
   }

   return kSUCCESS;
}

void AtDataReductionTask::Exec(Option_t *opt)
{
   // Get raw event
   if (fInputEventArray->GetEntriesFast() == 0)
      return;
   fEvent = dynamic_cast<AtBaseEvent *>(fInputEventArray->At(0));

   // If we should skip this event mark bad and don't fill tree
   if (fReductionFunc())
      LOG(info) << "Keeping event " << fEvent->GetEventID() << " at " << FairRootManager::Instance()->GetEntryNr();
   else {

      LOG(info) << "Skipping event " << fEvent->GetEventID() << " at " << FairRootManager::Instance()->GetEntryNr();

      FairRootManager *ioMan = FairRootManager::Instance();
      for (auto name : fOutputBranchs) {
         auto b = dynamic_cast<TClonesArray *>(ioMan->GetObject(name));
         if (fInputEventArray == nullptr)
            LOG(fatal) << "Cannot find branch " << name << "!";

         auto e = dynamic_cast<AtBaseEvent *>(b->At(0));
         if (e == nullptr) {
            LOG(error) << "Not setting " << name;
            continue;
         }
         e->SetIsGood(false);
      }
      FairRunAna::Instance()->MarkFill(false);
   }
}
