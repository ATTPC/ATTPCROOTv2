#include "AtPatternModificationTask.h"

#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"

#include <FairRootManager.h>

ClassImp(AtPatternModificationTask);

AtPatternModificationTask::AtPatternModificationTask()
   : fPatternEventBranchName("AtPatternEvent"), fRawEventBranchName("AtRawEvent"), fEventBranchName("AtEvent")
{
}

void AtPatternModificationTask::SetPatternEventBranch(TString branchName)
{
   fPatternEventBranchName = branchName;
}

void AtPatternModificationTask::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtPatternModificationTask::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}

InitStatus AtPatternModificationTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fPatternEventBranchName));
   if (fPatternEventArray == nullptr) {
      LOG(error) << "Cannot find AtPatternEvent array!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray == nullptr) {
      LOG(info) << "AtRawEvent branch name was not set. No AtRawEvent will be passed to the AtPatternModifications.";
   }

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fEventBranchName));
   if (fEventArray == nullptr) {
      LOG(info) << "AtEvent branch name was not set. No AtEvent will be passed to the AtPatternModifications.";
   }

   return kSUCCESS;
}

void AtPatternModificationTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   LOG(info) << " AtPatternModificationTask::Exec() : Applying pattern modifications to pattern event " << fEventCnt;

   AtPatternEvent *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));

   AtRawEvent *rawEvent{nullptr};
   if (fRawEventArray != nullptr)
      rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

   AtEvent *event{nullptr};
   if (fEventArray != nullptr)
      event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   for (int i = 0; i < fPatternModifications.size(); i++)
      fPatternModifications[i]->ModifyPatternEvent(patternEvent, rawEvent, event);

   ++fEventCnt;
}
