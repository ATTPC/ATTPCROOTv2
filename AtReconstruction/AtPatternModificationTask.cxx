#include "AtPatternModificationTask.h"

#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"

#include <FairRootManager.h>

ClassImp(AtPatternModificationTask);

AtPatternModificationTask::AtPatternModificationTask(
   std::vector<std::unique_ptr<AtPatternModification>> patternModifications)
   : fInputBranchName("AtPatternEvent"), fOutputBranchName("AtPatternEventModified"), fRawEventBranchName("AtRawEvent"),
     fEventBranchName("AtEvent"), fPatternEventModifiedArray(TClonesArray("AtPatternEvent", 1))
{
   fPatternModifications = std::move(patternModifications);
}

void AtPatternModificationTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtPatternModificationTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

void AtPatternModificationTask::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtPatternModificationTask::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}

void AtPatternModificationTask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}

InitStatus AtPatternModificationTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
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

   ioMan->Register(fOutputBranchName, "AtTPC", &fPatternEventModifiedArray, kIsPersistence);

   return kSUCCESS;
}

void AtPatternModificationTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   LOG(info) << "Applying pattern modifications to pattern event " << fEventCnt;

   AtPatternEvent *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
   AtPatternEvent *patternEventModified = dynamic_cast<AtPatternEvent *>(fPatternEventModifiedArray.ConstructedAt(0));

   *patternEventModified = *patternEvent;

   AtRawEvent *rawEvent{nullptr};
   if (fRawEventArray != nullptr)
      rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

   AtEvent *event{nullptr};
   if (fEventArray != nullptr)
      event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   for (int i = 0; i < fPatternModifications.size(); i++)
      fPatternModifications[i]->ModifyPatternEvent(patternEventModified, rawEvent, event);

   ++fEventCnt;
}
