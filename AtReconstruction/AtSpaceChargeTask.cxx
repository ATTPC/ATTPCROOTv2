#include "AtSpaceChargeTask.h"
#include "AtSpaceChargeModel.h"

#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRunAna.h"

#include "AtRawEvent.h"
#include "AtEvent.h"
#include "TClonesArray.h"

#include <iostream>

using XYZPoint = ROOT::Math::XYZPoint;

ClassImp(AtSpaceChargeTask);

AtSpaceChargeTask::AtSpaceChargeTask(SCModelPtr &&SCModel)
   : fSCModel(std::move(SCModel)), fOutputEventArray(TClonesArray("AtEvent", 1)), fInputEventArray(nullptr)
{
}

InitStatus AtSpaceChargeTask::Init()
{
   if (FairRootManager::Instance() == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fInputEventArray = (TClonesArray *)FairRootManager::Instance()->GetObject(fInputBranchName.data());
   if (fInputEventArray == nullptr) {
      LOG(fatal) << "SpaceChargeTask: Cannot find AtEvent array!";
      return kFATAL;
   }

   // fSCModel->Init();

   FairRootManager::Instance()->Register(fOuputBranchName.data(), "AtTpc", &fOutputEventArray, fIsPersistent);

   return kSUCCESS;
}

void AtSpaceChargeTask::SetParContainers() {}

void AtSpaceChargeTask::Exec(Option_t *opt)
{
   fOutputEventArray.Clear("C");

   if (fInputEventArray->GetEntriesFast() == 0)
      return;

   auto inputEvent = dynamic_cast<AtEvent *>(fInputEventArray->At(0));
   auto outputEvent = dynamic_cast<AtEvent *>(fOutputEventArray.ConstructedAt(0));

   for (auto &inHit : inputEvent->GetHitArray()) {

      XYZPoint newPosition;
      newPosition = fSCModel->DirectCorrection(inHit.GetPosition());
      auto newHit = outputEvent->AddHit(inHit);
      newHit.SetPosition(newPosition);
   }
}
