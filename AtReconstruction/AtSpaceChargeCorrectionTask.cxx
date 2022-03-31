#include "AtSpaceChargeCorrectionTask.h"
#include "AtSpaceChargeModel.h"

#include "FairLogger.h"
#include "FairRootManager.h"

#include "AtEvent.h"
#include "TClonesArray.h"

using XYZPoint = ROOT::Math::XYZPoint;

ClassImp(AtSpaceChargeCorrectionTask);

AtSpaceChargeCorrectionTask::AtSpaceChargeCorrectionTask(SCModelPtr &&SCModel)
   : fSCModel(std::move(SCModel)), fOutputEventArray(TClonesArray("AtEvent", 1)), fInputEventArray(nullptr)
{
}

InitStatus AtSpaceChargeCorrectionTask::Init()
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

   FairRootManager::Instance()->Register(fOuputBranchName.data(), "AtTpc", &fOutputEventArray, fIsPersistent);

   return kSUCCESS;
}

void AtSpaceChargeCorrectionTask::SetParContainers() {}

void AtSpaceChargeCorrectionTask::Exec(Option_t *opt)
{
   fOutputEventArray.Clear("C");

   if (fInputEventArray->GetEntriesFast() == 0)
      return;

   auto inputEvent = dynamic_cast<AtEvent *>(fInputEventArray->At(0));
   auto outputEvent = dynamic_cast<AtEvent *>(fOutputEventArray.ConstructedAt(0));
   outputEvent->CopyFrom(*inputEvent);

   for (auto &inHit : inputEvent->GetHitArray()) {
      XYZPoint newPosition;
      newPosition = fSCModel->ApplySpaceCharge(inHit.GetPosition());
      auto newHit = outputEvent->AddHit(inHit);
      newHit.SetPosition(newPosition);
   }
}
