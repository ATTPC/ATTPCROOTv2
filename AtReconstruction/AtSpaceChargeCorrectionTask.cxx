#include "AtSpaceChargeCorrectionTask.h"

#include <FairTask.h>
#include <Math/Point3Dfwd.h>
#include <TObject.h>
#include <fairlogger/Logger.h>
#include <Math/Point3D.h>
#include <utility>
#include <vector>

#include "AtSpaceChargeModel.h"
#include <FairRootManager.h>
#include "AtEvent.h"
#include <TClonesArray.h>
#include "AtHit.h"

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

   fInputEventArray = dynamic_cast<TClonesArray *>(FairRootManager::Instance()->GetObject(fInputBranchName.data()));
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
      newPosition = fSCModel->CorrectSpaceCharge(inHit.GetPosition());
      auto &newHit = outputEvent->AddHit(inHit);
      newHit.SetPosition(newPosition);
      LOG(debug) << inHit.GetPosition() << " " << outputEvent->GetHitArray().back().GetPosition();
   }
}
