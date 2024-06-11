#include "AtSpaceChargeCorrectionTask.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEvent.h"
#include "AtHit.h"
#include "AtSpaceChargeModel.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <TClonesArray.h>
#include <TObject.h>

#include <utility>
#include <vector>

class AtDigiPar;
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

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(fatal) << "No runtime database!";

   auto fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   fSCModel->LoadParameters(fPar);
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
   *outputEvent = *inputEvent;
   outputEvent->ClearHits();

   for (auto &inHit : inputEvent->GetHits()) {
      XYZPoint newPosition;
      newPosition = fSCModel->CorrectSpaceCharge(inHit->GetPosition());
      auto &newHit = outputEvent->AddHit(inHit->Clone());
      newHit.SetPosition(newPosition);
      LOG(debug) << inHit->GetPosition() << " " << outputEvent->GetHits().back()->GetPosition();
   }
}
