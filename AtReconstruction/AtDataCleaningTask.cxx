#include "AtDataCleaningTask.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEvent.h"
#include "AtHit.h"

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

ClassImp(AtDataCleaningTask);

AtDataCleaningTask::AtDataCleaningTask(DataCleaner &&cleaner)
   : fCleaner(std::move(cleaner)), fOutputEventArray(TClonesArray("AtEvent", 1)), fInputEventArray(nullptr)
{
}

InitStatus AtDataCleaningTask::Init()
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

void AtDataCleaningTask::SetParContainers() {}

void AtDataCleaningTask::Exec(Option_t *opt)
{
   fOutputEventArray.Clear("C");

   if (fInputEventArray->GetEntriesFast() == 0)
      return;

   auto inputEvent = dynamic_cast<AtEvent *>(fInputEventArray->At(0));
   auto outputEvent = dynamic_cast<AtEvent *>(fOutputEventArray.ConstructedAt(0));
   *outputEvent = *inputEvent;
   outputEvent->ClearHits();

   auto hits = fCleaner->CleanData(inputEvent->GetHits());
   for (auto &hit : hits)
      outputEvent->AddHit(std::move(hit));
}
