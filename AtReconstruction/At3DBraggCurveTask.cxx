#include "At3DBraggCurveTask.h"

#include "At3DBraggCurveHandler.h"
#include "AtPatternEvent.h"
#include "AtPatternTypes.h"
#include "AtRawEvent.h"

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h> // for TClonesArray
#include <TObject.h>      // for TObject

#include <memory> // for allocator

ClassImp(At3DBraggCurveTask);

At3DBraggCurveTask::At3DBraggCurveTask()
   : fRawEventBranchName("AtRawEvent"), fPatternEventBranchName("AtPatternEvent"),
     fOutputPatternEventBranchName("AtPatternEvent3DBragg"), kIsPersistence(kFALSE),
     fOutputPatternEventArray("AtPatternEvent", 1)
{
}

At3DBraggCurveTask::~At3DBraggCurveTask() = default;

InitStatus At3DBraggCurveTask::Init()
{
   LOG(info) << "Initilization of At3DBraggCurveTask";

   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << " At3DBraggCurveTask : Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray == nullptr) {
      LOG(error) << "At3DBraggCurveTask : Cannot find AtRawEvent array! Will get the 3D Bragg curves without the use "
                    "of the trace projection! ";
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fPatternEventBranchName));
   if (fPatternEventArray == nullptr) {
      LOG(error) << "At3DBraggCurveTask : Cannot find AtPatternEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputPatternEventBranchName, "AtTPC", &fOutputPatternEventArray, kIsPersistence);

   f3DBraggHandler = new At3DBraggCurveHandler();

   return kSUCCESS;
}

void At3DBraggCurveTask::Exec(Option_t *opt)
{

   if (fRawEventArray != nullptr) {
      if (fRawEventArray->GetEntriesFast() == 0)
         return;
      fRawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   }

   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   fPatternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));

   fOutputPatternEventArray.Delete();
   auto patternEvent3DBraggCurve = dynamic_cast<AtPatternEvent *>(fOutputPatternEventArray.ConstructedAt(0));

   // We first add the noise.
   auto &noiseHits = fPatternEvent->GetNoiseHits();
   for (auto &&noiseHit : noiseHits)
      patternEvent3DBraggCurve->AddNoise(std::move(*noiseHit));

   std::vector<AtTrack> &tracks = fPatternEvent->GetTrackCand();
   for (int i = 0; i < tracks.size(); ++i) {
      AtTrack newTrack = AtTrack(tracks[i]);
      if (fRawEventArray == nullptr)
         f3DBraggHandler->SetTrack(newTrack);
      else
         f3DBraggHandler->SetTrack(newTrack, fRawEvent);

      if (f3DBraggFitter != nullptr) {
         auto fitResult = f3DBraggFitter->ProcessTrack(newTrack);
         newTrack.Set3DBraggFitResult(std::move(fitResult));
      }

      patternEvent3DBraggCurve->AddTrack(newTrack);
   }
}
