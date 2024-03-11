#include "AtFitterTask.h"

#include "AtDigiPar.h"
#include "AtFitter.h"
#include "AtGenfit.h"
#include "AtParsers.h"
#include "AtPatternEvent.h"
#include "AtTrackingEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>
#include <Track.h>

#include <algorithm>
#include <iostream>

class AtTrack;
class AtFittedTrack;

ClassImp(AtFitterTask);

AtFitterTask::AtFitterTask(std::unique_ptr<AtFITTER::AtFitter> fitter)
   : fInputBranchName("AtPatternEvent"), fOutputBranchName("AtTrackingEvent"), fIsPersistence(kFALSE),
     fTrackingEventArray(TClonesArray("AtTrackingEvent", 1)), fFitter(std::move(fitter))
{
}

void AtFitterTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

void AtFitterTask::SetInputBranch(TString branchName)
{
   fInputBranchName = branchName;
}

void AtFitterTask::SetOutputBranch(TString branchName)
{
   fOutputBranchName = branchName;
}

InitStatus AtFitterTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtPatternEvent"));
   if (fPatternEventArray == nullptr) {
      LOG(error) << "Cannot find AtPatternEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fTrackingEventArray, fIsPersistence);

   return kSUCCESS;
}

void AtFitterTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtFitterTask";

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

void AtFitterTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   fTrackingEventArray.Delete();

   auto trackingEvent = dynamic_cast<AtTrackingEvent *>(fTrackingEventArray.ConstructedAt(0));

   std::cout << " Event Counter " << fEventCnt << "\n";

   AtPatternEvent &patternEvent = *(dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0)));
   std::vector<AtTrack> &tracks = patternEvent.GetTrackCand();
   std::cout << " AtFitterTask:Exec -  Number of candidate tracks : " << tracks.size() << "\n";

   auto fittedTracks = fFitter->ProcessTracks(tracks);

   std::cout << " Number of fitted tracks " << fittedTracks.size() << "\n";

   for (auto &fittedTrack : fittedTracks)
      trackingEvent->AddFittedTrack(std::move(fittedTrack));

   ++fEventCnt;
}
