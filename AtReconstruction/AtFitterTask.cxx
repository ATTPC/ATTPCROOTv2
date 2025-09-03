#include "AtFitterTask.h"

#include "AtDigiPar.h"
#include "AtEvent.h"
#include "AtFitMetadata.h"
#include "AtFitter.h"
#include "AtParsers.h"
#include "AtPatternEvent.h"
#include "AtRawEvent.h"
#include "AtTrackingEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>

#include <algorithm>
#include <iostream>

class AtTrack;
class AtFittedTrack;

ClassImp(AtFitterTask);

AtFitterTask::AtFitterTask(std::unique_ptr<EventFit::AtFitter> fitter)
   : fInputBranchName("AtPatternEvent"), fOutputBranchName("AtTrackingEvent"), fIsPersistence(kFALSE),
     fTrackingEventArray(TClonesArray("AtTrackingEvent", 1)), fFitter(std::move(fitter)), fRawEventBranchName(""),
     fEventBranchName(""), fFitMetadataBranchName("")
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

void AtFitterTask::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtFitterTask::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}

void AtFitterTask::SetFitMetadataBranch(TString branchName)
{
   fFitMetadataBranchName = branchName;
   fSaveFitMetadata = true;
}

InitStatus AtFitterTask::Init()
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

   ioMan->Register(fOutputBranchName, "AtTPC", &fTrackingEventArray, fIsPersistence);
   ioMan->Register(fFitMetadataBranchName, "AtTPC", &fFitMetadataArray, fIsPersistence && fSaveFitMetadata);

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray == nullptr) {
      LOG(info) << "AtRawEvent branch name was not set. No AtRawEvent will be passed to the fitter.";
   }

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fEventBranchName));
   if (fEventArray == nullptr) {
      LOG(info) << "AtEvent branch name was not set. No AtEvent will be passed to the fitter.";
   }

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

   // If there is AtRawEvent available, get it so it can be passed to the fitter.
   AtRawEvent *rawEvent = nullptr;
   if (fRawEventArray)
      rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

   // If there is AtEvent available, get it so it can be passed to the fitter.
   AtEvent *event = nullptr;
   if (fEventArray)
      event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   fTrackingEventArray.Delete();

   auto trackingEvent = dynamic_cast<AtTrackingEvent *>(fTrackingEventArray.ConstructedAt(0));
   auto fitMetadata = dynamic_cast<AtFitMetadata *>(fFitMetadataArray.ConstructedAt(0));

   LOG(info) << " Fitting event " << fEventCnt;

   AtPatternEvent *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
   std::vector<AtTrack> &tracks = patternEvent->GetTrackCand();
   LOG(info) << " Number of candidate tracks : " << tracks.size();

   fFitter->FitEvent(trackingEvent, patternEvent, fitMetadata, rawEvent, event);

   auto &fittedTracks = trackingEvent->GetFittedTracks();

   LOG(info) << " Number of fitted tracks : " << fittedTracks.size();

   ++fEventCnt;
}
