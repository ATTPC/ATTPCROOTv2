#include "AtSimpleSimulationReplayTask.h"

#include "AtMCTrack.h"

#include <FairLogger.h>
#include <FairTask.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TTree.h>

AtSimpleSimulationReplayTask::AtSimpleSimulationReplayTask(std::unique_ptr<AtSimpleSimulation> sim)
   : AtSimpleSimulationTask(std::move(sim))
{
}

InitStatus AtSimpleSimulationReplayTask::InitEventSource()
{
   if (fPrimaryTrackSourceFile.empty())
      return kSUCCESS;

   fPrimaryTrackFile = TFile::Open(fPrimaryTrackSourceFile.c_str(), "READ");
   if (fPrimaryTrackFile == nullptr || fPrimaryTrackFile->IsZombie()) {
      LOG(fatal) << "AtSimpleSimulationReplayTask: cannot open primary track source " << fPrimaryTrackSourceFile;
      return kFATAL;
   }

   fPrimaryTrackTree = dynamic_cast<TTree *>(fPrimaryTrackFile->Get("cbmsim"));
   if (fPrimaryTrackTree == nullptr) {
      LOG(fatal) << "AtSimpleSimulationReplayTask: missing cbmsim tree in primary track source "
                 << fPrimaryTrackSourceFile;
      return kFATAL;
   }

   fPrimaryTrackTree->SetBranchAddress("MCTrack", &fPrimaryTrackInput);
   LOG(info) << "AtSimpleSimulationReplayTask: replaying primary MC tracks from " << fPrimaryTrackSourceFile;
   return kSUCCESS;
}

AtSimpleSimulationTask::EventState AtSimpleSimulationReplayTask::LoadEvent()
{
   if (fPrimaryTrackTree == nullptr)
      return {};

   if (!LoadPrimaryTracksFromSource())
      return {};

   // The replay task transports all primaries from every source event. There is no
   // beam/reaction state machine because the source file already contains the correct
   // primaries — no generators are running and AtVertexPropagator state is not needed.
   EventState state;
   state.hasEvent = true;
   state.beamEvent = false;
   state.transportPrimaries = true;
   return state;
}

void AtSimpleSimulationReplayTask::FinishEventSource()
{
   if (fPrimaryTrackFile == nullptr)
      return;

   fPrimaryTrackFile->Close();
   delete fPrimaryTrackFile;
   fPrimaryTrackFile = nullptr;
   fPrimaryTrackTree = nullptr;
   fPrimaryTrackInput = nullptr;
}

bool AtSimpleSimulationReplayTask::LoadPrimaryTracksFromSource()
{
   if (fPrimaryTrackTree == nullptr || fSourceEventIndex >= fPrimaryTrackTree->GetEntries())
      return false;

   fCollector.Clear();
   fPrimaryTrackTree->GetEntry(fSourceEventIndex++);
   if (fPrimaryTrackInput == nullptr)
      return true;

   for (int i = 0; i < fPrimaryTrackInput->GetEntriesFast(); ++i) {
      auto *track = dynamic_cast<AtMCTrack *>(fPrimaryTrackInput->At(i));
      if (track == nullptr || track->GetMotherId() != -1)
         continue;

      Int_t ntr = 0;
      fCollector.PushTrack(1, -1, track->GetPdgCode(), track->GetPx(), track->GetPy(), track->GetPz(), track->GetEnergy(),
                           track->GetStartX(), track->GetStartY(), track->GetStartZ(), track->GetStartT(), 0., 0., 0.,
                           kPPrimary, ntr, 0., 0, -1);
   }

   return true;
}

ClassImp(AtSimpleSimulationReplayTask);
