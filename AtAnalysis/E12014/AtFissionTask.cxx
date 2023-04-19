#include "AtFissionTask.h"

#include "AtEvent.h"
#include "AtFissionEvent.h"
#include "AtHit.h"          // for AtHit
#include "AtPatternEvent.h" // for AtPatternEvent

#include <FairLogger.h>      // for LOG, Logger
#include <FairRootManager.h> // for FairRootManager

#include <TObject.h> // for TObject

#include <algorithm> // for sort, find_if
#include <utility>   // for move

AtFissionTask::AtFissionTask(double lambda)
   : FairTask("AtFissionTask"), fFissionEventArray("AtFissionEvent", 1), fLambda(lambda)
{
}

InitStatus AtFissionTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kFATAL;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fPatternBranch));
   if (fPatternEventArray == nullptr) {
      LOG(fatal) << "Cannot find AtPatternEvent array in branch " << fPatternBranch << "!";
      return kFATAL;
   }
   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fEventBranch));
   if (fEventArray == nullptr) {
      LOG(fatal) << "Cannot find AtPatternEvent array in branch " << fEventBranch << "!";
      return kFATAL;
   }

   ioMan->Register(fOutBranch, "AtTPC", &fFissionEventArray, fIsPersistant);

   return kSUCCESS;
}

/**
 * Get the beam hits from the fission event, and sort them by hitID.
 */
std::vector<AtHit *> AtFissionTask::GetSortedBeamHits(AtFissionEvent *event)
{
   auto beamHits = event->GetBeamHitsCorr();
   std::sort(beamHits.begin(), beamHits.end(),
             [](const AtHit *a, const AtHit *b) { return a->GetHitID() < b->GetHitID(); });
   return beamHits;
}

std::vector<AtHit *> AtFissionTask::GetSortedFragmentHits(AtFissionEvent *event, int fragID)
{
   auto hits = event->GetFragHitsCorr(fragID);
   std::sort(hits.begin(), hits.end(), [](const AtHit *a, const AtHit *b) { return a->GetHitID() < b->GetHitID(); });
   return hits;
}

void AtFissionTask::Exec(Option_t *opt)
{
   fFissionEventArray.Clear();
   auto *fissionEvent = dynamic_cast<AtFissionEvent *>(fFissionEventArray.ConstructedAt(0));
   if (fissionEvent == nullptr) {
      LOG(fatal) << "Failed to create or load a fission event in the branch.";
      return;
   }

   auto *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
   if (patternEvent == nullptr) {
      LOG(fatal) << "Failed to load an AtPatternEvent in the branch.";
      return;
   }

   auto *event = dynamic_cast<AtEvent *>(fEventArray->At(0));
   if (event == nullptr) {
      LOG(fatal) << "Failed to load an AtEvent in the branch.";
      return;
   }

   // Skip if event is not good
   if (!event->IsGood())
      return;

   try {
      // Copy this pattern event into our fission event
      *fissionEvent = AtFissionEvent(*patternEvent);
      fissionEvent->SetLambda(fLambda);

      // This is the vector we will search through for the hits to clone into our fission event
      event->SortHitArrayID();
      auto &uncorrHits = event->GetHits();

      // Get this hits associated with the beam and copy the uncorrected version of them
      auto clonedHits = GetMatchingHits(GetSortedBeamHits(fissionEvent), uncorrHits);
      fissionEvent->SetBeamHits(std::move(clonedHits));

      // Get this hits associated with the fragments and copy the uncorrected version of them
      for (int i = 0; i < 2; ++i) {
         clonedHits = GetMatchingHits(GetSortedFragmentHits(fissionEvent, i), uncorrHits);
         fissionEvent->SetFragHits(i, std::move(clonedHits));
      }
   } catch (...) {
      LOG(error) << "Failed to create a fission event for event " << event->GetEventID();
   }
}

AtFissionTask::HitVector
AtFissionTask::GetMatchingHits(const std::vector<AtHit *> hitsToFind, const HitVector &hitsToClone)
{
   auto lastHit = hitsToClone.begin();
   HitVector ret;
   for (auto hit : hitsToFind) {
      auto hitIt = std::find_if(lastHit, hitsToClone.end(), [hit](const std::unique_ptr<AtHit> &hit2) {
         return hit->GetHitID() == hit2->GetHitID();
      });

      if (hitIt == hitsToClone.end())
         continue;

      ret.push_back(hitIt->get()->Clone());
      lastHit = hitIt;
   }
   return ret;
}
