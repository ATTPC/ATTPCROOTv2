#include "AtTabFission.h"

#include "AtFissionEvent.h" // for AtFissionEvent
#include "AtPattern.h"      // for AtPattern
#include "AtTabInfo.h"      // for AtTabInfoFairRoot, AtTabInfo
#include "AtTrack.h"        // for AtTrack

#include <FairLogger.h> // for LOG

#include <TAttMarker.h>       // for TAttMarker
#include <TEveElement.h>      // for TEveElement
#include <TEveEventManager.h> // for TEveEventManager
#include <TEveManager.h>      // for TEveManager, gEve
#include <TEvePointSet.h>     // for TEvePointSet

#include <array>   // for array
#include <utility> // for move
namespace DataHandling {
class AtSubject;
}

ClassImp(AtTabFission);

AtTabFission::AtTabFission()
   : AtTabMain(),
     fCorrHitSet({std::make_unique<TEvePointSet>("Corrected Frag 0"),
                  std::make_unique<TEvePointSet>("Corrected Frag 1"), std::make_unique<TEvePointSet>("Corrected Beam")})
{
}
void AtTabFission::InitTab()
{
   AtTabMain::InitTab();
   gEve->AddEvent(fEveFissionEvent.get());

   fFissionEventBranch.SetBranchName("AtFissionEvent");
   auto fissionInfo = std::make_unique<AtTabInfoFairRoot<AtFissionEvent>>(fFissionEventBranch);
   fTabInfo->AddAugment(std::move(fissionInfo));

   fUncorrHitSet->SetDestroyOnZeroRefCnt(false);
   for (auto &set : fCorrHitSet)
      set->SetDestroyOnZeroRefCnt(false);
}

void AtTabFission::Update(DataHandling::AtSubject *sub)
{
   if (sub == fEntry)
      UpdateFissionElements();

   AtTabMain::Update(sub);
}

void AtTabFission::UpdateFissionElements()
{
   if (fEveFissionEvent == nullptr)
      return;

   fFissionEventBranch.Notify();
   auto fissionEvent = GetFairRootInfo<AtFissionEvent>();

   if (fissionEvent == nullptr) {
      LOG(debug) << "Cannot update fission event: no event availible.";
      return;
   }

   // Grab the pattern
   try {
      auto &track = fissionEvent->GetYTrack();

      fEveFissionEvent->RemoveElements();

      // Add the corrected hits to the viewer
      for (int i = 0; i < 2; ++i) {
         auto hitSet = fCorrHitSet[i].get();
         SetPointsFromHits(*hitSet, fissionEvent->GetFragHitsCorr(i));
         fHitAttr.Copy(*hitSet);
         hitSet->SetMarkerColor(GetTrackColor(i));
         fEveFissionEvent->AddElement(hitSet);
      }
      auto hitSet = fCorrHitSet[2].get();
      fHitAttr.Copy(*hitSet);
      SetPointsFromHits(*hitSet, fissionEvent->GetBeamHitsCorr());
      hitSet->SetMarkerColor(GetTrackColor(2));
      fEveFissionEvent->AddElement(hitSet);

      // Add the uncorrected hits to the viewer
      hitSet = fUncorrHitSet.get();
      fHitAttr.Copy(*hitSet);
      SetPointsFromHits(*hitSet, fissionEvent->GetFragHits());
      fEveFissionEvent->AddElement(fUncorrHitSet.get());

      // Add the pattern to the viewer
      auto pattern = track.GetPattern()->GetEveElement();
      pattern->SetDestroyOnZeroRefCnt(false);
      pattern->SetMainColor(kRed);
      pattern->FindChild("beam")->SetRnrState(false);
      fEveFissionEvent->AddElement(pattern);
   } catch (...) {
      fEveFissionEvent->RemoveElements();
   }
}

void AtTabFission::UpdateRenderState()
{
   fEveEvent->SetRnrState(false);
   fEvePatternEvent->SetRnrState(false);
   fEveFissionEvent->SetRnrState(true);
   fUncorrHitSet->SetRnrState(false);
}
