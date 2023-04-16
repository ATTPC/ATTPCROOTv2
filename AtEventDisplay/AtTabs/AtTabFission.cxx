#include "AtTabFission.h"

#include "AtEvent.h" // for AtEvent, AtEvent::HitVector
#include "AtFissionEvent.h"
#include "AtHit.h"           // for AtHit, AtHit::XYZPoint
#include "AtMap.h"           // for AtMap
#include "AtPad.h"           // for AtPad
#include "AtPadReference.h"  // for operator<<
#include "AtPattern.h"       // for AtPattern
#include "AtPatternEvent.h"  // for AtPatternEvent
#include "AtRawEvent.h"      // for AtRawEvent
#include "AtTabInfo.h"       // for AtTabInfoFairRoot, AtTabInfoBase
#include "AtTrack.h"         // for AtTrack
#include "AtViewerManager.h" // for AtViewerManager

#include <FairLogger.h> // for LOG, Logger

#include <Math/Point3D.h>        // for PositionVector3D
#include <TAttMarker.h>          // for TAttMarker
#include <TCanvas.h>             // for TCanvas
#include <TEveBrowser.h>         // for TEveBrowser
#include <TEveEventManager.h>    // for TEveEventManager
#include <TEveGeoNode.h>         // for TEveGeoTopNode
#include <TEveManager.h>         // for TEveManager, gEve
#include <TEvePointSet.h>        // for TEvePointSet
#include <TEveViewer.h>          // for TEveViewer
#include <TEveWindow.h>          // for TEveWindowPack, TEveWindowSlot, TEv...
#include <TGLCamera.h>           // for TGLCamera
#include <TGLViewer.h>           // for TGLViewer
#include <TGTab.h>               // for TGTab
#include <TGeoManager.h>         // for gGeoManager, TGeoManager
#include <TGeoVolume.h>          // for TGeoVolume
#include <TH1.h>                 // for TH1I
#include <TH2Poly.h>             // for TH2Poly
#include <TNamed.h>              // for TNamed
#include <TObject.h>             // for TObject
#include <TRootEmbeddedCanvas.h> // for TRootEmbeddedCanvas
#include <TString.h>             // for Form, TString
#include <TStyle.h>              // for TStyle, gStyle
#include <TVirtualPad.h>         // for TVirtualPad, gPad
#include <TVirtualX.h>           // for TVirtualX, gVirtualX

#include <algorithm>          // for max
#include <array>              // for array
#include <cstdio>             // for sprintf
#include <ext/alloc_traits.h> // for __alloc_traits<>::value_type
#include <iostream>           // for operator<<, endl, basic_ostream
#include <utility>            // for move
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

void AtTabFission::Exec()
{
   UpdateFissionElements();
   AtTabMain::Exec();
}

void AtTabFission::UpdateFissionElements()
{
   if (fEveFissionEvent == nullptr)
      return;

   // auto eventInfo = fTabInfo->GetAugment<AtTabInfoFairRoot<AtFissionEvent>>("AtFissionEvent");
   // eventInfo->Update(&fFissionEventBranch);

   fFissionEventBranch.Notify();
   auto fissionEvent = GetFairRootInfo<AtFissionEvent>();

   if (fissionEvent == nullptr) {
      LOG(error) << "Cannot update fission event: no event availible.";
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
