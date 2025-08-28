#include "AtTabBraggCurve.h"

#include "AtPattern.h" // for AtPattern
#include "AtPatternEvent.h"
#include "AtTabInfo.h" // for AtTabInfoFairRoot, AtTabInfo
#include "AtTrack.h"   // for AtTrack

#include <FairLogger.h> // for LOG

#include <TAttMarker.h> // for TAttMarker
#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveElement.h>      // for TEveElement
#include <TEveEventManager.h> // for TEveEventManager
#include <TEveGeoNode.h>
#include <TEveManager.h>  // for TEveManager, gEve
#include <TEvePointSet.h> // for TEvePointSet
#include <TEveViewer.h>
#include <TEveWindow.h>
#include <TGLViewer.h>
#include <TGTab.h>
#include <TGeoManager.h>
#include <TRootEmbeddedCanvas.h>

#include <array>   // for array
#include <utility> // for move
namespace DataHandling {
class AtSubject;
}

ClassImp(AtTabBraggCurve);

AtTabBraggCurve::AtTabBraggCurve() : AtTabMain() {}

AtTabBraggCurve::~AtTabBraggCurve()
{
   delete fHistELossVRange;
   delete fCvsELossVRange;
}

void AtTabBraggCurve::Update(DataHandling::AtSubject *sub)
{
   // If we should update the stuff that depends on the AtEvent
   if (sub == fEventBranch || sub == fEntry) {
      UpdateEventElements();
   }
   if (sub == fPatternEventBranch || sub == fEntry) {
      UpdatePatternEventElements();
   }

   // If we should update the 3D display
   if (sub == fEventBranch || sub == fPatternEventBranch || sub == fEntry) {
      gEve->Redraw3D(false); // false -> don't reset camera
   }
}

void AtTabBraggCurve::MakeTab(TEveWindowSlot *slot)
{
   TEveWindowPack *pack = nullptr;

   // 3D
   pack = slot->MakePack();
   pack->SetElementName("BraggCurve");
   pack->SetHorizontal();
   pack->SetShowTitleBar(kFALSE);

   pack->NewSlot()->MakeCurrent();
   TEveViewer *view3D = gEve->SpawnNewViewer("3D View", "");
   view3D->AddScene(gEve->GetGlobalScene());
   view3D->AddScene(gEve->GetEventScene());

   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   // fCvsDeDx = new TCanvas("dEdx Bragg curve Canvas");
   // fCvsDeDx->ToggleEditor();
   slot->StopEmbedding();

   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("Bragg curve Canvas");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsELossVRange = ecvs->GetCanvas();
   // fCvsELossVRange->AddExec("ex", "AtTab3DBraggCurve::NextTrack()");

   fCvsELossVRange->ToggleEventStatus();
   DrawHistELossVRange();

   if (gGeoManager) {
      TGeoNode *geoNode = gGeoManager->GetTopNode();
      Int_t option = 1;
      Int_t level = 3;
      Int_t nNodes = 10000;
      auto *topNode = new TEveGeoTopNode(gGeoManager, geoNode, option, level, nNodes);
      gEve->AddGlobalElement(topNode);

      Int_t transparency = 80;
      gGeoManager->GetVolume("drift_volume")->SetTransparency(transparency);
      gEve->FullRedraw3D(kTRUE);
   }

   gEve->GetBrowser()->GetTabRight()->SetTab(1);

   gEve->Redraw3D(true, true);

   TGLViewer *dfViewer = gEve->GetDefaultGLViewer(); // Is this doing anything?
   dfViewer->CurrentCamera().RotateRad(-.7, 0.5);
   dfViewer->DoDraw();
   UpdateRenderState();
}

void AtTabBraggCurve::DrawHistELossVRange()
{
   AtTrack::BraggCurve braggCurve;
   braggCurve.nBins = 1000;
   braggCurve.binSize = 1;
   DrawHistELossVRange(braggCurve);
}

void AtTabBraggCurve::DrawHistELossVRange(AtTrack::BraggCurve braggCurve)
{
   if (fHistELossVRange != nullptr)
      fCvsELossVRange->GetListOfPrimitives()->Remove(fHistELossVRange);

   int nBins = braggCurve.nBins;
   double binSize = braggCurve.binSize;

   fHistELossVRange = new TH1F("Charge vs Range", "Charge vs Range", nBins, 0, nBins * binSize);
   fHistELossVRange->SetDirectory(0);
   fCvsELossVRange->cd();
   fHistELossVRange->Draw();
   fHistELossVRange->GetXaxis()->SetTitle("Range [mm]");
   fHistELossVRange->GetYaxis()->SetTitle("Charge [ADC]");

   for (int i = 0; i < braggCurve.IntegratedELossValues.size(); i++) {
      fHistELossVRange->SetBinContent(i + 1, braggCurve.IntegratedELossValues[i]);
      fHistELossVRange->SetBinError(i + 1, braggCurve.ELossErrors[i]);
   }

   fCvsELossVRange->Modified();
   fCvsELossVRange->Update();
}

void AtTabBraggCurve::UpdatePatternEventElements()
{
   AtTabMain::UpdatePatternEventElements();

   auto fPatternEvent = GetFairRootInfo<AtPatternEvent>();
   if (fPatternEvent == nullptr) {
      LOG(debug) << "Cannot update AtPatternEvent elements: no event available";
      return;
   }

   auto &tracks = fPatternEvent->GetTrackCand();
   if (tracks.size()) {
      fTrackIdx = 0;
      DrawHistELossVRange(tracks[fTrackIdx].GetBraggCurve());
   } else {
      fTrackIdx = -1;
      DrawHistELossVRange();
   }
}
