#include "AtTabBraggCurve.h"

#include "AtBraggFitMetadata.h"
#include "AtEvent.h"
#include "AtFitTrackMetadata.h"
#include "AtPattern.h" // for AtPattern
#include "AtPatternEvent.h"
#include "AtTabInfo.h" // for AtTabInfoFairRoot, AtTabInfo
#include "AtTrack.h"   // for AtTrack
#include "AtTrackingEvent.h"
#include "AtViewerManager.h"

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
#include <TStyle.h>

#include <array>   // for array
#include <utility> // for move
namespace DataHandling {
class AtSubject;
}

using XYZVector = ROOT::Math::XYZVector;

ClassImp(AtTabBraggCurve);

AtTabBraggCurve::AtTabBraggCurve() : AtTabMain()
{
   fTrackNum = &AtViewerManager::Instance()->GetTrackNum();
   fTrackNum->Attach(this);

   fTrackingEventBranch = &AtViewerManager::Instance()->GetTrackingEventBranch();
   fTrackingEventBranch->Attach(this);

   fFitMetadataBranch = &AtViewerManager::Instance()->GetFitMetadataBranch();
   fFitMetadataBranch->Attach(this);
}

AtTabBraggCurve::~AtTabBraggCurve()
{
   fTrackNum->Detach(this);
   fTrackingEventBranch->Detach(this);
   fFitMetadataBranch->Detach(this);
   delete fHistELossVRange;
   delete fCvsELossVRange;
}

void AtTabBraggCurve::InitTab()
{
   LOG(info) << " =====  Bragg curve tab init =====";

   gEve->AddEvent(fEveEvent.get());
   fEveEvent->AddElement(fHitSet.get());

   gEve->AddEvent(fEvePatternEvent.get());

   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtEvent>>(*fEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtPatternEvent>>(*fPatternEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtTrackingEvent>>(*fTrackingEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtFitMetadata>>(*fFitMetadataBranch));

   gStyle->SetPalette(55);

   LOG(info) << " Initialization complete!";
}

void AtTabBraggCurve::Update(DataHandling::AtSubject *sub)
{
   // If we should update the stuff that depends on the AtEvent
   if (sub == fEventBranch || sub == fEntry)
      UpdateEventElements();
   if (sub == fPatternEventBranch || sub == fEntry || sub == fTrackNum)
      UpdatePatternEventElements();
   if (sub == fFitMetadataBranch || sub == fEntry || sub == fTrackNum)
      UpdateFitMetadata();
   if (sub == fTrackingEventBranch || sub == fEntry || sub == fTrackNum)
      UpdateTrackingEventElements();

   // If we should update the 3D display
   if (sub == fEventBranch || sub == fPatternEventBranch || sub == fEntry)
      gEve->Redraw3D(false); // false -> don't reset camera
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

   auto patternEvent = GetFairRootInfo<AtPatternEvent>();
   if (patternEvent == nullptr) {
      LOG(debug) << "Cannot update AtPatternEvent elements: no event available.";
      DrawHistELossVRange();
      return;
   }

   auto &tracks = patternEvent->GetTrackCand();
   if (!tracks.size()) {
      LOG(warning) << "No tracks in this event.";
      DrawHistELossVRange();
      return;
   }

   if(fTrackNum->Get() == -1) {
      LOG(warning) << "Selected track num is -1. No Bragg curve will be drawn.";
      DrawHistELossVRange();
      return;
   }

   if(fTrackNum->Get() >= tracks.size()) {
      LOG(warning) << "Selected track num is " << fTrackNum->Get() << ", but we have " << tracks.size() << " tracks. The Bragg curve for the last one will be drawn.";
      DrawHistELossVRange(tracks[tracks.size() - 1].GetBraggCurve());
      return;
   }
   DrawHistELossVRange(tracks[fTrackNum->Get()].GetBraggCurve());
}

void AtTabBraggCurve::UpdateTrackingEventElements()
{
   // Reset the graphs to begin with.
   if (fFittedELossGraph != nullptr)
      fCvsELossVRange->GetListOfPrimitives()->Remove(fFittedELossGraph);

   auto trackingEvent = GetFairRootInfo<AtTrackingEvent>();
   if (trackingEvent == nullptr) {
      LOG(debug) << "Cannot update AtTrackingEvent elements: no event available.";
      return;
   }

   auto &fittedTracks = trackingEvent->GetFittedTracks();
   if (!fittedTracks.size()) {
      LOG(warning) << "No fitted tracks in this event.";
      return;
   }

   if(fTrackNum->Get() == -1) {
      LOG(warning) << "Selected track num is -1. No fitted track information will be printed.";
      return;
   }

   if(fTrackNum->Get() >= fittedTracks.size()) {
      LOG(warning) << "Selected track num is " << fTrackNum->Get() << ", but we have " << fittedTracks.size() << " fitted tracks. The information for the last fitted track will be printed.";
      PrintFittedTrackInfo(*fittedTracks.at(fittedTracks.size() - 1));
      DrawBestFittingELoss(*fittedTracks.at(fittedTracks.size() - 1));
      return;
   }
   PrintFittedTrackInfo(*fittedTracks.at(fTrackNum->Get()));
   DrawBestFittingELoss(*fittedTracks.at(fTrackNum->Get()));
}

void AtTabBraggCurve::UpdateFitMetadata()
{
   auto fitMetadata = GetFairRootInfo<AtFitMetadata>();
   if (fitMetadata == nullptr) {
      LOG(debug) << "Cannot update AtFitMetadata: no metadata available.";
      return;
   }

   if (!fitMetadata->GetNumEntries()) {
      LOG(warning) << "No fit metadata entries in this event.";
      return;
   }

   if(fTrackNum->Get() == -1) {
      LOG(warning) << "Selected track num is -1. No fit metadata will be printed.";
      return;
   }

   PrintFittedTrackMetadata(fitMetadata);
}

void AtTabBraggCurve::PrintFittedTrackInfo(AtFittedTrack fittedTrack)
{
   std::cout << " Fitted track with ID " << fittedTrack.GetTrackID() << " information:" << std::endl;

   // If track punched through or no ELoss profile, there is nothing to print other than a warning.
   std::unique_ptr<AtFitTrackMetadata> &fitTrackMetadata = fittedTrack.GetTrackMetadata();
   auto braggFitMetadata = dynamic_cast<AtBraggFitMetadata *>(fitTrackMetadata.get());
   Bool_t isPunchThrough = braggFitMetadata->GetIsPunchThrough();
   Bool_t isReconstructedELoss = braggFitMetadata->GetIsReconstructedELoss();
   if (isPunchThrough || !isReconstructedELoss) {
      std::cout << " Punch through? " << isPunchThrough << std::endl;
      std::cout << " Reconstructed ELoss profile? " << isPunchThrough << std::endl;
      return;
   }

   // Particle info.
   AtFittedTrack::ParticleInfo particleInfo = fittedTrack.GetParticleInfo();
   std::cout << " Particle information:" << std::endl;
   std::cout << "   - PDG code: " << particleInfo.idPDG.Data() << std::endl;
   std::cout << "   - Z = " << particleInfo.charge << std::endl;
   std::cout << "   - m = " << particleInfo.mass << " amu" << std::endl;

   // Kinematics.
   AtFittedTrack::Kinematics kinematics = fittedTrack.GetKinematics();
   std::cout << " Kinematics:" << std::endl;
   std::cout << "   - kineticEnergy = " << kinematics.kineticEnergy << " MeV" << std::endl;
   std::cout << "   - theta = " << kinematics.theta * 180 / TMath::Pi() << " deg" << std::endl;
   std::cout << "   - phi = " << kinematics.phi * 180 / TMath::Pi() << " deg" << std::endl;

   // Vertex.
   XYZVector vertex = fittedTrack.GetVertex();
   std::cout << "Vertex = (" << vertex.X() << ", " << vertex.Y() << ", " << vertex.Z() << ") [mm]" << std::endl;

   std::cout << std::endl;
}

void AtTabBraggCurve::DrawBestFittingELoss(AtFittedTrack fittedTrack)
{
   std::unique_ptr<AtFitTrackMetadata> &fitTrackMetadata = fittedTrack.GetTrackMetadata();
   auto braggFitMetadata = dynamic_cast<AtBraggFitMetadata *>(fitTrackMetadata.get());
   if (braggFitMetadata == nullptr) {
      LOG(error) << "The fit metadata is not of type AtBraggFitMetadata. The fit ELoss profile will not be plotted!";
      return;
   }

   // Again, if punch through or no ELoss profile, there is no best fit to draw.
   Bool_t isPunchThrough = braggFitMetadata->GetIsPunchThrough();
   Bool_t isReconstructedELoss = braggFitMetadata->GetIsReconstructedELoss();
   if (isPunchThrough || !isReconstructedELoss)
      return;

   auto fitELossValues = braggFitMetadata->GetELossFitValues();
   double amplitudeFactor = braggFitMetadata->GetAmplitudeFactor();

   fFittedELossGraph = new TGraph();
   for (auto pair : fitELossValues)
      fFittedELossGraph->AddPoint(pair.second, amplitudeFactor * pair.first);
   fCvsELossVRange->cd();
   fFittedELossGraph->Draw("same");
   fCvsELossVRange->Modified();
   fCvsELossVRange->Update();
}

void AtTabBraggCurve::PrintFittedTrackMetadata(AtFitMetadata *fitMetadata)
{
   int trackIdx{fTrackNum->Get()};
   if(fTrackNum->Get() >= fitMetadata->GetNumEntries()) {
      LOG(warning) << "Selected track num is " << fTrackNum->Get() << ", but we have " << fitMetadata->GetNumEntries() << " metadata entries. The metadata for the last fitted track will be printed.";
      trackIdx = fitMetadata->GetNumEntries() - 1;
   }

   std::cout << " === Metadata of all fits for track with ID " << trackIdx << " in event " << fitMetadata->GetEventID()
             << " === " << std::endl;

   auto &fitTrackMetadatas = fitMetadata->GetTrackMetadatasVector(trackIdx);

   for (auto &fitTrackMetadata : fitTrackMetadatas) {
      auto braggFitMetadata = dynamic_cast<AtBraggFitMetadata *>(fitTrackMetadata.get());
      if (braggFitMetadata == nullptr) {
         LOG(error) << "The fit metadata is not of type AtBraggFitMetadata. The fit metadata will not be printed!";
         return;
      }

      braggFitMetadata->Print();

      std::cout << std::endl;
   }
}
