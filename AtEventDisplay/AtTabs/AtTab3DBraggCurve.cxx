#include "AtTab3DBraggCurve.h"

#include "AtContainerManip.h" // for GetPointerVector
#include "AtEvent.h"          // for AtEvent, AtEvent::HitVector
#include "AtHit.h"            // for AtHit, AtHit::XYZPoint
#include "AtMap.h"            // for AtMap
#include "AtPad.h"            // for AtPad
#include "AtPadReference.h"   // for operator<<
#include "AtPattern.h"        // for AtPattern
#include "AtPatternEvent.h"   // for AtPatternEvent
#include "AtPatternLine.h"    // for AtPattern
#include "AtRawEvent.h"       // for AtRawEvent
#include "AtTabInfo.h"        // for AtTabInfoFairRoot, AtTabInfoBase
#include "AtTrack.h"          // for AtTrack
#include "AtViewerManager.h"  // for AtViewerManager

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
#include <TH1F.h>                // for TH1I
#include <TNamed.h>              // for TNamed
#include <TObject.h>             // for TObject
#include <TRootEmbeddedCanvas.h> // for TRootEmbeddedCanvas
#include <TString.h>             // for Form, TString
#include <TStyle.h>              // for TStyle, gStyle
#include <TVirtualPad.h>         // for TVirtualPad, gPad
#include <TVirtualX.h>           // for TVirtualX, gVirtualX

#include <algorithm> // for max
#include <array>     // for array
#include <cstdio>    // for sprintf
#include <iostream>  // for operator<<, endl, basic_ostream
#include <utility>   // for move
// IWYU pragma: no_include <ext/alloc_traits.h>
namespace DataHandling {
class AtSubject;
}
class TGeoNode; // lines 45-45

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTab3DBraggCurve);

AtTab3DBraggCurve::AtTab3DBraggCurve() : AtTabBase("3DBraggCurve")
{
   if (AtViewerManager::Instance() == nullptr)
      throw "AtViewerManager must be initialized before creating tabs!";

   fEventBranch = &AtViewerManager::Instance()->GetEventBranch();
   fEventBranch->Attach(this);

   fPatternEventBranch = &AtViewerManager::Instance()->GetPatternEventBranch();
   fPatternEventBranch->Attach(this);

   fEntry = &AtViewerManager::Instance()->GetCurrentEntry();
   fEntry->Attach(this);
}

AtTab3DBraggCurve::~AtTab3DBraggCurve()
{
   fEventBranch->Detach(this);
   fPatternEventBranch->Detach(this);
   fEntry->Detach(this);
}

void AtTab3DBraggCurve::InitTab()
{
   std::cout << " ===== AtTab3DBraggCurve::Init =====" << std::endl;

   gEve->AddEvent(fEveEvent.get());
   fEveEvent->AddElement(fHitSet.get());

   gEve->AddEvent(fEvePatternEvent.get());

   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtEvent>>(*fEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtPatternEvent>>(*fPatternEventBranch));

   gStyle->SetPalette(55);

   std::cout << " AtTab3DBraggCurve::Init : Initialization complete! " << std::endl;
}

void AtTab3DBraggCurve::ExpandNumPatterns(int num)
{

   // Expand vector so it's large enough for all of the patterns in the event
   if (fPatternHitSets.size() < num)
      LOG(info) << "Expanding number of patterns to " << num << " from " << fPatternHitSets.size() << std::endl;
   while (fPatternHitSets.size() < num) {
      int trackID = fPatternHitSets.size();

      auto trackSet = std::make_unique<TEvePointSet>(TString::Format("Track_%d", trackID));
      trackSet->SetDestroyOnZeroRefCnt(false);
      fHitAttr.Copy(*trackSet);
      trackSet->SetMarkerColor(GetTrackColor(fPatternHitSets.size()));

      fPatternHitSets.push_back(std::move(trackSet));
   }
}

Color_t AtTab3DBraggCurve::GetTrackColor(int i)
{
   std::vector<Color_t> colors = {kBlue - 7,    kGreen - 8, kOrange, kViolet, kYellow, kTeal - 6,
                                  kMagenta + 1, kBlue,      kViolet, kYellow, kCyan};
   if (i < colors.size()) {
      return colors.at(i);
   } else
      return kAzure;
}

void AtTab3DBraggCurve::Update(DataHandling::AtSubject *sub)
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

void AtTab3DBraggCurve::UpdateRenderState()
{
   fEveEvent->SetRnrState(true);
   fEvePatternEvent->SetRnrState(false);
}

void AtTab3DBraggCurve::UpdatePatternEventElements()
{
   if (fEvePatternEvent == nullptr)
      return;

   auto fPatternEvent = GetFairRootInfo<AtPatternEvent>();
   if (fPatternEvent == nullptr) {
      LOG(debug) << "Cannot update AtPatternEvent elements: no event available";
      return;
   }

   // Make sure we have enough TEve elements to draw all the tracks
   auto &tracks = fPatternEvent->GetTrackCand();
   ExpandNumPatterns(tracks.size());

   // Remove all the elements, and re-add them
   fEvePatternEvent->RemoveElements();
   for (int i = 0; i < tracks.size(); ++i) {
      if (tracks[i].GetPattern() == nullptr)
         continue;

      // Update the hit points and re-add them to the event
      auto hitSet = fPatternHitSets.at(i).get();
      fHitAttr.Copy(*hitSet);
      hitSet->SetMarkerColor(GetTrackColor(i));
      SetPointsFromTrack(*hitSet, tracks[i]);
      fEvePatternEvent->AddElement(hitSet);

      // Get the pattern and add it to the event
      auto pattern = tracks[i].GetPattern()->GetEveElement();
      pattern->SetDestroyOnZeroRefCnt(false);
      pattern->SetMainColor(GetTrackColor(i));
      fEvePatternEvent->AddElement(pattern);
   }

   fCvsDeDx->GetListOfPrimitives()->Remove(fDeDxGraphs);
   fCvsQvS->GetListOfPrimitives()->Remove(fHistQvS);
   fCvsQvS->GetListOfPrimitives()->Remove(fFittedBraggCurveGraph);
   if (tracks.size()) {
      fTrackIdx = 0;
      Draw3DBraggCurve(tracks[fTrackIdx]);
   } else {
      fTrackIdx = -1;
      DrawDeDxGraphs();
      DrawHistQvS();
   }
}

void AtTab3DBraggCurve::UpdateEventElements()
{
   auto fEvent = GetFairRootInfo<AtEvent>();
   if (fEvent == nullptr) {
      LOG(debug) << "Cannot update AtEvent elements: no event available";
      return;
   }

   auto &hits = fEvent->GetHits();
   LOG(info) << cBLUE << " Number of hits : " << hits.size() << " in " << fEvent->GetEventID() << cNORMAL;

   SetPointsFromHits(*fHitSet, hits);
}

void AtTab3DBraggCurve::SetPointsFromHits(TEvePointSet &hitSet, const std::vector<std::unique_ptr<AtHit>> &hits)
{
   SetPointsFromHits(hitSet, ContainerManip::GetPointerVector(hits));
}

void AtTab3DBraggCurve::SetPointsFromHits(TEvePointSet &hitSet, const std::vector<AtHit *> &hits)
{
   Int_t nHits = hits.size();

   hitSet.Reset(nHits);
   hitSet.SetOwnIds(true);
   fHitAttr.Copy(hitSet); // Copy attributes from fHitAttr into hitSet.

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      auto &hit = *hits.at(iHit);
      Int_t PadMultHit = 0;
      if (GetFairRootInfo<AtEvent>())
         PadMultHit = GetFairRootInfo<AtEvent>()->GetHitPadMult(hit.GetPadNum());

      if (hit.GetCharge() < fThreshold || PadMultHit > fMaxHitMulti)
         continue;

      auto position = hit.GetPosition();

      hitSet.SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      hitSet.SetPointId(new TNamed(Form("Hit %d", iHit), ""));
   }

   gEve->ElementChanged(&hitSet);
}

void AtTab3DBraggCurve::SetPointsFromTrack(TEvePointSet &hitSet, const AtTrack &track)
{
   Int_t nHits = track.GetHitArray().size();

   hitSet.Reset(nHits);
   hitSet.SetOwnIds(true);

   for (Int_t i = 0; i < nHits; i++) {

      auto &hit = *track.GetHitArray()[i];

      if (hit.GetCharge() < fThreshold)
         continue;

      auto position = hit.GetPosition();
      hitSet.SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      hitSet.SetPointId(new TNamed(Form("Hit %d", i), ""));
   }

   gEve->ElementChanged(&hitSet);
}

void AtTab3DBraggCurve::MakeTab(TEveWindowSlot *slot)
{
   TEveWindowPack *pack = nullptr;

   // 3D
   pack = slot->MakePack();
   pack->SetElementName("3DBraggCurve");
   pack->SetHorizontal();
   // pack->SetVertical();
   pack->SetShowTitleBar(kFALSE);

   pack->NewSlot()->MakeCurrent();
   TEveViewer *view3D = gEve->SpawnNewViewer("3D View", "");
   view3D->AddScene(gEve->GetGlobalScene());
   view3D->AddScene(gEve->GetEventScene());
   // }

   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsDeDx = new TCanvas("dEdx Bragg curve Canvas");
   fCvsDeDx->ToggleEditor();
   slot->StopEmbedding();

   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("3D Bragg curve Canvas");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsQvS = ecvs->GetCanvas();
   // fCvsQvS->AddExec("ex", "AtTab3DBraggCurve::NextTrack()");

   fCvsDeDx->SetName("dEdx Bragg curve Canvas");
   DrawDeDxGraphs();

   fCvsQvS->ToggleEventStatus();
   DrawHistQvS();

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

void AtTab3DBraggCurve::DrawDeDxGraphs()
{

   if (fDeDxGraphs != nullptr)
      fCvsDeDx->GetListOfPrimitives()->Remove(fDeDxGraphs);

   fDeDxGraphs = new TMultiGraph();
   fCvsDeDx->cd();
   fDeDxGraphs->Draw("A pmc plc");
   Double_t maxLength = TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2));
   fDeDxGraphs->GetXaxis()->SetTitle("ArchLength [cm]");
   fDeDxGraphs->GetYaxis()->SetTitle("#frac{dE}{dx} [MeV cm^{-1}]");
   fDeDxGraphs->GetXaxis()->SetLimits(0, maxLength / 10);
   fDeDxGraphs->SetMinimum(0);
   fDeDxGraphs->SetMaximum(50);
   fCvsDeDx->Modified();
   fCvsDeDx->Update();
}

void AtTab3DBraggCurve::DrawHistQvS()
{

   if (fHistQvS != nullptr)
      fCvsQvS->GetListOfPrimitives()->Remove(fHistQvS);

   Double_t maxLength = TMath::Sqrt(std::pow(1000., 2) + std::pow(250., 2));
   int nSBins = std::ceil(maxLength / fDS);

   fHistQvS = new TH1F("Charge vs archLength", "Charge vs archLength", nSBins, 0, nSBins * fDS);
   fHistQvS->SetDirectory(0);
   fCvsQvS->cd();
   fHistQvS->Draw();
   fHistQvS->GetXaxis()->SetTitle("ArchLength [mm]");
   fHistQvS->GetYaxis()->SetTitle("Charge [ADC]");
   fCvsQvS->Modified();
   fCvsQvS->Update();
}

void AtTab3DBraggCurve::Draw3DBraggCurve(AtTrack &track)
{
   auto *pattern = dynamic_cast<const AtPatterns::AtPatternLine *>(track.GetPattern());
   Double_t theta =
      TMath::ATan2(std::abs(pattern->GetDirection().Z()),
                   TMath::Sqrt(std::pow(pattern->GetDirection().X(), 2) + std::pow(pattern->GetDirection().Y(), 2)));
   fDS = fDZ / TMath::Cos(theta);

   DrawHistQvS();

   std::vector<std::pair<Double_t, Double_t>> archLengthVsELossValues = *(track.Get3DBraggCurveValues());

   std::cout << "Number of pairs: " << archLengthVsELossValues.size() << std::endl;

   for (std::pair<Double_t, Double_t> pairBragg : archLengthVsELossValues)
      fHistQvS->Fill(pairBragg.first, pairBragg.second);

   for (Int_t i = 1; i <= fHistQvS->GetNbinsX(); i++)
      // fHistQvS->SetBinError(i, TMath::Sqrt(fHistQvS->GetBinContent(i)));
      fHistQvS->SetBinError(i, 0.1 * fHistQvS->GetBinContent(i));

   auto fitResult = track.Get3DBraggFitResult();
   if (fitResult != nullptr) {
      auto bestIdx = fitResult->GetBestFitIndex();
      // bestIdx = 0;

      std::cout << "Best fitting index = " << bestIdx << std::endl;

      for (Int_t i = 0; i < fitResult->GetNumberOfEntries(); i++) {
         std::cout << "\nK_" << i << "    = " << fitResult->GetKineticEnergy(i) << " MeV" << std::endl;
         std::cout << "ampl_" << i << " = " << fitResult->GetAmplitudeFactor(i) << " ADC/MeV" << std::endl;
         std::cout << "chi2_" << i << " = " << fitResult->GetChi2(i) << std::endl;
      }

      auto ELoss = fitResult->GetELoss(bestIdx);
      auto amplitudeFactor = fitResult->GetAmplitudeFactor(bestIdx);
      fFittedBraggCurveGraph = new TGraph();
      for (auto pair : ELoss)
         fFittedBraggCurveGraph->AddPoint(pair.first, amplitudeFactor * pair.second);
      fFittedBraggCurveGraph->Draw("same");
   }

   DrawDeDxGraphs();
   if (fitResult != nullptr && fFitter != nullptr) {

      for (Int_t i = 0; i < fitResult->GetNumberOfEntries(); i++) {
         fFitter->SetProjectileIndex(i);
         auto dEdx = fFitter->GetDeDx(fitResult->GetKineticEnergy(i));
         TGraph *g = new TGraph();
         for (auto pair : dEdx)
            g->AddPoint(pair.first, pair.second);
         g->SetTitle(TString::Format("Particle %d", i));
         fDeDxGraphs->Add(g, "PL");
      }
      fCvsDeDx->BuildLegend(0.1, 0.9, 0.2, 0.5);
   }

   fCvsDeDx->Modified();
   fCvsDeDx->Update();
   fCvsQvS->Modified();
   fCvsQvS->Update();
}
