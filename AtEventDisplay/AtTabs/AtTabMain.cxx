#include "AtTabMain.h"

#include "AtContainerManip.h" // for GetPointerVector
#include "AtEvent.h"          // for AtEvent, AtEvent::HitVector
#include "AtHit.h"            // for AtHit, AtHit::XYZPoint
#include "AtMap.h"            // for AtMap
#include "AtPad.h"            // for AtPad
#include "AtPadReference.h"   // for operator<<
#include "AtPattern.h"        // for AtPattern
#include "AtPatternEvent.h"   // for AtPatternEvent
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
#include <TH1.h>                 // for TH1I
#include <TH2Poly.h>             // for TH2Poly
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

ClassImp(AtTabMain);

AtTabMain::AtTabMain() : AtTabBase("Main")
{
   if (AtViewerManager::Instance() == nullptr)
      throw "AtViewerManager must be initialized before creating tabs!";

   fPadNum = &AtViewerManager::Instance()->GetPadNum();
   fPadNum->Attach(this);

   fEventBranch = &AtViewerManager::Instance()->GetEventBranch();
   fEventBranch->Attach(this);

   fRawEventBranch = &AtViewerManager::Instance()->GetRawEventBranch();
   fRawEventBranch->Attach(this);

   fPatternEventBranch = &AtViewerManager::Instance()->GetPatternEventBranch();
   fPatternEventBranch->Attach(this);

   fEntry = &AtViewerManager::Instance()->GetCurrentEntry();
   fEntry->Attach(this);
};

AtTabMain::~AtTabMain()
{
   fPadNum->Detach(this);
   fEventBranch->Detach(this);
   fRawEventBranch->Detach(this);
   fPatternEventBranch->Detach(this);
   fEntry->Detach(this);
}

void AtTabMain::InitTab()
{
   std::cout << " =====  AtTabMain::Init =====" << std::endl;

   gEve->AddEvent(fEveEvent.get());
   fEveEvent->AddElement(fHitSet.get());

   gEve->AddEvent(fEvePatternEvent.get());

   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtEvent>>(*fEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtRawEvent>>(*fRawEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtPatternEvent>>(*fPatternEventBranch));

   gStyle->SetPalette(55);

   std::cout << " AtTabMain::Init : Initialization complete! "
             << "\n";
}

void AtTabMain::ExpandNumPatterns(int num)
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

Color_t AtTabMain::GetTrackColor(int i)
{
   std::vector<Color_t> colors = {kBlue - 7,    kGreen - 8, kOrange, kViolet, kYellow, kTeal - 6,
                                  kMagenta + 1, kBlue,      kViolet, kYellow, kCyan};
   if (i < colors.size()) {
      return colors.at(i);
   } else
      return kAzure;
}

void AtTabMain::Update(DataHandling::AtSubject *sub)
{
   // If we should update the stuff that depends on the AtEvent
   if (sub == fEventBranch || sub == fEntry) {
      UpdatePadPlane();
      UpdateEventElements();
   }
   if (sub == fPatternEventBranch || sub == fEntry) {
      UpdatePatternEventElements();
   }
   if (sub == fRawEventBranch || sub == fEntry || sub == fPadNum) {
      DrawWave(fPadNum->Get());
   }

   // If we should update the 3D display
   if (sub == fEventBranch || sub == fPatternEventBranch || sub == fEntry) {
      gEve->Redraw3D(false); // false -> don't reset camera
   }
}

void AtTabMain::DrawPadPlane()
{
   if (fPadPlane) {
      fPadPlane->Reset(nullptr);
      return;
   }

   AtViewerManager::Instance()->GetMap()->GeneratePadPlane();
   fPadPlane = AtViewerManager::Instance()->GetMap()->GetPadPlane();
   fPadPlane->SetBit(TH1::kNoTitle);
   fCvsPadPlane->cd();
   fPadPlane->Draw("COL L0");
   fPadPlane->SetMinimum(1.0);
   gStyle->SetOptStat(0);
   gStyle->SetPalette(103);
   gPad->Update();
}

void AtTabMain::DrawPadWave()
{
   char name[20];
   sprintf(name, "fPadWave_DT%i", fTabId);
   fPadWave = new TH1I(name, name, 512, 0, 511);
   fPadWave->SetBit(TH1::kNoTitle);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

void AtTabMain::DumpEvent(std::string fileName)
{
   auto fEvent = GetFairRootInfo<AtEvent>();
   if (fEvent == nullptr) {
      std::cout << "CRITICAL ERROR: Event missing for TabMain. Aborting draw." << std::endl;
      return;
   }

   std::ofstream dumpEvent;
   dumpEvent.open(fileName);
   dumpEvent << " Event ID : " << fEvent->GetEventID() << std::endl;

   for (auto &hit : fEvent->GetHits())
      dumpEvent << hit->GetPosition().X() << "," << hit->GetPosition().Y() << "," << hit->GetPosition().Z() << ","
                << hit->GetCharge() << std::endl;

   dumpEvent.close();
}

void AtTabMain::UpdateRenderState()
{
   fEveEvent->SetRnrState(true);
   fEvePatternEvent->SetRnrState(false);
}

void AtTabMain::UpdatePatternEventElements()
{
   if (fEvePatternEvent == nullptr)
      return;

   auto fPatternEvent = GetFairRootInfo<AtPatternEvent>();
   if (fPatternEvent == nullptr) {
      LOG(debug) << "Cannot update AtPatternEvent elements: no event availible";
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
}

void AtTabMain::UpdateEventElements()
{
   auto fEvent = GetFairRootInfo<AtEvent>();
   if (fEvent == nullptr) {
      LOG(debug) << "Cannot update AtEvent elements: no event availible";
      return;
   }

   auto &hits = fEvent->GetHits();
   LOG(info) << cBLUE << " Number of hits : " << hits.size() << " in " << fEvent->GetEventID() << cNORMAL;

   SetPointsFromHits(*fHitSet, hits);
}

void AtTabMain::UpdatePadPlane()
{

   if (fPadPlane)
      fPadPlane->Reset(nullptr);
   else
      return;

   LOG(debug) << "Updating pad plane ";

   auto fEvent = GetFairRootInfo<AtEvent>();
   if (fEvent == nullptr) {
      LOG(debug) << "Cannot fill pad plane histogram: no event availible";
      return;
   }
   auto &hits = fEvent->GetHits();

   for (auto &hit : hits) {
      int padMultiHit = GetFairRootInfo<AtEvent>()->GetHitPadMult(hit->GetPadNum());
      if (hit->GetCharge() < fThreshold || padMultiHit > fMaxHitMulti)
         continue;
      auto position = hit->GetPosition();
      fPadPlane->Fill(position.X(), position.Y(), hit->GetCharge());
   }

   fCvsPadPlane->Modified();
   fCvsPadPlane->Update();
}
void AtTabMain::SetPointsFromHits(TEvePointSet &hitSet, const std::vector<std::unique_ptr<AtHit>> &hits)
{
   SetPointsFromHits(hitSet, ContainerManip::GetPointerVector(hits));
}

void AtTabMain::SetPointsFromHits(TEvePointSet &hitSet, const std::vector<AtHit *> &hits)
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

void AtTabMain::SetPointsFromTrack(TEvePointSet &hitSet, const AtTrack &track)
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

bool AtTabMain::DrawWave(Int_t PadNum)
{
   fPadWave->Reset();
   auto fRawEvent = GetFairRootInfo<AtRawEvent>();

   // std::cout << "checking fRawEvent" << std::endl;
   if (fRawEvent == nullptr) {
      return false;
   }
   // std::cout << "fRawEvent is not nullptr" << std::endl;
   AtPad *fPad = fRawEvent->GetPad(PadNum);
   if (fPad == nullptr) {
      LOG(error) << "Pad in event is null!";
      return false;
   } else
      LOG(debug) << "Drawing pad " << fPad->GetPadNum();

   auto adc = fPad->GetADC();

   for (Int_t i = 0; i < 512; i++) {
      fPadWave->SetBinContent(i, adc[i]);
   }

   fCvsPadWave->cd();
   fPadWave->Draw();
   fCvsPadWave->Modified();
   fCvsPadWave->Update();

   return true;
}

void AtTabMain::MakeTab(TEveWindowSlot *slot)
{
   TEveWindowPack *pack = nullptr;

   // 3D
   pack = slot->MakePack();
   pack->SetElementName("Main");
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
   fCvsPadWave = new TCanvas("AtPad Canvas");
   fCvsPadWave->ToggleEditor();
   slot->StopEmbedding();

   // Pad Plane
   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("AtTPC Pad Plane");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsPadPlane = ecvs->GetCanvas();
   fCvsPadPlane->AddExec("ex", "AtTabMain::SelectPad()");

   fCvsPadWave->SetName(TString::Format("fCvsPadWave_DT%i", fTabId));
   DrawPadWave();

   fCvsPadPlane->ToggleEventStatus();
   DrawPadPlane();

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

void AtTabMain::SelectPad()
{

   // Return earlyt if this was not triggered by a left mouse click.
   if (gPad->GetEvent() != 11)
      return;
   auto *h = dynamic_cast<TH2Poly *>(gPad->GetSelected());
   if (!h)
      return;

   gPad->GetCanvas()->FeedbackMode(true);

   int pyold = gPad->GetUniqueID();
   int px = gPad->GetEventX();
   int py = gPad->GetEventY();
   float uxmin = gPad->GetUxmin();
   float uxmax = gPad->GetUxmax();
   int pxmin = gPad->XtoAbsPixel(uxmin);
   int pxmax = gPad->XtoAbsPixel(uxmax);
   if (pyold)
      gVirtualX->DrawLine(pxmin, pyold, pxmax, pyold);
   gVirtualX->DrawLine(pxmin, py, pxmax, py);
   gPad->SetUniqueID(py);
   Float_t upx = gPad->AbsPixeltoX(px);
   Float_t upy = gPad->AbsPixeltoY(py);
   Double_t x = gPad->PadtoX(upx);
   Double_t y = gPad->PadtoY(upy);
   Int_t bin = h->FindBin(x, y);
   const char *bin_name = h->GetBinName(bin);
   LOG(debug) << " ==========================";
   LOG(debug) << " Bin number selected : " << bin << " Bin name :" << bin_name;

   AtMap *tmap = AtViewerManager::Instance()->GetMap();
   if (tmap == nullptr) {
      LOG(fatal) << "AtMap not set! Pass a valid map to the constructor of AtViewerManager!";
   } else {
      Int_t tPadNum = tmap->BinToPad(bin);
      LOG(debug) << " Bin : " << bin << " to Pad : " << tPadNum;
      LOG(debug) << " Electronic mapping: " << tmap->GetPadRef(tPadNum);
      LOG(debug) << " Raw Event Pad Num " << tPadNum;
      AtViewerManager::Instance()->GetPadNum().Set(tPadNum);
   }
}
