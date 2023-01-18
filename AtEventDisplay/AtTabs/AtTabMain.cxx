#include "AtTabMain.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"
#include "AtHit.h" // for AtHit
#include "AtMap.h"
#include "AtPad.h" // for AtPad
#include "AtRawEvent.h"
#include "AtTabInfo.h"

#include <FairLogger.h> // for LOG

#include <Math/Point3D.h> // for PositionVector3D
#include <TAttMarker.h>   // for kFullDotMedium
#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveGeoNode.h>
#include <TEveManager.h>
#include <TEvePointSet.h>  // for TEvePointSet
#include <TEveTreeTools.h> // for TEvePointSelectorConsumer, TEvePoint...
#include <TEveViewer.h>
#include <TEveWindow.h>
#include <TGLCamera.h> // for TGLCamera
#include <TGLViewer.h>
#include <TGTab.h>
#include <TGeoManager.h>
#include <TGeoVolume.h>
#include <TH1.h> // for TH1I
#include <TH2Poly.h>
#include <TNamed.h> // for TNamed
#include <TROOT.h>  // for TROOT, gROOT
#include <TRandom.h>
#include <TRootEmbeddedCanvas.h>
#include <TSeqCollection.h> // for TSeqCollection
#include <TStyle.h>
#include <TVirtualPad.h> // for TVirtualPad, gPad

#include <array>              // for array
#include <cstdio>             // for sprintf
#include <ext/alloc_traits.h> // for __alloc_traits<>::value_type
#include <iostream>           // for operator<<, endl, basic_ostream
#include <utility>            // for move
class TGeoNode;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabMain);

AtTabMain::AtTabMain()
   : fThreshold(0), fHitSet(nullptr), fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium), fCvsPadPlane(nullptr),
     fPadPlane(nullptr), fCvsPadWave(nullptr), fPadWave(nullptr), fMultiHit(0)
{
}

void AtTabMain::InitTab()
{

   std::cout << " =====  AtTabMain::Init =====" << std::endl;

   auto tTabInfoEvent = std::make_unique<AtTabInfoFairRoot<AtEvent>>();
   auto tTabInfoRawEvent = std::make_unique<AtTabInfoFairRoot<AtRawEvent>>();

   fTabInfo->AddAugment(fInfoEventName, std::move(tTabInfoEvent));
   fTabInfo->AddAugment(fInfoRawEventName, std::move(tTabInfoRawEvent));

   gStyle->SetPalette(55);

   std::cout << " AtTabMain::Init : Initialization complete! "
             << "\n";
}

void AtTabMain::MakeTab()
{
   TEveWindowSlot *slot = nullptr;
   TEveWindowPack *pack = nullptr;

   // 3D
   slot = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
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
   fCvsPadPlane->AddExec("ex", "AtEventManagerNew::SelectPad()");

   fCvsPadWave->SetName(TString::Format("fCvsPadWave_DT%i", fTabNumber));
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
}

void AtTabMain::DrawEvent()
{
   LOG(debug) << "Drawing tab " << fTabNumber;
   DrawHitPoints();
   gEve->Redraw3D(false);
   UpdateCvsPadPlane();
   LOG(debug) << "Done Drawing tab " << fTabNumber << std::endl;
}

void AtTabMain::DrawPad(Int_t PadNum)
{
   LOG(debug) << "Drawing main pad " << fTabNumber;
   DrawWave(PadNum);
   UpdateCvsPadWave();
   LOG(debug) << "Done Drawing main pad " << fTabNumber;
}

void AtTabMain::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

void AtTabMain::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

void AtTabMain::Reset()
{
   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, AtEventManagerNew::Instance());
   }

   if (fPadPlane != nullptr)
      fPadPlane->Reset(nullptr);
}

void AtTabMain::DrawPadPlane()
{
   if (fPadPlane) {
      fPadPlane->Reset(nullptr);
      return;
   }

   AtEventManagerNew::Instance()->GetMap()->GeneratePadPlane();
   fPadPlane = AtEventManagerNew::Instance()->GetMap()->GetPadPlane();
   fCvsPadPlane->cd();
   // fPadPlane -> Draw("COLZ L0"); //0  == bin lines adre not drawn
   fPadPlane->Draw("COL L0");
   fPadPlane->SetMinimum(1.0);
   gStyle->SetOptStat(0);
   gStyle->SetPalette(103);
   gPad->Update();
}

void AtTabMain::DrawPadWave()
{
   char name[20];
   sprintf(name, "fPadWave_DT%i", fTabNumber);
   fPadWave = new TH1I(name, name, 512, 0, 511);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

AtEvent *AtTabMain::GetEvent()
{
   return dynamic_cast<AtTabInfoFairRoot<AtEvent> *>(fTabInfo->GetAugment(fInfoEventName))->GetInfo();
}
AtRawEvent *AtTabMain::GetRawEvent()
{
   auto info = dynamic_cast<AtTabInfoFairRoot<AtRawEvent> *>(fTabInfo->GetAugment(fInfoRawEventName));
   LOG(debug) << "Getting raw event from " << info->GetBranch();
   return info->GetInfo();
}

void AtTabMain::DrawHitPoints()
{
   auto fEvent = GetEvent();
   if (fEvent == nullptr) {
      std::cout << "CRITICAL ERROR: Event missing for TabMain. Aborting draw." << std::endl;
      return;
   }
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   LOG(debug) << "Drawing 3D cloud for " << fEvent->GetEventID();

   Int_t eventID = fEvent->GetEventID();
   TString TSevt = " Event ID : ";
   TString TSpad = " Pad ID : ";
   dumpEvent << TSevt << eventID << std::endl;

   //////////////////////////////////////////////

   auto &hits = fEvent->GetHits();
   LOG(info) << cBLUE << " Number of hits : " << hits.size() << cNORMAL;

   FillPadPlane(hits);
   fHitSet = GetPointsFromHits(hits);
   // Adding raw data points
   gEve->AddElement(fHitSet);

   dumpEvent.close();
}
void AtTabMain::FillPadPlane(const std::vector<std::unique_ptr<AtHit>> &hits)
{
   for (auto &hit : hits) {
      int padMultiHit = GetEvent()->GetHitPadMult(hit->GetPadNum());
      if (hit->GetCharge() < fThreshold || padMultiHit > fMultiHit)
         continue;
      auto position = hit->GetPosition();
      fPadPlane->Fill(position.X(), position.Y(), hit->GetCharge());
   }
}

TEvePointSet *AtTabMain::GetPointsFromHits(const std::vector<std::unique_ptr<AtHit>> &hits)
{
   Int_t nHits = hits.size();
   // auto hitSet = std::make_unique<TEvePointSet>("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   auto hitSet = new TEvePointSet("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   hitSet->SetOwnIds(kTRUE);
   hitSet->SetMarkerColor(fHitColor);
   hitSet->SetMarkerSize(fHitSize);
   hitSet->SetMarkerStyle(fHitStyle);
   LOG(info) << cBLUE << " Number of hits : " << nHits << cNORMAL;

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *hits.at(iHit);
      Int_t PadNumHit = hit.GetPadNum();
      Int_t PadMultHit = GetEvent()->GetHitPadMult(PadNumHit);

      if (hit.GetCharge() < fThreshold)
         continue;
      if (PadMultHit > fMultiHit)
         continue;

      auto position = hit.GetPosition();

      hitSet->SetMarkerColor(fHitColor);
      hitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      hitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
   }

   return hitSet;
}

bool AtTabMain::DrawWave(Int_t PadNum)
{
   auto fRawEvent = GetRawEvent();

   // std::cout << "checking fRawEvent" << std::endl;
   if (fRawEvent == nullptr) {
      std::cout << "fRawEvent is NULL!" << std::endl;
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
   fPadWave->Reset();
   for (Int_t i = 0; i < 512; i++) {
      fPadWave->SetBinContent(i, adc[i]);
   }

   fCvsPadWave->cd();
   fPadWave->Draw();
   fCvsPadWave->Update();
   return true;
}

void AtTabMain::UpdateCvsPadPlane()
{
   fCvsPadPlane->Modified();
   fCvsPadPlane->Update();
}

void AtTabMain::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();
}
