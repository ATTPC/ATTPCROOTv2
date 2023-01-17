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
   : fRawEvent(nullptr), fEvent(nullptr), fDetmap(nullptr), fThreshold(0), fHitSet(nullptr), fPadPlanePal(nullptr),
     fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium), fCvsPadPlane(nullptr), fPadPlane(nullptr),
     fCvsPadWave(nullptr), fPadWave(nullptr), fMultiHit(0), fEventBranch("AtEvent"), fRawEventBranch("AtRawEvent"),
     fInfoEventName("AtEvent"), fInfoRawEventName("AtRawEvent")
{
}

void AtTabMain::InitTab()
{

   std::cout << " =====  AtTabMain::Init =====" << std::endl;

   // gROOT->Reset();
   fEventManager = AtEventManagerNew::Instance();

   if (fDetmap == nullptr) {
      LOG(fatal) << "Map was never set using the function SetMap() in AtEventDrawTaskNew!";
   }

   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap.get());

   auto tTabInfoEvent = std::make_unique<AtTabInfoFairRoot<AtEvent>>(fEventBranch);
   auto tTabInfoRawEvent = std::make_unique<AtTabInfoFairRoot<AtRawEvent>>(fRawEventBranch);

   fTabInfo->AddAugment(fInfoEventName, std::move(tTabInfoEvent));
   fTabInfo->AddAugment(fInfoRawEventName, std::move(tTabInfoRawEvent));

   gStyle->SetPalette(55);

   //******* NO CALLS TO TCANVAS BELOW THIS ONE

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
   pack->SetElementName("AtTPC 3D/Pad plane views");
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

   char name[20];
   sprintf(name, "fCvsPadWave_DT%i", fTabNumber);
   fCvsPadWave->SetName(name);
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

   gEve->Redraw3D(kTRUE, kTRUE);

   TGLViewer *dfViewer = gEve->GetDefaultGLViewer(); // Is this doing anything?
   dfViewer->CurrentCamera().RotateRad(-.7, 0.5);
   dfViewer->DoDraw();
}

void AtTabMain::UpdateTab()
{
   fEvent = dynamic_cast<AtTabInfoFairRoot<AtEvent> *>(fTabInfo->GetAugment(fInfoEventName))->GetInfo();
   fRawEvent = dynamic_cast<AtTabInfoFairRoot<AtRawEvent> *>(fTabInfo->GetAugment(fInfoRawEventName))->GetInfo();
   LOG(debug) << "Event ID " << fEvent->GetEventID() << " Raw Event ID " << fRawEvent->GetEventID();
}

void AtTabMain::DrawEvent()
{
   LOG(debug) << "Drawing tab " << fTabNumber;
   DrawHitPoints();
   gEve->Redraw3D(kFALSE);
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
      gEve->RemoveElement(fHitSet, fEventManager);
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

   fDetmap->GeneratePadPlane();
   fPadPlane = fDetmap->GetPadPlane();
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

void AtTabMain::DrawHitPoints()
{

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

   Int_t nHits = fEvent->GetNumHits();
   fHitSet = new TEvePointSet("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   fHitSet->SetOwnIds(kTRUE);
   fHitSet->SetMarkerColor(fHitColor);
   fHitSet->SetMarkerSize(fHitSize);
   fHitSet->SetMarkerStyle(fHitStyle);
   LOG(info) << cBLUE << " Number of hits : " << nHits << cNORMAL;

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *fEvent->GetHits().at(iHit);
      Int_t PadNumHit = hit.GetPadNum();
      Int_t PadMultHit = fEvent->GetHitPadMult(PadNumHit);

      if (hit.GetCharge() < fThreshold)
         continue;
      if (PadMultHit > fMultiHit)
         continue;
      auto position = hit.GetPosition();

      fHitSet->SetMarkerColor(fHitColor);
      fHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      fHitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
      fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());
   }

   // Adding raw data points
   gEve->AddElement(fHitSet);

   dumpEvent.close();
}

void AtTabMain::DrawWave(Int_t PadNum)
{
   //std::cout << "checking fRawEvent" << std::endl;
   if (fRawEvent == nullptr) {
      std::cout << "fRawEvent is NULL!" << std::endl;
   } else {
      //std::cout << "fRawEvent is not nullptr" << std::endl;
      AtPad *fPad = fRawEvent->GetPad(PadNum);
      if (fPad == nullptr)
         return;
      auto rawadc = fPad->GetRawADC();
      auto adc = fPad->GetADC();
      fPadWave->Reset();
      for (Int_t i = 0; i < 512; i++) {
         fPadWave->SetBinContent(i, adc[i]);
      }

      fCvsPadWave->cd();
      fPadWave->Draw();
      fCvsPadWave->Update();
   }
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
