#include "AtEventTabMain.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"
#include "AtMap.h"
#include "AtRawEvent.h"

#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveGeoNode.h>
#include <TEveManager.h>
#include <TEvePointSet.h>  // for TEvePointSet
#include <TEveTreeTools.h> // for TEvePointSelectorConsumer, TEvePoint...
#include <TEveViewer.h>
#include <TEveWindow.h>
#include <TGLViewer.h>
#include <TGTab.h>
#include <TGeoManager.h>
#include <TGeoVolume.h>
#include <TH2Poly.h>
#include <TROOT.h> // for TROOT, gROOT
#include <TRandom.h>
#include <TRootEmbeddedCanvas.h>
#include <TStyle.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtEventTabMain);

AtEventTabMain::AtEventTabMain()
   : fRawEvent(nullptr), fEvent(nullptr), fDetmap(nullptr), fThreshold(0), fHitSet(nullptr), fPadPlanePal(nullptr),
     fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium), fCvsPadPlane(nullptr), fPadPlane(nullptr),
     fCvsPadWave(nullptr), fPadWave(nullptr), fMultiHit(0)
{
}

void AtEventTabMain::Init()
{

   std::cout << " =====  AtEventTabMain::Init =====" << std::endl;

   // gROOT->Reset();
   fEventManager = AtEventManagerNew::Instance();

   if (fDetmap == nullptr) {
      LOG(fatal) << "Map was never set using the function SetMap() in AtEventDrawTaskNew!";
   }

   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap.get());

   gStyle->SetPalette(55);

   //******* NO CALLS TO TCANVAS BELOW THIS ONE

   std::cout << " AtEventTabMain::Init : Initialization complete! "
             << "\n";
}

void AtEventTabMain::MakeTab()
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
   sprintf(name, "fCvsPadWave_DT%i", fTaskNumber);
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

      // gGeoManager -> DefaultColors();
      // gGeoManager -> GetVolume("field_cage_in")     -> SetVisibility(kFALSE); //active
      gGeoManager->GetVolume("drift_volume")->SetTransparency(transparency);
      // gGeoManager -> GetVolume("cageSide")          -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("cageCorner")        -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("frontWindow")       -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("frontWindowFrame")  -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("frontWindowCradle") -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("bottomPlate")       -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("backWindowFrame")   -> SetTransparency(transparency);
      ////gGeoManager -> GetVolume("backWindow")        -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("topFrame")          -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("ribmain")           -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("wirePlane")         -> SetTransparency(transparency);
      // gGeoManager -> GetVolume("padPlane")          -> SetTransparency(transparency);

      gEve->FullRedraw3D(kTRUE);
      fEventManager->SetfEvent();
   }

   gEve->GetBrowser()->GetTabRight()->SetTab(1);

   gEve->Redraw3D(kTRUE, kTRUE);

   TGLViewer *dfViewer = gEve->GetDefaultGLViewer(); // Is this doing anything?
   dfViewer->CurrentCamera().RotateRad(-.7, 0.5);
   dfViewer->DoDraw();
}

void AtEventTabMain::DrawEvent(AtRawEvent *rawEvent, AtEvent *event)
{
   fRawEvent = rawEvent;
   fEvent = event;
   DrawHitPoints();
   gEve->Redraw3D(kFALSE);
   UpdateCvsPadPlane();
}

void AtEventTabMain::DrawPad(Int_t PadNum)
{
   DrawWave(PadNum);
   UpdateCvsPadWave();
}

void AtEventTabMain::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

void AtEventTabMain::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

void AtEventTabMain::Reset()
{
   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, fEventManager);
   }

   if (fPadPlane != nullptr)
      fPadPlane->Reset(nullptr);
}

void AtEventTabMain::DrawPadPlane()
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

void AtEventTabMain::DrawPadWave()
{
   char name[20];
   sprintf(name, "fPadWave_DT%i", fTaskNumber);
   fPadWave = new TH1I(name, name, 512, 0, 511);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

void AtEventTabMain::DrawHitPoints()
{
   if (fEvent == nullptr) {
      std::cout << "CRITICAL ERROR: Event missing for TabMain. Aborting draw." << std::endl;
      return;
   }
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

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
   std::cout << cBLUE << " Number of hits : " << nHits << cNORMAL << std::endl;

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

void AtEventTabMain::DrawWave(Int_t PadNum)
{
   std::cout << "checking fRawEvent" << std::endl;
   if (fRawEvent == nullptr) {
      std::cout << "fRawEvent is NULL!" << std::endl;
   } else {
      std::cout << "fRawEvent is not nullptr" << std::endl;
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

void AtEventTabMain::UpdateCvsPadPlane()
{
   fCvsPadPlane->Modified();
   fCvsPadPlane->Update();
}

void AtEventTabMain::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();
}
