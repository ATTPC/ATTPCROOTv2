
#include "AtEventManager.h"

#include "AtEvent.h" // for AtEvent

#include <FairRootManager.h>
#include <FairRunAna.h>

#include <Rtypes.h>
#include <TCanvas.h>
#include <TChain.h>
#include <TClonesArray.h> // for TClonesArray
#include <TEveBrowser.h>
#include <TEveEventManager.h>
#include <TEveGeoNode.h>
#include <TEveManager.h>
#include <TEveViewer.h>
#include <TEveWindow.h>
#include <TFile.h>
#include <TGButton.h>
#include <TGClient.h>
#include <TGFrame.h>
#include <TGLCamera.h>
#include <TGLViewer.h>
#include <TGLabel.h>
#include <TGLayout.h>
#include <TGNumberEntry.h>
#include <TGTab.h>
#include <TGWindow.h>
#include <TGeoManager.h>
#include <TGeoVolume.h>
#include <TH2.h>
#include <TH2Poly.h>
#include <TObject.h>
#include <TRootBrowser.h>
#include <TRootEmbeddedCanvas.h>
#include <TString.h>
#include <TStyle.h>
#include <TSystem.h>
#include <TVirtualPad.h>
#include <TVirtualX.h>

#include "S800Ana.h"
#include "S800Calc.h"

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";
constexpr auto cWHITERED = "\033[37;41m";

#include <iostream>
#include <string>

class TGeoNode;

using namespace std;

ClassImp(AtEventManager);

AtEventManager *AtEventManager::fInstance = nullptr;
AtEventManager *AtEventManager::Instance()
{
   return fInstance;
}

AtEventManager::AtEventManager()
   : TEveEventManager("AtEventManager", ""), fRootManager(FairRootManager::Instance()), fRunAna(FairRunAna::Instance()),
     fEntry(0), fEvent(nullptr), fCurrentEvent(nullptr), f3DThresDisplay(nullptr), fCvsPadPlane(nullptr),
     fPadWave(nullptr), fPadAll(nullptr), fCvsQEvent(nullptr), fCvsHough(nullptr), fCvsRad(nullptr),
     drawallpad(nullptr), eraseQevent(nullptr), drawReconstruction(nullptr), saveASCIIevent(nullptr),
     toggleCorr(nullptr), kDrawAllOn(false), kEraseQ(false), kDrawReconstruction(false), kDraw3DGeo(false),
     kDraw3DHist(false), kToggleData(false), k3DThreshold(0), fTofObjCorr(0), fMTDCObjRange(0), fMTDCXfRange(0),
     cArray(nullptr), cS800Array(nullptr), cevent(nullptr), cS800Calc(nullptr), fCvsPIDFull(nullptr), fCvsPID(nullptr),
     fCvsPID2(nullptr), fPIDFull(nullptr), fCvsPID2Full(nullptr), fPID2Full(nullptr), fEntries(0)

{
   fInstance = this;
}

AtEventManager::~AtEventManager() = default;

/*void
AtEventManager::InitRiemann(Int_t option, Int_t level, Int_t nNodes)
{
  TEveManager::Create();
  fRunAna->Init();
  fEvent= gEve->AddEvent(this);
}*/

void AtEventManager::Init(Int_t option, Int_t level, Int_t nNodes)
{

   gStyle->SetOptTitle(0);
   // gStyle->SetCanvasPreferGL(kTRUE);
   gStyle->SetPalette(55);
   TEveManager::Create();

   Int_t dummy;
   UInt_t width, height;
   UInt_t widthMax = 1400, heightMax = 650;
   // Double_t ratio = (Double_t)widthMax / heightMax;
   gVirtualX->GetWindowSize(gClient->GetRoot()->GetId(), dummy, dummy, width, height);
   // Assume that width of screen is always larger than the height of screen
   /*
      if (width > widthMax) {
         width = widthMax;
         height = heightMax;
      } else
         height = (Int_t)(width / ratio);
   */
   // gEve->GetMainWindow()->Resize(width,height);

   /**************************************************************************/

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
   // if(kDraw3DGeo){
   TEveViewer *view3D = gEve->SpawnNewViewer("3D View", "");
   view3D->AddScene(gEve->GetGlobalScene());
   view3D->AddScene(gEve->GetEventScene());
   // }

   // Old arrangement

   /* slot = pack->NewSlotWithWeight(1.5);
    TRootEmbeddedCanvas* ecvs = new TRootEmbeddedCanvas();
    TEveWindowFrame* frame = slot->MakeFrame(ecvs);
    frame->SetElementName("AtTPC Pad Plane");
    pack->GetEveFrame()->SetShowTitleBar(kFALSE);
    fCvsPadPlane = ecvs->GetCanvas();*/

   // New arrangement

   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fPadWave = new TCanvas("AtPad Canvas");
   fPadWave->ToggleEditor();
   slot->StopEmbedding();

   /*slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsPadPlane = new TCanvas("AtPoly");
   fPadWave->ToggleEditor();
   slot->StopEmbedding();
   fCvsPadPlane->AddExec("ex","AtEventManager::DrawWave()");*/

   // Pad Plane
   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("AtTPC Pad Plane");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsPadPlane = ecvs->GetCanvas();
   // fCvsPadPlane->AddExec("ex","AtEventManager::DrawWave()"); //OBSOLETE DO NOT USE

   // A test
   /*slot = pack2->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs2 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame2 = slot->MakeFrame(ecvs2);
   frame2->SetElementName("AtTPC Pad Plane All");
   fPadAll = ecvs2->GetCanvas();*/

   // Sixth tab 3D Histogram
   TEveWindowSlot *slot5 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack6 = slot5->MakePack();
   pack6->SetShowTitleBar(kFALSE);
   pack6->SetElementName("3D Histogram View");
   slot5 = pack6->NewSlotWithWeight(1.5);
   auto *ecvs6 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame6 = slot5->MakeFrame(ecvs6);
   frame6->SetElementName("3D Histogram View");
   fCvs3DHist = ecvs6->GetCanvas();

   // Third tab
   TEveWindowSlot *slot2 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack3 = slot2->MakePack();
   pack3->SetShowTitleBar(kFALSE);
   pack3->SetElementName("Pad plane raw signals");

   slot2 = pack3->NewSlotWithWeight(1.5);
   auto *ecvs3 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame3 = slot2->MakeFrame(ecvs3);
   frame3->SetElementName("AtTPC Pad Plane All");
   fPadAll = ecvs3->GetCanvas();

   slot2 = pack3->NewSlotWithWeight(1.5);
   auto *ecvs31 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame31 = slot2->MakeFrame(ecvs31);
   frame31->SetElementName("AtTPC Mesh");
   fCvsMesh = ecvs31->GetCanvas();

   /* TEveWindowSlot* slot2 =
    TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
    TEveWindowPack* pack3;
    pack3 = slot2->MakePack();
    //pack3->SetShowTitleBar(kFALSE);
    pack3->SetHorizontal();
    pack3->SetElementName("Pad Plane Raw Signals");
    TRootEmbeddedCanvas* ecvs2 = new TRootEmbeddedCanvas();


    //TEveWindowFrame* frame2 = slot2->MakeFrame(ecvs2);



    fPadAll = ecvs2->GetCanvas();*/

   // fPadAll = new TCanvas();

   // Forth tab Reconstruction
   TEveWindowSlot *slot3 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack4 = slot3->MakePack();
   pack4->SetShowTitleBar(kFALSE);
   pack4->SetHorizontal();
   pack4->SetElementName("Reconstruction");

   slot3 = pack4->NewSlotWithWeight(1.5);
   auto *ecvs4 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame4 = slot3->MakeFrame(ecvs4);
   frame4->SetElementName("Hough Space");
   fCvsHough = ecvs4->GetCanvas();

   slot3 = pack4->NewSlotWithWeight(1.5);
   auto *ecvs4_add = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame4_add = slot3->MakeFrame(ecvs4_add);
   frame4_add->SetElementName("Radius of curvature");
   fCvsRad = ecvs4_add->GetCanvas();

   slot3 = pack4->NewSlotWithWeight(1.5);
   auto *ecvs4_add2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame4_add2 = slot3->MakeFrame(ecvs4_add2);
   frame4_add2->SetElementName("Theta");
   fCvsTheta = ecvs4_add2->GetCanvas();

   slot3 = pack4->NewSlotWithWeight(1.5);
   auto *ecvs4_add3 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame4_add3 = slot3->MakeFrame(ecvs4_add3);
   frame4_add3->SetElementName("Theta X Phi");
   fCvsThetaxPhi = ecvs4_add3->GetCanvas();

   ///////////
   /*  TEveWindowSlot* slotH =
     TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
     TEveWindowPack* packH = slotH->MakePack();
     packH->SetShowTitleBar(kFALSE);
     packH->SetHorizontal();
     packH->SetElementName("Reconstruction (Prototype)");

     slotH = packH->NewSlotWithWeight(1.5);
     TRootEmbeddedCanvas* ecvsH1 = new TRootEmbeddedCanvas();
     TEveWindowFrame* frameH1 = slotH->MakeFrame(ecvsH1);
     frameH1->SetElementName("Quadrant 1");
     fCvsQuadrant1 = ecvsH1->GetCanvas();

     slotH = packH->NewSlotWithWeight(1.5);
     TRootEmbeddedCanvas* ecvsH2 = new TRootEmbeddedCanvas();
     TEveWindowFrame* frameH2 = slotH->MakeFrame(ecvsH2);
     frameH2->SetElementName("Quadrant 2");
     fCvsQuadrant2 = ecvsH2->GetCanvas();

     slotH = packH->NewSlotWithWeight(1.5);
     TRootEmbeddedCanvas* ecvsH3 = new TRootEmbeddedCanvas();
     TEveWindowFrame* frameH3 = slotH->MakeFrame(ecvsH3);
     frameH3->SetElementName("Quadrant 3");
     fCvsQuadrant3 = ecvsH3->GetCanvas();

     slotH = packH->NewSlotWithWeight(1.5);
     TRootEmbeddedCanvas* ecvsH4 = new TRootEmbeddedCanvas();
     TEveWindowFrame* frameH4 = slotH->MakeFrame(ecvsH4);
     frameH4->SetElementName("Quadrant 4");
     fCvsQuadrant4 = ecvsH4->GetCanvas();*/

   /////////

   // Fifth tab Phi Reconstruction
   TEveWindowSlot *slot4 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack5 = slot4->MakePack();
   pack5->SetShowTitleBar(kFALSE);
   pack5->SetElementName("Prototype Phi Recons.");
   slot4 = pack5->NewSlotWithWeight(1.5);
   auto *ecvs5 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame5 = slot4->MakeFrame(ecvs5);
   frame5->SetElementName("Phi Reconstruction");
   fCvsPhi = ecvs5->GetCanvas();

   // Sixth tab Monte Carlo
   TEveWindowSlot *slotMC = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packMC = slotMC->MakePack();
   packMC->SetShowTitleBar(kFALSE);
   packMC->SetHorizontal();
   packMC->SetElementName("Monte Carlo");

   slotMC = packMC->NewSlotWithWeight(1.5);
   auto *ecvsMC_XY = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameMC_XY = slotMC->MakeFrame(ecvsMC_XY);
   frameMC_XY->SetElementName("XY Projection");
   fCvsMC_XY = ecvsMC_XY->GetCanvas();

   slotMC = packMC->NewSlotWithWeight(1.5);
   auto *ecvsMC_Z = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameMC_Z = slotMC->MakeFrame(ecvsMC_Z);
   frameMC_Z->SetElementName("Time Projection");
   fCvsMC_Z = ecvsMC_Z->GetCanvas();

   TEveWindowSlot *slotAux = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packAux = slotAux->MakePack();
   packAux->SetShowTitleBar(kFALSE);
   packAux->SetHorizontal();
   packAux->SetElementName("Auxiliary GET Channels");

   slotAux = packAux->NewSlotWithWeight(1.5);
   auto *ecvsAux1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameAux1 = slotAux->MakeFrame(ecvsAux1);
   frameAux1->SetElementName("Auxiliary GET Channels");
   fCvsAux = ecvsAux1->GetCanvas();

   // Seventh tab
   TEveWindowSlot *slotS800 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packPID = slotS800->MakePack();
   packPID->SetShowTitleBar(kFALSE);
   packPID->SetElementName("S800 PIDs");
   packPID->SetHorizontal();

   slotS800 = packPID->NewSlotWithWeight(1.5);
   auto ecvsS800tof1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameS800tof1 = slotS800->MakeFrame(ecvsS800tof1);
   frameS800tof1->SetElementName("T[Xf_Obj]-TObj (gated)");
   fCvsPID = ecvsS800tof1->GetCanvas();

   slotS800 = packPID->NewSlotWithWeight(1.5);
   auto ecvsS800tof2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameS800tof2 = slotS800->MakeFrame(ecvsS800tof2);
   frameS800tof2->SetElementName("T[Xf_Obj]-TObj (full)");
   fCvsPIDFull = ecvsS800tof2->GetCanvas();
   DrawPIDFull();
   // Eighth tab
   // TEveWindowSlot *slotS8002 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   // TEveWindowPack *packPID2 = slotS8002->MakePack();
   // packPID2->SetShowTitleBar(kFALSE);
   // packPID2->SetElementName("S800 PIDs");

   slotS800 = packPID->NewSlotWithWeight(1.5);
   auto ecvsS800dE1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameS800dE1 = slotS800->MakeFrame(ecvsS800dE1);
   frameS800dE1->SetElementName("ICSumE-ToF (gated)");
   fCvsPID2 = ecvsS800dE1->GetCanvas();

   slotS800 = packPID->NewSlotWithWeight(1.5);
   auto ecvsS800dE2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameS800dE2 = slotS800->MakeFrame(ecvsS800dE2);
   frameS800dE2->SetElementName("ICSumE-ToF (full)");
   fCvsPID2Full = ecvsS800dE2->GetCanvas();
   DrawPID2Full();
   /**************************************************************************/

   fS800Ana.SetMTDCXfRange(fMTDCXfRange); // these 3 Setter must be before Init()
   fS800Ana.SetMTDCObjRange(fMTDCObjRange);
   fS800Ana.SetTofObjCorr(fTofObjCorr);

   fRunAna->Init();

   FillPIDFull(); // plot the Full PID at the beginning of the visualiztion

   if (gGeoManager) {
      TGeoNode *geoNode = gGeoManager->GetTopNode();
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
      fEvent = gEve->AddEvent(this);
   }

   /**************************************************************************/

   gEve->GetBrowser()->GetTabRight()->SetTab(1);
   make_gui();

   gEve->Redraw3D(kTRUE, kTRUE);

   TGLViewer *dfViewer = gEve->GetDefaultGLViewer(); // Is this doing anything?
   dfViewer->CurrentCamera().RotateRad(-.7, 0.5);
   dfViewer->DoDraw();

   // RunEvent();
   std::cout << "End of AtEventManager" << std::endl;
}

void AtEventManager::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
   // cout<<fCurrentEvent->GetIntNumber()<<endl;
}

void AtEventManager::GotoEvent(Int_t event)
{

   fEntry = event;
   std::cout << cWHITERED << " Event number : " << fEntry << cNORMAL << std::endl;
   fRunAna->Run((Long64_t)event);
}

void AtEventManager::NextEvent()
{

   Bool_t gated = kFALSE;
   while (gated == kFALSE) {
      fEntry += 1;
      cArray = nullptr;
      cevent = nullptr;
      if (fEntry < 1 || fEntry > fEntries) {
         fEntry = fEntries;
         std::cout << " No gated events found! " << std::endl;
         break;
      }
      fRootManager->ReadEvent(fEntry);
      cArray = dynamic_cast<TClonesArray *>(fRootManager->GetObject("AtEventH"));
      // cArray = dynamic_cast<TObject *>(fRootManager->GetObject("AtEventH"));
      cevent = dynamic_cast<AtEvent *>(cArray->At(0));
      gated = cevent->GetIsExtGate();
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManager::PrevEvent()
{

   Bool_t gated = kFALSE;
   while (gated == kFALSE) {
      fEntry -= 1;
      cArray = nullptr;
      cevent = nullptr;
      if (fEntry < 1 || fEntry > fEntries) {
         fEntry = 1;
         std::cout << " No gated events found! " << std::endl;
         break;
      }
      fRootManager->ReadEvent(fEntry);
      cArray = dynamic_cast<TClonesArray *>(fRootManager->GetObject("AtEventH"));
      // cArray = dynamic_cast<TObject *>(fRootManager->GetObject("AtEventH"));
      cevent = dynamic_cast<AtEvent *>(cArray->At(0));
      gated = cevent->GetIsExtGate();
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManager::DrawPIDFull()
{

   fPIDFull = new TH2F("PIDFull", "PIDFull", 200, -150, 50, 300, 150, 450);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);

   fPIDFull->Draw("colz");
}

void AtEventManager::DrawPID2Full()
{

   fPID2Full = new TH2F("PID2Full", "PID2Full", 200, -150, 50, 300, 150, 450);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);

   fPID2Full->Draw("colz");
}

void AtEventManager::FillPIDFull()
{

   TChain *chain = FairRootManager::Instance()->GetInChain();
   fEntries = chain->GetEntriesFast();

   for (int neve = 1; neve < fEntries; neve++) {
      fRootManager->ReadEvent(neve);
      cS800Calc = dynamic_cast<S800Calc *>(fRootManager->GetObject("s800cal"));
      if (cS800Calc == nullptr)
         break;
      // cS800Calc = (S800Calc*) cS800Array->At(0);

      fS800Ana.Calc(cS800Calc);
      if (fS800Ana.GetObjCorr_ToF() != -999)
         fPIDFull->Fill(fS800Ana.GetObjCorr_ToF(), fS800Ana.GetXfObj_ToF());
      if (fS800Ana.GetObjCorr_ToF() != -999)
         fPID2Full->Fill(fS800Ana.GetObjCorr_ToF(), fS800Ana.GetICSum_E());
   }
}

/*void AtEventManager::NextEvent()
{
   fEntry += 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManager::PrevEvent()
{
   fEntry -= 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}*/

void AtEventManager::DrawWave()
{
   int event = gPad->GetEvent();
   if (event != 11)
      return; // may be comment this line
   TObject *select = gPad->GetSelected();
   if (!select)
      return;
   if (select->InheritsFrom(TH2::Class())) {
      auto *h = dynamic_cast<TH2Poly *>(select);
      gPad->GetCanvas()->FeedbackMode(kTRUE);
      // Char_t *bin_name = h->GetBinName();

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
      // std::cout<<" X : "<<x<<"  Y: "<<y<<std::endl;
      // std::cout<<bin_name<<std::endl;
      std::cout << " Bin number selected : " << bin << " Bin name :" << bin_name << std::endl;
   }

   /*int event = gPad->GetEvent();
   if (event != 11) return; //may be comment this line
   TObject *select = gPad->GetSelected();
   if (!select) return;
   if (select->InheritsFrom("TObject")) {
       TH2PolyBin *h = (TH2PolyBin*)select;
       gPad->GetCanvas()->FeedbackMode(kTRUE);
       Int_t bin = h->GetBinNumber();
       std::cout<<" Clicked on bin : "<<bin<<std::endl;
   }*/
}

void AtEventManager::RunEvent()
{
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManager::make_gui()
{
   // Create minimal GUI for event navigation.

   TChain *chain = FairRootManager::Instance()->GetInChain();
   Int_t Entries = chain->GetEntriesFast();

   TEveBrowser *browser = gEve->GetBrowser();
   browser->StartEmbedding(TRootBrowser::kLeft);

   auto *frmMain = new TGMainFrame(gClient->GetRoot(), 1000, 600);
   frmMain->SetWindowName("XX GUI");
   frmMain->SetCleanup(kDeepCleanup);

   auto *hf = new TGVerticalFrame(frmMain);
   {

      // TString icondir( Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")) );
      //  TGPictureButton* b = 0;

      // EvNavHandler    *fh = new EvNavHandler;
      // AtEventManager *fh = new AtEventManager; //Wrong!! Another instance produces different events

      drawallpad = new TGTextButton(hf, "&Enable Draw All Pads");
      drawallpad->SetToolTipText(
         "Press to Enable/Disble drawing of all pads signal\n (Display on AtTPC Pad Plane Raw Signals tab) ", 400);
      drawallpad->Connect("Clicked()", "AtEventManager", fInstance, "ChangeDrawAllPads()");
      hf->AddFrame(drawallpad, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      eraseQevent = new TGTextButton(hf, "&Erase Q Event Pad");
      eraseQevent->SetToolTipText("Press to erase Event Q histogram upon calling the next event", 400);
      eraseQevent->Connect("Clicked()", "AtEventManager", fInstance, "EraseQEvent()");
      hf->AddFrame(eraseQevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      drawReconstruction = new TGTextButton(hf, "&Visualize Reconstruction");
      drawReconstruction->SetToolTipText("Press to enable Reconstruction visualization", 400);
      drawReconstruction->Connect("Clicked()", "AtEventManager", fInstance, "EnableDrawReconstruction()");
      hf->AddFrame(drawReconstruction, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      saveASCIIevent = new TGTextButton(hf, "&Save event as text file");
      saveASCIIevent->SetToolTipText("Dump the waveform of each hit into a text file", 400);
      saveASCIIevent->Connect("Clicked()", "AtEventManager", fInstance, "SaveASCIIEvent()");
      hf->AddFrame(saveASCIIevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      toggleCorr = new TGTextButton(hf, "&Toggle Corrected Data");
      toggleCorr->SetToolTipText("Press to toggle between data corrected by Lorentz Angle ", 400);
      toggleCorr->Connect("Clicked()", "AtEventManager", fInstance, "ToggleCorrData()");
      hf->AddFrame(toggleCorr, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      /*  b = new TGPictureButton(hf, gClient->GetPicture(icondir+"arrow_left.gif"));
        hf->AddFrame(b);
        b->Connect("Clicked()", "AtEventManager", fInstance, "PrevEvent()");

        b = new TGPictureButton(hf, gClient->GetPicture(icondir+"arrow_right.gif"));
        hf->AddFrame(b);
        b->Connect("Clicked()", "AtEventManager", fInstance, "NextEvent()");*/

      // b = new TGPictureButton(hf, gClient->GetPicture(icondir+"goto.gif"));
      // hf->AddFrame(b);
      // b->Connect("Clicked()", "AtEventManager", fInstance, "GotoEvent(Int_t)");
   }

   auto *hf_2 = new TGHorizontalFrame(frmMain);
   {

      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));
      TGPictureButton *b = nullptr;

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_left.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManager", fInstance, "PrevEvent()");

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_right.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManager", fInstance, "NextEvent()");
   }

   frmMain->AddFrame(hf);
   frmMain->AddFrame(hf_2);

   TString Infile = "Input file : ";
   //  TFile* file =FairRunAna::Instance()->GetInputFile();
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();
   Infile += file->GetName();
   auto *TFName = new TGLabel(frmMain, Infile.Data());
   frmMain->AddFrame(TFName);

   UInt_t RunId = FairRunAna::Instance()->getRunId();
   TString run = "Run Id : ";
   run += RunId;
   auto *TRunId = new TGLabel(frmMain, run.Data());
   frmMain->AddFrame(TRunId);

   TString nevent = "No of events : ";
   nevent += Entries;
   auto *TEvent = new TGLabel(frmMain, nevent.Data());
   frmMain->AddFrame(TEvent);

   auto *f = new TGHorizontalFrame(frmMain);
   auto *l = new TGLabel(f, "Current Event:");
   f->AddFrame(l, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));

   fCurrentEvent = new TGNumberEntry(f, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                     TGNumberFormat::kNELLimitMinMax, 0, Entries);
   f->AddFrame(fCurrentEvent, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));
   fCurrentEvent->Connect("ValueSet(Long_t)", "AtEventManager", fInstance, "SelectEvent()");
   frmMain->AddFrame(f);

   auto *fThres = new TGHorizontalFrame(frmMain);
   auto *lThres = new TGLabel(fThres, "3D threshold:");
   fThres->AddFrame(lThres, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   f3DThresDisplay = new TGNumberEntry(fThres, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                       TGNumberFormat::kNELLimitMinMax, 0, Entries);
   fThres->AddFrame(f3DThresDisplay, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));
   f3DThresDisplay->Connect("ValueSet(Long_t)", "AtEventManager", fInstance, "Select3DThres()");
   frmMain->AddFrame(fThres);

   frmMain->MapSubwindows();
   frmMain->Resize();
   frmMain->MapWindow();

   browser->StopEmbedding();
   browser->SetTabTitle("AtTPC Event Control", 0);
}

void AtEventManager::ChangeDrawAllPads()
{
   drawallpad->SetState(kButtonDown);
   if (!kDrawAllOn) {
      drawallpad->SetText("&Disable Draw All Pads");
      kDrawAllOn = kTRUE;
   } else {
      drawallpad->SetText("&Enable Draw All Pads");
      kDrawAllOn = kFALSE;
   }
   drawallpad->SetState(kButtonUp);
}

void AtEventManager::EnableDrawReconstruction()
{

   drawReconstruction->SetState(kButtonDown);
   if (!kDrawReconstruction) {
      drawReconstruction->SetText("&Disable Vis. Recons.");
      kDrawReconstruction = kTRUE;
   } else {
      drawReconstruction->SetText("&Visualize Reconstruction");
      kDrawReconstruction = kFALSE;
   }
   drawReconstruction->SetState(kButtonUp);
}

void AtEventManager::EraseQEvent()
{

   kEraseQ = kTRUE;
}

void AtEventManager::Draw3DGeo()
{
   kDraw3DGeo = kTRUE;
}

void AtEventManager::Draw3DHist()
{
   kDraw3DHist = kTRUE;
}

void AtEventManager::Select3DThres()
{

   k3DThreshold = f3DThresDisplay->GetIntNumber();
}

void AtEventManager::SaveASCIIEvent()
{

   Int_t event = fEntry;
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();
   std::string file_name = file->GetName();
   std::string cmd = "mv event.dat event_" + std::to_string(event) + ".dat";
   gSystem->Exec(cmd.c_str());
}

void AtEventManager::ToggleCorrData()
{

   toggleCorr->SetState(kButtonDown);
   if (!kToggleData) {
      toggleCorr->SetText("&Toggle Raw Data");
      kToggleData = kTRUE;
   } else {
      toggleCorr->SetText("&Toggle Corrected Data");
      kToggleData = kFALSE;
   }
   toggleCorr->SetState(kButtonUp);
}
