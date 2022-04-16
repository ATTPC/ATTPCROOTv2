#include "AtEventManagerS800.h"

#include "AtEvent.h" // for AtEvent

#include <FairRootManager.h> // for FairRootManager

#include <TCanvas.h>             // for TCanvas
#include <TChain.h>              // for TChain
#include <TClonesArray.h>        // for TClonesArray
#include <TEveBrowser.h>         // for TEveBrowser
#include <TEveGeoNode.h>         // for TEveGeoTopNode
#include <TEveManager.h>         // for TEveManager, gEve
#include <TEveViewer.h>          // for TEveViewer
#include <TEveWindow.h>          // for TEveWindowPack, TEveWindowFrame
#include <TFile.h>               // for TFile
#include <TGButton.h>            // for TGTextButton, TGPictureButton, kBut...
#include <TGClient.h>            // for TGClient, gClient
#include <TGFrame.h>             // for TGMainFrame, TGHorizontalFrame, TGV...
#include <TGLCamera.h>           // for TGLCamera
#include <TGLViewer.h>           // for TGLViewer
#include <TGLabel.h>             // for TGLabel
#include <TGLayout.h>            // for TGLayoutHints, kLHintsCenterX, kLHi...
#include <TGNumberEntry.h>       // for TGNumberEntry, TGNumberFormat, TGNu...
#include <TGTab.h>               // for TGTab
#include <TGWindow.h>            // for TGWindow
#include <TGeoManager.h>         // for gGeoManager, TGeoManager
#include <TGeoVolume.h>          // for TGeoVolume
#include <TH2.h>                 // for TH2F, TH2
#include <TH2Poly.h>             // for TH2Poly
#include <TObject.h>             // for TObject
#include <TROOT.h>               // IWYU pragma: keep
#include <TRootBrowser.h>        // for TRootBrowser, TRootBrowser::kLeft
#include <TRootEmbeddedCanvas.h> // for TRootEmbeddedCanvas
#include <TString.h>             // for TString, operator+, Form
#include <TStyle.h>              // for TStyle, gStyle
#include <TSystem.h>             // for TSystem, gSystem
#include <TVirtualPad.h>         // for TVirtualPad, gPad
#include <TVirtualX.h>           // for TVirtualX

#include "S800Calc.h" // for S800Calc, CRDC, MultiHitTOF, IC

#include <cmath>    // for isnan, atan
#include <iostream> // for operator<<, basic_ostream::operator<<
#include <string>   // for allocator, char_traits, operator+
#include <vector>   // for vector
class TGeoNode;     // lines 84-84

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";
constexpr auto cWHITERED = "\033[37;41m";
using namespace std;

ClassImp(AtEventManagerS800);

AtEventManagerS800 *AtEventManagerS800::fInstance = nullptr;
AtEventManagerS800 *AtEventManagerS800::Instance()
{
   return fInstance;
}

AtEventManagerS800::AtEventManagerS800()
   : TEveEventManager("AtEventManagerS800", ""), fRootManager(FairRootManager::Instance()),
     fRunAna(FairRunAna::Instance()), fEntry(0), fEvent(nullptr), fCurrentEvent(nullptr), f3DThresDisplay(nullptr),
     fCvsPadPlane(nullptr), fPadWave(nullptr), fPadAll(nullptr), fCvsQEvent(nullptr), fCvsHough(nullptr),
     fCvsRad(nullptr), drawallpad(nullptr), eraseQevent(nullptr), drawHoughSpace(nullptr), saveASCIIevent(nullptr),
     toggleCorr(nullptr), kDrawAllOn(false), kDrawAllOff(false), kEraseQ(false), kDrawHoughOn(false), kDraw3DGeo(false),
     kDraw3DHist(false), kToggleData(false), k3DThreshold(0), fCvsLvsTheta(nullptr), fCvsPID(nullptr),
     fCvsMesh(nullptr), fCvsPIDFull(nullptr), fCvsPID2(nullptr), fCvsPID2Full(nullptr)

{
   fInstance = this;
}

AtEventManagerS800::~AtEventManagerS800() = default;

/*void
AtEventManagerS800::InitRiemann(Int_t option, Int_t level, Int_t nNodes)
{
  TEveManager::Create();
  fRunAna->Init();
  fEvent= gEve->AddEvent(this);
}*/

void AtEventManagerS800::Init(Int_t option, Int_t level, Int_t nNodes)
{

   gStyle->SetOptTitle(0);
   // gStyle->SetCanvasPreferGL(kTRUE);
   gStyle->SetPalette(55);
   TEveManager::Create();

   Int_t dummy;
   UInt_t width, height;
   UInt_t widthMax = 1400, heightMax = 650;
   // Double_t ratio = (Double_t)widthMax / heightMax;
   TVirtualX::Instance()->GetWindowSize(gClient->GetRoot()->GetId(), dummy, dummy, width, height);
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
   /*slot = pack2->NewSlot();
   slot->StartEmbedding();
   fPadWave = new TCanvas("AtPad Canvas");
   fPadWave->ToggleEditor();
   slot->StopEmbedding();
   */
   /*slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsPadPlane = new TCanvas("AtPoly");
   fPadWave->ToggleEditor();
   slot->StopEmbedding();
   fCvsPadPlane->AddExec("ex","AtEventManagerS800::DrawWave()");*/

   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs01 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame01 = slot->MakeFrame(ecvs01);
   frame01->SetElementName("AtTPC Mesh");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsMesh = ecvs01->GetCanvas();

   // Pad Plane
   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("AtTPC Pad Plane");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsPadPlane = ecvs->GetCanvas();
   // fCvsPadPlane->AddExec("ex","AtEventManagerS800::DrawWave()"); //OBSOLETE DO NOT USE

   // A test
   /*slot = pack2->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs2 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame2 = slot->MakeFrame(ecvs2);
   frame2->SetElementName("AtTPC Pad Plane All");
   fPadAll = ecvs2->GetCanvas();*/

   /*
        //Sixth tab 3D Histogram
   TEveWindowSlot* slot5 =
   TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack* pack6 = slot5->MakePack();
   pack6->SetShowTitleBar(kFALSE);
   pack6->SetElementName("3D Histogram View");
   slot5 = pack6->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs6 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame6 = slot5->MakeFrame(ecvs6);
   frame6->SetElementName("3D Histogram View");
   fCvs3DHist = ecvs6->GetCanvas();
   */

   // Third tab
   TEveWindowSlot *slot2 = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack3 = slot2->MakePack();
   pack3->SetShowTitleBar(kFALSE);
   pack3->SetElementName("S800 PID1");

   slot2 = pack3->NewSlotWithWeight(1.5);
   auto *ecvs3 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame3 = slot2->MakeFrame(ecvs3);
   frame3->SetElementName("T[Xf_Obj]-TObj (gated)");
   fCvsPID = ecvs3->GetCanvas();

   slot2 = pack3->NewSlotWithWeight(1.5);
   auto *ecvs31 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame31 = slot2->MakeFrame(ecvs31);
   frame31->SetElementName("T[Xf_Obj]-TObj (full)");
   fCvsPIDFull = ecvs31->GetCanvas();
   DrawPIDFull();

   TEveWindowSlot *slot2b = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *pack3b = slot2b->MakePack();
   pack3b->SetShowTitleBar(kFALSE);
   pack3b->SetElementName("S800 PID2");

   slot2b = pack3b->NewSlotWithWeight(1.5);
   auto *ecvs3b = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame3b = slot2b->MakeFrame(ecvs3b);
   frame3b->SetElementName("ICSumE-ToF (gated)");
   fCvsPID2 = ecvs3b->GetCanvas();

   slot2b = pack3b->NewSlotWithWeight(1.5);
   auto *ecvs31b = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame31b = slot2b->MakeFrame(ecvs31b);
   frame31b->SetElementName("ICSumE-ToF (full)");
   fCvsPID2Full = ecvs31b->GetCanvas();
   DrawPID2Full();

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
   frame4->SetElementName("Cumulated PadPlane Hits");
   fCvsHough = ecvs4->GetCanvas();

   slot3 = pack4->NewSlotWithWeight(1.5);
   auto *ecvs4_add = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame4_add = slot3->MakeFrame(ecvs4_add);
   frame4_add->SetElementName("Track Length vs Theta");
   fCvsLvsTheta = ecvs4_add->GetCanvas();

   /*
   slot3 = pack4->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs4_add2 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame4_add2 = slot3->MakeFrame(ecvs4_add2);
   frame4_add2->SetElementName("Theta");
   fCvsTheta = ecvs4_add2->GetCanvas();

   slot3 = pack4->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs4_add3 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame4_add3 = slot3->MakeFrame(ecvs4_add3);
   frame4_add3->SetElementName("Theta X Phi");
   fCvsThetaxPhi = ecvs4_add3->GetCanvas();
   */
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
   /*
      //Fifth tab Phi Reconstruction
   TEveWindowSlot* slot4 =
   TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack* pack5 = slot4->MakePack();
   pack5->SetShowTitleBar(kFALSE);
   pack5->SetElementName("Prototype Phi Recons.");
   slot4 = pack5->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvs5 = new TRootEmbeddedCanvas();
   TEveWindowFrame* frame5 = slot4->MakeFrame(ecvs5);
   frame5->SetElementName("Phi Reconstruction");
   fCvsPhi = ecvs5->GetCanvas();
   */
   /*
    //Sixth tab Monte Carlo
   TEveWindowSlot* slotMC =
   TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack* packMC = slotMC->MakePack();
   packMC->SetShowTitleBar(kFALSE);
   packMC->SetHorizontal();
   packMC->SetElementName("Monte Carlo");

   slotMC = packMC->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvsMC_XY = new TRootEmbeddedCanvas();
   TEveWindowFrame* frameMC_XY = slotMC->MakeFrame(ecvsMC_XY);
   frameMC_XY->SetElementName("XY Projection");
   fCvsMC_XY = ecvsMC_XY->GetCanvas();

   slotMC = packMC->NewSlotWithWeight(1.5);
   TRootEmbeddedCanvas* ecvsMC_Z = new TRootEmbeddedCanvas();
   TEveWindowFrame* frameMC_Z = slotMC->MakeFrame(ecvsMC_Z);
   frameMC_Z->SetElementName("Time Projection");
   fCvsMC_Z = ecvsMC_Z->GetCanvas();
   */

   /**************************************************************************/

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
}

void AtEventManagerS800::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
   // cout<<fCurrentEvent->GetIntNumber()<<endl;
}

void AtEventManagerS800::GotoEvent(Int_t event)
{

   fEntry = event;
   std::cout << cWHITERED << " Event number : " << fEntry << cNORMAL << std::endl;
   fRunAna->Run((Long64_t)event);
}

void AtEventManagerS800::NextEvent()
{

   Bool_t gated = kFALSE;
   while (gated == kFALSE) {
      fEntry += 1;
      cArray = nullptr;
      cevent = nullptr;
      if (fEntry < 1 || fEntry > Entries) {
         fEntry = Entries;
         std::cout << " No gated events found! " << std::endl;
         break;
      }
      fRootManager->ReadEvent(fEntry);
      cArray = dynamic_cast<TClonesArray *>(fRootManager->GetObject("AtEventH"));
      cevent = dynamic_cast<AtEvent *>(cArray->At(0));
      gated = cevent->IsInGate();
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerS800::PrevEvent()
{

   Bool_t gated = kFALSE;
   while (gated == kFALSE) {
      fEntry -= 1;
      cArray = nullptr;
      cevent = nullptr;
      if (fEntry < 1 || fEntry > Entries) {
         fEntry = 1;
         std::cout << " No gated events found! " << std::endl;
         break;
      }
      fRootManager->ReadEvent(fEntry);
      cArray = dynamic_cast<TClonesArray *>(fRootManager->GetObject("AtEventH"));
      cevent = dynamic_cast<AtEvent *>(cArray->At(0));
      gated = cevent->IsInGate();
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerS800::DrawPIDFull()
{

   fPIDFull = new TH2F("PIDFull", "PIDFull", 500, -150, 50, 300, 230, 260);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);

   fPIDFull->Draw("colz");
}

void AtEventManagerS800::DrawPID2Full()
{

   fPID2Full = new TH2F("PID2Full", "PID2Full", 500, -150, 50, 1000, 1400, 2200);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);

   fPID2Full->Draw("colz");
}

void AtEventManagerS800::FillPIDFull()
{
   //	TFile *fi=new TFile("/mnt/analysis/e18027/codes/AtTPCROOTv2/macro/Unpack_HDF5/e18027_S800/cuts/afp.root"); // open
   // file
   // 	TCutG *CUT1=(TCutG *)fi->Get("afp");	   // read CUTEC
   //	TFile *fi2=new TFile("/mnt/analysis/e18027/codes/AtTPCROOTv2/macro/Unpack_HDF5/e18027_S800/cuts/tof.root"); //
   // open file
   // 	TCutG *CUT2=(TCutG *)fi2->Get("tof");	   // read CUTEC

   TChain *chain = FairRootManager::Instance()->GetInChain();
   Entries = chain->GetEntriesFast();
   // std::cout << "el plot full antes....  "<< Entries<< '\n';
   for (int neve = 1; neve < Entries; neve++) {
      fRootManager->ReadEvent(neve);
      //    cS800Array = (TClonesArray*) fRootManager->GetObject("s800cal");
      //    if(cS800Array == nullptr) break;
      //    cS800Calc = (S800Calc*) cS800Array->At(0);

      cS800Calc = dynamic_cast<S800Calc *>(fRootManager->GetObject("s800cal"));
      if (cS800Calc == nullptr)
         break;
      // cS800Calc = (S800Calc*) cS800Array->At(0);

      Double_t x0_corr_tof = 0.;
      Double_t afp_corr_tof = 0.;
      Double_t afp_corr_dE = 0.;
      Double_t x0_corr_dE = 0.;
      Double_t rf_offset = 0.;
      Double_t corrGainE1up = 1;
      Double_t corrGainE1down = 1;

      // Double_t S800_timeRf = cS800Calc->GetMultiHitTOF()->GetFirstRfHit();
      // Double_t S800_timeE1up = cS800Calc->GetMultiHitTOF()->GetFirstE1UpHit();
      // Double_t S800_timeE1down = cS800Calc->GetMultiHitTOF()->GetFirstE1DownHit();
      // Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
      // Double_t S800_timeXf = cS800Calc->GetMultiHitTOF()->GetFirstXfHit();
      // Double_t S800_timeObj = cS800Calc->GetMultiHitTOF()->GetFirstObjHit();

      Int_t CondMTDCXfObj = 0;
      Double_t ObjCorr1C1 = 100.;  // 70
      Double_t ObjCorr1C2 = 0.009; // 0.0085
      vector<Float_t> S800_timeMTDCObj = cS800Calc->GetMultiHitTOF()->GetMTDCObj();
      vector<Float_t> S800_timeMTDCXf = cS800Calc->GetMultiHitTOF()->GetMTDCXf();
      Float_t S800_timeObjSelect = -999;
      Float_t S800_timeXfSelect = -999;
      Float_t ObjCorr = -999;

      for (float k : S800_timeMTDCXf) {
         if (k > 140 && k < 230)
            S800_timeXfSelect = k;
      }
      for (float k : S800_timeMTDCObj) {
         if (k > -115 && k < -20)
            S800_timeObjSelect = k;
      }

      Double_t XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
      if (S800_timeXfSelect != -999 && S800_timeObjSelect != -999) {
         XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
         CondMTDCXfObj = 1;
      }
      Double_t S800_ICSum = cS800Calc->GetIC()->GetSum();

      Double_t S800_x0 = cS800Calc->GetCRDC(0)->GetX();
      Double_t S800_x1 = cS800Calc->GetCRDC(1)->GetX();
      // Double_t S800_y0 = cS800Calc->GetCRDC(0)->GetY();
      // Double_t S800_y1 = cS800Calc->GetCRDC(1)->GetY();

      // Double_t S800_E1up = cS800Calc->GetSCINT(0)->GetDEup();
      // Double_t S800_E1down = cS800Calc->GetSCINT(0)->GetDEdown();

      // Double_t S800_tof = S800_timeObj - S800_timeE1;

      Double_t S800_afp = atan((S800_x1 - S800_x0) / 1073.);
      // Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
      // Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
      // Double_t S800_dE = cS800Calc->GetSCINT(0)->GetDE();//check if is this scint (0)
      // Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
      // Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

      if (CondMTDCXfObj && std::isnan(S800_ICSum) == 0 && std::isnan(S800_afp) == 0 && std::isnan(S800_x0) == 0)
         ObjCorr = S800_timeObjSelect + ObjCorr1C1 * S800_afp + ObjCorr1C2 * S800_x0;

      // std::cout<<"draw func in cut "<<S800_timeObjSelect<<" "<<XfObj_tof<<" "<<S800_ICSum<<std::endl;

      if (ObjCorr != -999)
         fPIDFull->Fill(ObjCorr, XfObj_tof);
      if (ObjCorr != -999)
         fPID2Full->Fill(ObjCorr, S800_ICSum);
   }
}

void AtEventManagerS800::DrawWave()
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
         TVirtualX::Instance()->DrawLine(pxmin, pyold, pxmax, pyold);
      TVirtualX::Instance()->DrawLine(pxmin, py, pxmax, py);
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

void AtEventManagerS800::RunEvent()
{
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerS800::make_gui()
{
   // Create minimal GUI for event navigation.

   TChain *chain = FairRootManager::Instance()->GetInChain();
   Entries = chain->GetEntriesFast();

   // std::cout << "Numero entradas en el gui   "<<Entries << '\n';

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
      // AtEventManagerS800 *fh = new AtEventManagerS800; //Wrong!! Another instance produces different events

      drawallpad = new TGTextButton(hf, "&Enable Draw All Pads");
      drawallpad->SetToolTipText(
         "Press to Enable/Disble drawing of all pads signal\n (Display on AtTPC Pad Plane Raw Signals tab) ", 400);
      drawallpad->Connect("Clicked()", "AtEventManagerS800", fInstance, "ChangeDrawAllPads()");
      hf->AddFrame(drawallpad, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      eraseQevent = new TGTextButton(hf, "&Erase Q Event Pad");
      eraseQevent->SetToolTipText("Press to erase Event Q histogram upon calling the next event", 400);
      eraseQevent->Connect("Clicked()", "AtEventManagerS800", fInstance, "EraseQEvent()");
      hf->AddFrame(eraseQevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      drawHoughSpace = new TGTextButton(hf, "&Visualize Reconstruction");
      drawHoughSpace->SetToolTipText("Press to enable Reconstruction visualization", 400);
      drawHoughSpace->Connect("Clicked()", "AtEventManagerS800", fInstance, "EnableDrawHoughSpace()");
      hf->AddFrame(drawHoughSpace, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      saveASCIIevent = new TGTextButton(hf, "&Save event as text file");
      saveASCIIevent->SetToolTipText("Dump the waveform of each hit into a text file", 400);
      saveASCIIevent->Connect("Clicked()", "AtEventManagerS800", fInstance, "SaveASCIIEvent()");
      hf->AddFrame(saveASCIIevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      toggleCorr = new TGTextButton(hf, "&Toggle Corrected Data");
      toggleCorr->SetToolTipText("Press to toggle between data corrected by Lorentz Angle ", 400);
      toggleCorr->Connect("Clicked()", "AtEventManagerS800", fInstance, "ToggleCorrData()");
      hf->AddFrame(toggleCorr, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      /*  b = new TGPictureButton(hf, gClient->GetPicture(icondir+"arrow_left.gif"));
        hf->AddFrame(b);
        b->Connect("Clicked()", "AtEventManagerS800", fInstance, "PrevEvent()");

        b = new TGPictureButton(hf, gClient->GetPicture(icondir+"arrow_right.gif"));
        hf->AddFrame(b);
        b->Connect("Clicked()", "AtEventManagerS800", fInstance, "NextEvent()");*/

      // b = new TGPictureButton(hf, gClient->GetPicture(icondir+"goto.gif"));
      // hf->AddFrame(b);
      // b->Connect("Clicked()", "AtEventManagerS800", fInstance, "GotoEvent(Int_t)");
   }

   auto *hf_2 = new TGHorizontalFrame(frmMain);
   {

      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));
      TGPictureButton *b = nullptr;

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_left.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerS800", fInstance, "PrevEvent()");

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_right.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerS800", fInstance, "NextEvent()");
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
   fCurrentEvent->Connect("ValueSet(Long_t)", "AtEventManagerS800", fInstance, "SelectEvent()");
   frmMain->AddFrame(f);

   auto *fThres = new TGHorizontalFrame(frmMain);
   auto *lThres = new TGLabel(fThres, "3D threshold:");
   fThres->AddFrame(lThres, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   f3DThresDisplay = new TGNumberEntry(fThres, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                       TGNumberFormat::kNELLimitMinMax, 0, Entries);
   fThres->AddFrame(f3DThresDisplay, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));
   f3DThresDisplay->Connect("ValueSet(Long_t)", "AtEventManagerS800", fInstance, "Select3DThres()");
   frmMain->AddFrame(fThres);

   frmMain->MapSubwindows();
   frmMain->Resize();
   frmMain->MapWindow();

   browser->StopEmbedding();
   browser->SetTabTitle("AtTPC Event Control", 0);
}

void AtEventManagerS800::ChangeDrawAllPads()
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

void AtEventManagerS800::EnableDrawHoughSpace()
{

   drawHoughSpace->SetState(kButtonDown);
   if (!kDrawHoughOn) {
      drawHoughSpace->SetText("&Disable Vis. Recons.");
      kDrawHoughOn = kTRUE;
   } else {
      drawHoughSpace->SetText("&Visualize Reconstruction");
      kDrawHoughOn = kFALSE;
   }
   drawHoughSpace->SetState(kButtonUp);
}

void AtEventManagerS800::EraseQEvent()
{

   kEraseQ = kTRUE;
}

void AtEventManagerS800::Draw3DGeo()
{
   kDraw3DGeo = kTRUE;
}

void AtEventManagerS800::Draw3DHist()
{
   kDraw3DHist = kTRUE;
}

void AtEventManagerS800::Select3DThres()
{

   k3DThreshold = f3DThresDisplay->GetIntNumber();
}

void AtEventManagerS800::SaveASCIIEvent()
{

   Int_t event = fEntry;
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();
   std::string file_name = file->GetName();
   std::string cmd = "mv event.dat event_" + std::to_string(event) + ".dat";
   gSystem->Exec(cmd.c_str());
}

void AtEventManagerS800::ToggleCorrData()
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
