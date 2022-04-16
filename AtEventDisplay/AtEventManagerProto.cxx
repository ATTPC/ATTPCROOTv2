#include "AtEventManagerProto.h"

#include <FairRootManager.h>
#include <FairRunAna.h>

#include <Rtypes.h>
#include <TCanvas.h>
#include <TChain.h>
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

#include <iostream>
#include <string>

class TGeoNode;

using namespace std;

ClassImp(AtEventManagerProto);

AtEventManagerProto *AtEventManagerProto::fInstance = nullptr;
AtEventManagerProto *AtEventManagerProto::Instance()
{
   return fInstance;
}

AtEventManagerProto::AtEventManagerProto()
   : TEveEventManager("AtEventManagerProto", ""), fRootManager(FairRootManager::Instance()),
     fRunAna(FairRunAna::Instance()), fEntry(0), fEvent(nullptr), kDrawPROn(false), drawPatternRecognition(nullptr),
     saveASCIIevent(nullptr), fCurrentEvent(nullptr)

{
   fInstance = this;
}

AtEventManagerProto::~AtEventManagerProto() = default;

void AtEventManagerProto::Init(Int_t option, Int_t level, Int_t nNodes)
{

   gStyle->SetOptTitle(0);
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

   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fPadWave = new TCanvas("AtPad Canvas");
   fPadWave->ToggleEditor();
   slot->StopEmbedding();

   // Pad Plane
   slot = pack2->NewSlotWithWeight(1.5);
   auto *ecvs = new TRootEmbeddedCanvas();
   TEveWindowFrame *frame = slot->MakeFrame(ecvs);
   frame->SetElementName("AtTPC Pad Plane");
   pack->GetEveFrame()->SetShowTitleBar(kFALSE);
   fCvsPadPlane = ecvs->GetCanvas();

   // Raw signals
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

   // Quadrants Radius vs Time

   TEveWindowSlot *slotH = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packH = slotH->MakePack();
   packH->SetShowTitleBar(kFALSE);
   packH->SetHorizontal();
   packH->SetElementName("Radius - Z ");

   slotH = packH->NewSlotWithWeight(1.5);
   auto *ecvsH1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameH1 = slotH->MakeFrame(ecvsH1);
   frameH1->SetElementName("Quadrant 1");
   fCvsQuadrant1 = ecvsH1->GetCanvas();

   slotH = packH->NewSlotWithWeight(1.5);
   auto *ecvsH2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameH2 = slotH->MakeFrame(ecvsH2);
   frameH2->SetElementName("Quadrant 2");
   fCvsQuadrant2 = ecvsH2->GetCanvas();

   slotH = packH->NewSlotWithWeight(1.5);
   auto *ecvsH3 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameH3 = slotH->MakeFrame(ecvsH3);
   frameH3->SetElementName("Quadrant 3");
   fCvsQuadrant3 = ecvsH3->GetCanvas();

   slotH = packH->NewSlotWithWeight(1.5);
   auto *ecvsH4 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameH4 = slotH->MakeFrame(ecvsH4);
   frameH4->SetElementName("Quadrant 4");
   fCvsQuadrant4 = ecvsH4->GetCanvas();

   // Quadrants Radius vs Energy Loss

   TEveWindowSlot *slotHEL = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packHEL = slotHEL->MakePack();
   packHEL->SetShowTitleBar(kFALSE);
   packHEL->SetHorizontal();
   packHEL->SetElementName("Radius - Energy Loss ");

   slotHEL = packHEL->NewSlotWithWeight(1.5);
   auto *ecvsHEL1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHEL1 = slotHEL->MakeFrame(ecvsHEL1);
   frameHEL1->SetElementName("Quadrant Energy Loss 1");
   fCvsELQuadrant1 = ecvsHEL1->GetCanvas();

   slotHEL = packHEL->NewSlotWithWeight(1.5);
   auto *ecvsHEL2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHEL2 = slotHEL->MakeFrame(ecvsHEL2);
   frameHEL2->SetElementName("Quadrant Energy Loss 2");
   fCvsELQuadrant2 = ecvsHEL2->GetCanvas();

   slotHEL = packHEL->NewSlotWithWeight(1.5);
   auto *ecvsHEL3 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHEL3 = slotHEL->MakeFrame(ecvsHEL3);
   frameHEL3->SetElementName("Quadrant Energy Loss 3");
   fCvsELQuadrant3 = ecvsHEL3->GetCanvas();

   slotHEL = packHEL->NewSlotWithWeight(1.5);
   auto *ecvsHEL4 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHEL4 = slotHEL->MakeFrame(ecvsHEL4);
   frameHEL4->SetElementName("Quadrant Energy Loss 4");
   fCvsELQuadrant4 = ecvsHEL4->GetCanvas();

   // Vertex and kinematics
   TEveWindowSlot *slotHK = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   TEveWindowPack *packHK = slotHK->MakePack();
   packHK->SetShowTitleBar(kFALSE);
   packHK->SetHorizontal();
   packHK->SetElementName("Vertex - Kinematics ");

   slotHK = packHK->NewSlotWithWeight(1.5);
   auto *ecvsHK1 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHK1 = slotHK->MakeFrame(ecvsHK1);
   frameHK1->SetElementName("Vertex Position");
   fCvsVertex = ecvsHK1->GetCanvas();

   slotHK = packHK->NewSlotWithWeight(1.5);
   auto *ecvsHK2 = new TRootEmbeddedCanvas();
   TEveWindowFrame *frameHK2 = slotHK->MakeFrame(ecvsHK2);
   frameHK2->SetElementName("Angle-Angle Kinematics");
   fCvsKineAA = ecvsHK2->GetCanvas();

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

   fRunAna->Init();

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
}

void AtEventManagerProto::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
}

void AtEventManagerProto::GotoEvent(Int_t event)
{

   fEntry = event;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)event);
}

void AtEventManagerProto::NextEvent()
{
   fEntry += 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerProto::PrevEvent()
{
   fEntry -= 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerProto::DrawWave()
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
}

void AtEventManagerProto::RunEvent()
{
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerProto::make_gui()
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

      // drawallpad = new TGTextButton(hf, "&Enable Draw All Pads");
      // drawallpad -> SetToolTipText("Press to Enable/Disble drawing of all pads signal\n (Display on AtTPC Pad Plane
      // Raw Signals tab) ",400); drawallpad->Connect("Clicked()", "AtEventManager", fInstance, "ChangeDrawAllPads()");
      // hf->AddFrame(drawallpad, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      // eraseQevent = new TGTextButton(hf, "&Erase Q Event Pad");
      // eraseQevent -> SetToolTipText("Press to erase Event Q histogram upon calling the next event",400);
      // eraseQevent->Connect("Clicked()", "AtEventManager", fInstance, "EraseQEvent()");
      // hf->AddFrame(eraseQevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      drawPatternRecognition = new TGTextButton(hf, "&Enable Pattern Recognition visualization");
      drawPatternRecognition->SetToolTipText("Press to enable Pattern Recognition visualization", 400);
      drawPatternRecognition->Connect("Clicked()", "AtEventManagerProto", fInstance, "EnableDrawPatternRecognition()");
      hf->AddFrame(drawPatternRecognition, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      saveASCIIevent = new TGTextButton(hf, "&Save event as text file");
      saveASCIIevent->SetToolTipText("Dump the waveform of each hit into a text file", 400);
      saveASCIIevent->Connect("Clicked()", "AtEventManagerProto", fInstance, "SaveASCIIEvent()");
      hf->AddFrame(saveASCIIevent, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));

      // toggleCorr = new TGTextButton(hf, "&Toggle Corrected Data");
      // toggleCorr -> SetToolTipText("Press to toggle between data corrected by Lorentz Angle ",400);
      // toggleCorr->Connect("Clicked()", "AtEventManager", fInstance, "ToggleCorrData()");
      // hf->AddFrame(toggleCorr, new TGLayoutHints(kLHintsCenterX, 5, 5, 3, 4));
   }

   /*  TGHorizontalFrame* hf_2 = new TGHorizontalFrame(frmMain);
     {

     TString icondir( Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")) );
          TGPictureButton* b = 0;

          b = new TGPictureButton(hf_2, gClient->GetPicture(icondir+"arrow_left.gif"));
         hf_2->AddFrame(b);
         b->Connect("Clicked()", "AtEventManager", fInstance, "PrevEvent()");

         b = new TGPictureButton(hf_2, gClient->GetPicture(icondir+"arrow_right.gif"));
         hf_2->AddFrame(b);
         b->Connect("Clicked()", "AtEventManager", fInstance, "NextEvent()");

     }*/

   frmMain->AddFrame(hf);
   // frmMain->AddFrame(hf_2);

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

   /*TGHorizontalFrame* fThres = new TGHorizontalFrame(frmMain);
   TGLabel* lThres = new TGLabel(fThres, "3D threshold:");
   fThres->AddFrame(lThres, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));
   f3DThresDisplay = new TGNumberEntry(fThres, 0., 6, -1,
                                      TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                      TGNumberFormat::kNELLimitMinMax, 0, Entries);
   fThres->AddFrame(f3DThresDisplay, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));
   f3DThresDisplay->Connect("ValueSet(Long_t)","AtEventManager",fInstance, "Select3DThres()");
   frmMain->AddFrame(fThres);*/

   frmMain->MapSubwindows();
   frmMain->Resize();
   frmMain->MapWindow();

   browser->StopEmbedding();
   browser->SetTabTitle("AtTPC Event Control", 0);
}

void AtEventManagerProto::EnableDrawPatternRecognition()
{

   drawPatternRecognition->SetState(kButtonDown);
   if (!kDrawPROn) {
      drawPatternRecognition->SetText("&Disable Vis. Recons.");
      kDrawPROn = kTRUE;
   } else {
      drawPatternRecognition->SetText("&Visualize Reconstruction");
      kDrawPROn = kFALSE;
   }
   drawPatternRecognition->SetState(kButtonUp);
}

void AtEventManagerProto::SaveASCIIEvent()
{

   Int_t event = fEntry;
   TFile *file = FairRootManager::Instance()->GetInChain()->GetFile();
   std::string file_name = file->GetName();
   std::string cmd = "mv event.dat event_" + std::to_string(event) + ".dat";
   gSystem->Exec(cmd.c_str());
}
