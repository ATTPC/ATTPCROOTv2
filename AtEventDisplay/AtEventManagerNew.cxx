#include "AtEventManagerNew.h"

#include "AtEvent.h" // for AtEvent
#include "AtEventDrawTaskNew.h"
#include "AtEventTabTask.h"
#include "AtMap.h"

#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairTask.h>

#include <Rtypes.h>
#include <TCanvas.h>
#include <TChain.h>
#include <TClonesArray.h> // for TClonesArray
#include <TEveBrowser.h>
#include <TEveEventManager.h>
#include <TEveGeoNode.h>
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
#include <TList.h>
#include <TObject.h>
#include <TROOT.h>           // for TROOT, gROOT
#include <TRootBrowser.h>
#include <TRootEmbeddedCanvas.h>
#include <TString.h>
#include <TStyle.h>
#include <TSystem.h>
#include <TVirtualPad.h>
#include <TVirtualX.h>

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

ClassImp(AtEventManagerNew);

AtEventManagerNew *AtEventManagerNew::fInstance = nullptr;
AtEventManagerNew *AtEventManagerNew::Instance()
{
   return fInstance;
}

AtEventManagerNew::AtEventManagerNew()
   : TEveEventManager("AtEventManager", ""), fRootManager(FairRootManager::Instance()), fRunAna(FairRunAna::Instance()),
     fEntry(0), fEvent(nullptr), fCurrentEvent(nullptr), f3DThresDisplay(nullptr), fCvsPadPlane(nullptr),
     fPadWave(nullptr), kToggleData(false), cArray(nullptr), fEntries(0), fTabTaskNum(0)

{
   fInstance = this;
   fTabList = new TList();
}

AtEventManagerNew::~AtEventManagerNew() = default;

/*void
AtEventManagerNew::InitRiemann(Int_t option, Int_t level, Int_t nNodes)
{
  TEveManager::Create();
  fRunAna->Init();
  fEvent= gEve->AddEvent(this);
}*/

void AtEventManagerNew::AddTabTask(FairTask *task) {
   AddTask(task);
   fTabList->Add(task);
   fTabTaskNum++;
}

void AtEventManagerNew::Init(Int_t option, Int_t level, Int_t nNodes)
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

   MakeTabs();

   //MakeMainTab();

   fRunAna->Init();
   TChain *chain = FairRootManager::Instance()->GetInChain();
   fEntries = chain->GetEntriesFast();

   /*if (gGeoManager) {
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
   }*/

   /**************************************************************************/

 //  gEve->GetBrowser()->GetTabRight()->SetTab(1);
   make_gui();

//   gEve->Redraw3D(kTRUE, kTRUE);

//   TGLViewer *dfViewer = gEve->GetDefaultGLViewer(); // Is this doing anything?
//   dfViewer->CurrentCamera().RotateRad(-.7, 0.5);
//   dfViewer->DoDraw();

   // RunEvent();
   std::cout << "End of AtEventManagerNew" << std::endl;
}

void AtEventManagerNew::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
   // cout<<fCurrentEvent->GetIntNumber()<<endl;
}

void AtEventManagerNew::GotoEvent(Int_t event)
{

   fEntry = event;
   std::cout << cWHITERED << " Event number : " << fEntry << cNORMAL << std::endl;
   fRunAna->Run((Long64_t)event);
}

void AtEventManagerNew::NextEvent()
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
      gated = kTRUE;
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerNew::PrevEvent()
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
      gated = kTRUE;
   }

   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

//void AtEventManagerNew::SelectPad(Int_t drawNums)
void AtEventManagerNew::SelectPad()
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
      std::cout << " ==========================" << std::endl;
      std::cout << " Bin number selected : " << bin << " Bin name :" << bin_name << std::endl;

      AtMap *tmap = nullptr;
      tmap = dynamic_cast<AtMap *>(gROOT->GetListOfSpecials()->FindObject("fMap"));
      if(tmap == nullptr) {
         std::cout << "AtMap not set! Is an AtEventTabTask with AtEventTabMain included?" << std::endl;
      }
      else {
      Int_t tPadNum = tmap->BinToPad(bin);
      std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
      std::cout << " Electronic mapping: " << tmap->GetPadRef(tPadNum) << std::endl;
      //std::cout << " Event ID (Select Pad) : " << tRawEvent->GetEventID() << std::endl;
      std::cout << " Raw Event Pad Num " << tPadNum << std::endl;
      //DrawUpdates(drawNums, tPadNum);
      DrawUpdates(tPadNum);
      }
   }

}

void AtEventManagerNew::DrawUpdates(Int_t padNum) {
   for(const auto&& tabTask: *(AtEventManagerNew::Instance()->GetTabList()))
      dynamic_cast<AtEventTabTaskBase *>(tabTask)->DrawPad(padNum);
}

void AtEventManagerNew::RunEvent()
{
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerNew::make_gui()
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

   auto *hf_2 = new TGHorizontalFrame(frmMain);
   {

      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));
      TGPictureButton *b = nullptr;

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_left.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", fInstance, "PrevEvent()");

      b = new TGPictureButton(hf_2, gClient->GetPicture(icondir + "arrow_right.gif"));
      hf_2->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", fInstance, "NextEvent()");
   }

   frmMain->AddFrame(hf);
   frmMain->AddFrame(hf_2);

   TString Infile = "Input file : ";
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
   fCurrentEvent->Connect("ValueSet(Long_t)", "AtEventManagerNew", fInstance, "SelectEvent()");
   frmMain->AddFrame(f);

   frmMain->MapSubwindows();
   frmMain->Resize();
   frmMain->MapWindow();

   browser->StopEmbedding();
   browser->SetTabTitle("AtTPC Event Control", 0);
}

void AtEventManagerNew::MakeMainTab() {
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
   fCvsPadPlane->AddExec("ex", "AtEventManagerNew::SelectPad()");
   //fCvsPadPlane->AddExec("ex", "AtEventManagerNew::SelectPad(fDrawTaskNum)");


}

void AtEventManagerNew::MakeTabs() {
   char name[20];
   for(int i = 0; i < fTabTaskNum; i++) {
      sprintf(name, "fTabTask_%i", fTabTaskNum);
      auto *tabTask = dynamic_cast<AtEventTabTask *>(gROOT->GetListOfSpecials()->FindObject(name));
      if(tabTask == nullptr) {
         std::cout << "tabTask " << i << " is nullptr!" << std::endl;
      }
      else {
         tabTask->MakeTab();
      }
   }
}
