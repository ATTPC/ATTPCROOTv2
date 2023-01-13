#include "AtEventManagerNew.h"

#include "AtTabTask.h"
#include "AtMap.h"

#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairTask.h>

#include <Rtypes.h>
#include <TCanvas.h>
#include <TChain.h>
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
     fEntry(0), fEvent(nullptr), fCurrentEvent(nullptr), f3DThresDisplay(nullptr), kToggleData(false), fEntries(0), fTabTask(nullptr)

{
   fInstance = this;
}

AtEventManagerNew::~AtEventManagerNew() = default;

void AtEventManagerNew::AddTabTask(AtTabTask *task) {
   if(fTabTask != nullptr) 
      LOG(error) << "AtTabTask already exists in Event Manager. Only one can exist!";
   else {
      AddTask(task);
      fTabTask = task;
   }
}

void AtEventManagerNew::Init()
{
   gStyle->SetOptTitle(0);
   gStyle->SetPalette(55);
   TEveManager::Create();

   Int_t dummy;
   UInt_t width, height;
   UInt_t widthMax = 1400, heightMax = 650;
   gVirtualX->GetWindowSize(gClient->GetRoot()->GetId(), dummy, dummy, width, height);

   /**************************************************************************/

   fRunAna->Init();
   TChain *chain = FairRootManager::Instance()->GetInChain();
   fEntries = chain->GetEntriesFast();

   /**************************************************************************/
   fEvent = gEve->AddEvent(this);
   make_gui();

   std::cout << "End of AtEventManagerNew" << std::endl;
}

void AtEventManagerNew::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
}

void AtEventManagerNew::GotoEvent(Int_t event)
{

   fEntry = event;
   std::cout << cWHITERED << " Event number : " << fEntry << cNORMAL << std::endl;
   fRunAna->Run((Long64_t)event);
}

void AtEventManagerNew::NextEvent()
{
   fEntry += 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

void AtEventManagerNew::PrevEvent()
{
   fEntry -= 1;
   std::cout << " Event number : " << fEntry << std::endl;
   fRunAna->Run((Long64_t)fEntry);
}

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
         std::cout << "AtMap not set! Is an AtTabTask with AtTabMain included?" << std::endl;
      }
      else {
      Int_t tPadNum = tmap->BinToPad(bin);
      std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
      std::cout << " Electronic mapping: " << tmap->GetPadRef(tPadNum) << std::endl;
      //std::cout << " Event ID (Select Pad) : " << tRawEvent->GetEventID() << std::endl;
      std::cout << " Raw Event Pad Num " << tPadNum << std::endl;
      //DrawUpdates(drawNums, tPadNum);
      AtEventManagerNew::Instance()->DrawUpdates(tPadNum);
      }
   }
}

void AtEventManagerNew::DrawUpdates(Int_t padNum) {
   fTabTask->DrawPad(padNum);
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