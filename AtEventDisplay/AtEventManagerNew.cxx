#include "AtEventManagerNew.h"

#include "AtMap.h"
#include "AtTabTask.h"

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
#include <TGComboBox.h>
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
#include <TROOT.h> // for TROOT, gROOT
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

AtEventManagerNew::AtEventManagerNew(std::shared_ptr<AtMap> map)
   : TEveEventManager("AtEventManager", ""), fEntry(0), fCurrentEvent(nullptr), f3DThresDisplay(nullptr), fEntries(0),
     fTabTask(nullptr), fMap(map)

{
   if (fInstance != nullptr)
      LOG(fatal) << "Attempting to create a second instance of AtEventManagerNew! Only one is allowed!";
   fInstance = this;
}

AtEventManagerNew::~AtEventManagerNew()
{
   fInstance = nullptr;
}

void AtEventManagerNew::AddTask(FairTask *task)
{
   if (dynamic_cast<AtTabTask *>(task) == nullptr)
      FairRunAna::Instance()->AddTask(task);
   else
      AddTabTask(dynamic_cast<AtTabTask *>(task));
}

void AtEventManagerNew::AddTabTask(AtTabTask *task)
{
   if (fTabTask != nullptr)
      LOG(fatal) << "AtTabTask already exists in Event Manager. Only one can exist!";
   else {
      FairRunAna::Instance()->AddTask(task);
      fTabTask = task;
   }
}
void AtEventManagerNew::GenerateBranchLists()
{
   FairRunAna::Instance()->Run((Long64_t)0);
   auto ioMan = FairRootManager::Instance();

   // Loop through the entire branch list and try to identify the class type of each branch
   for (int i = 0; i < ioMan->GetBranchNameList()->GetSize(); i++) {
      auto branchName = ioMan->GetBranchName(i);
      auto branchArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(branchName));
      if (branchArray == nullptr || branchArray->GetSize() < 1)
         continue;

      // Check for event types this is very hacky but what can you do
      for (int j = 0; j < fBranchTypes.size(); ++j) {
         fSubjectBranchNames[j] = std::make_unique<BranchName>(fBranchTypes[j], "", "");
         if (std::string(branchArray->At(0)->ClassName()).compare(fBranchTypes[j]) == 0)
            fBranchNames[j].push_back(branchName);
      }
   }
}

void AtEventManagerNew::Init()
{
   gStyle->SetOptTitle(0);
   gStyle->SetPalette(55);
   TEveManager::Create();

   // Call init on the run after here all of the AtTabInfo will be created
   FairRunAna::Instance()->Init();
   TChain *chain = FairRootManager::Instance()->GetInChain();
   fEntries = chain->GetEntriesFast();
   gEve->AddEvent(this);

   // Everything is loaded so construct the list of branch names
   // Here we should also generate everything needed to create the data sources
   GenerateBranchLists();

   // The bulk of this is creating the sidebar
   make_gui();

   // Register the data sources with every tab now that everything is in place
   RegisterDataHandles();

   std::cout << "End of AtEventManagerNew" << std::endl;
}

void AtEventManagerNew::make_gui()
{
   // Create minimal GUI for event navigation.
   TChain *chain = FairRootManager::Instance()->GetInChain();
   Int_t Entries = chain->GetEntriesFast();

   // Get the sidebar and add a tab to the left (frmMain)
   TEveBrowser *browser = gEve->GetBrowser();
   browser->StartEmbedding(TRootBrowser::kLeft);

   auto *frmMain = new TGMainFrame(gClient->GetRoot(), 1000, 600);
   frmMain->SetWindowName("XX GUI");
   frmMain->SetCleanup(kDeepCleanup);

   auto *buttonFrame = new TGHorizontalFrame(frmMain); // Navigation button frame
   {

      TString icondir(Form("%s/icons/", gSystem->Getenv("VMCWORKDIR")));
      TGPictureButton *b = nullptr;

      b = new TGPictureButton(buttonFrame, gClient->GetPicture(icondir + "arrow_left.gif"));
      buttonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", fInstance, "PrevEvent()");

      b = new TGPictureButton(buttonFrame, gClient->GetPicture(icondir + "arrow_right.gif"));
      buttonFrame->AddFrame(b);
      b->Connect("Clicked()", "AtEventManagerNew", fInstance, "NextEvent()");
   }

   frmMain->AddFrame(buttonFrame);

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

   // f is frame with current event: [selection box]
   auto *f = new TGHorizontalFrame(frmMain);
   auto *l = new TGLabel(f, "Current Event:");
   f->AddFrame(l, new TGLayoutHints(kLHintsLeft | kLHintsCenterY, 1, 2, 1, 1));

   fCurrentEvent = new TGNumberEntry(f, 0., 6, -1, TGNumberFormat::kNESInteger, TGNumberFormat::kNEANonNegative,
                                     TGNumberFormat::kNELLimitMinMax, 0, Entries);
   f->AddFrame(fCurrentEvent, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));
   fCurrentEvent->Connect("ValueSet(Long_t)", "AtEventManagerNew", fInstance, "SelectEvent()");
   frmMain->AddFrame(f);

   /**** Begin code for branch selection *****/
   frmMain->AddFrame(new TGLabel(frmMain, "Selected Branches"), new TGLayoutHints(kLHintsCenterX));

   // Create frame to select raw event
   auto f2 = new TGHorizontalFrame(frmMain);
   f2->AddFrame(new TGLabel(f2, "Raw Event: "));

   fBranchBoxes[0] = new TGComboBox(f2);
   for (int i = 0; i < fBranchNames[0].size(); ++i)
      fBranchBoxes[0]->AddEntry(fBranchNames[0][i], i);
   fBranchBoxes[0]->Select(0);
   fBranchBoxes[0]->Connect("Selected(Int_t)", "AtEventManagerNew", this, "SelectAtRawEvent(Int_t)");

   f2->AddFrame(fBranchBoxes[0], new TGLayoutHints(kLHintsExpandX | kLHintsCenterY | kLHintsExpandY));
   frmMain->AddFrame(f2, new TGLayoutHints(kLHintsExpandX));

   // Create frame to select event
   f2 = new TGHorizontalFrame(frmMain);
   f2->AddFrame(new TGLabel(f2, "Event: "));

   fBranchBoxes[1] = new TGComboBox(f2);
   for (int i = 0; i < fBranchNames[1].size(); ++i)
      fBranchBoxes[1]->AddEntry(fBranchNames[1][i], i);
   fBranchBoxes[1]->Select(0);
   fBranchBoxes[1]->Connect("Selected(Int_t)", "AtEventManagerNew", this, "SelectAtEvent(Int_t)");

   f2->AddFrame(fBranchBoxes[1], new TGLayoutHints(kLHintsExpandX | kLHintsCenterY | kLHintsExpandY));
   frmMain->AddFrame(f2, new TGLayoutHints(kLHintsExpandX));

   // Create frame to select pattern event
   f2 = new TGHorizontalFrame(frmMain);
   f2->AddFrame(new TGLabel(f2, "Pattern Event: "));

   fBranchBoxes[2] = new TGComboBox(f2);
   for (int i = 0; i < fBranchNames[2].size(); ++i)
      fBranchBoxes[2]->AddEntry(fBranchNames[2][i], i);
   fBranchBoxes[2]->Select(0);
   fBranchBoxes[2]->Connect("Selected(Int_t)", "AtEventManagerNew", this, "SelectAtPatternEvent(Int_t)");

   f2->AddFrame(fBranchBoxes[2], new TGLayoutHints(kLHintsExpandX | kLHintsCenterY | kLHintsExpandY));
   frmMain->AddFrame(f2, new TGLayoutHints(kLHintsExpandX));

   auto redrawButton = new TGTextButton(frmMain, "Redraw All");
   redrawButton->Connect("Clicked()", "AtEventManagerNew", this, "RedrawEvent()");
   frmMain->AddFrame(redrawButton, new TGLayoutHints(kLHintsLeft, 1, 1, 1, 1));

   /**** End code for branch selection *****/

   // End addition of things to the sidebar
   frmMain->MapSubwindows();
   frmMain->Resize();
   frmMain->MapWindow();

   browser->StopEmbedding();
   browser->SetTabTitle("Event Control", 0);
}

void AtEventManagerNew::SelectEventBranch(int sel, int i)
{
   if (sel < 0)
      return;
   std::cout << "Changing " << fBranchTypes[i] << " to " << fBranchNames[i][sel] << std::endl;
   fSubjectBranchNames[i]->SetBranchName(fBranchNames[i][sel].Data());
   fSubjectBranchNames[i]->Notify();
}

void AtEventManagerNew::SelectAtRawEvent(Int_t sel)
{
   SelectEventBranch(sel, 0);
}
void AtEventManagerNew::SelectAtEvent(Int_t sel)
{
   SelectEventBranch(sel, 1);
}
void AtEventManagerNew::SelectAtPatternEvent(Int_t sel)
{
   SelectEventBranch(sel, 2);
}

void AtEventManagerNew::SelectEvent()
{
   GotoEvent(fCurrentEvent->GetIntNumber());
}

void AtEventManagerNew::RedrawEvent()
{
   GotoEvent(fEntry);
   DrawPad(fPadNum);
}
void AtEventManagerNew::GotoEvent(Int_t event)
{
   fEntry = event;
   std::cout << cWHITERED << " Event number : " << fEntry << cNORMAL << std::endl;
   FairRunAna::Instance()->Run((Long64_t)event);
}

void AtEventManagerNew::NextEvent()
{
   GotoEvent(fEntry + 1);
}

void AtEventManagerNew::PrevEvent()
{
   GotoEvent(fEntry - 1);
}

// Runs on any interaction with pad plane
void AtEventManagerNew::SelectPad()
{

   int event = gPad->GetEvent();

   if (event != 11) // Only continue if this was a left mouse click (rather than anything else)
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
   std::cout << " ==========================" << std::endl;
   std::cout << " Bin number selected : " << bin << " Bin name :" << bin_name << std::endl;

   AtMap *tmap = AtEventManagerNew::Instance()->GetMap();
   if (tmap == nullptr) {
      LOG(fatal) << "AtMap not set! Pass a valid map to the constructor of AtEventManagerNew!";
   } else {
      Int_t tPadNum = tmap->BinToPad(bin);
      std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
      std::cout << " Electronic mapping: " << tmap->GetPadRef(tPadNum) << std::endl;
      std::cout << " Raw Event Pad Num " << tPadNum << std::endl;
      AtEventManagerNew::Instance()->DrawPad(tPadNum);
   }
}

void AtEventManagerNew::DrawPad(Int_t padNum)
{
   fPadNum = padNum;
   fTabTask->DrawPad(padNum);
}

void AtEventManagerNew::RunEvent()
{
   FairRun::Instance()->Run((Long64_t)fEntry);
}

void AtEventManagerNew::RegisterDataHandles()
{
   if (fTabTask == nullptr)
      LOG(fatal) << "Cannot register data handles without a AtTabTask! Was it added to the AtEventManager?";
   for (auto &branchSubject : fSubjectBranchNames) {
      fTabTask->AddDataSourceToTabs(branchSubject.get());
   }
}
