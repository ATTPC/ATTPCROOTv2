#include "AtViewerManager.h"

#include "AtMap.h"
#include "AtSidebarFrames.h"
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

ClassImp(AtViewerManager);

AtViewerManager *AtViewerManager::fInstance = nullptr;
AtViewerManager *AtViewerManager::Instance()
{
   return fInstance;
}

AtViewerManager::AtViewerManager(std::shared_ptr<AtMap> map)
   : f3DThresDisplay(nullptr), fTabTask(std::make_unique<AtTabTask>()), fMap(map)

{
   if (fInstance != nullptr)
      LOG(fatal) << "Attempting to create a second instance of AtViewerManager! Only one is allowed!";
   fInstance = this;

   // In order to get the sidebar to actually be *in* the sidebar we need to create the EveManager
   // and embedd it there.
   TEveManager::Create();
   TEveBrowser *browser = gEve->GetBrowser();
   browser->StartEmbedding(TRootBrowser::kLeft);

   fSidebar = new AtEventSidebar(fEntry, fRawEventBranch, fEventBranch, fPatternEventBranch);

   browser->StopEmbedding();
   browser->SetTabTitle("Control", 0);
}

AtViewerManager::~AtViewerManager()
{
   fInstance = nullptr;
   fEntry.Detach(this);
   fRawEventBranch.Detach(this);
   fEventBranch.Detach(this);
   fPatternEventBranch.Detach(this);
}

void AtViewerManager::AddTask(FairTask *task)
{
   if (dynamic_cast<AtTabTask *>(task) == nullptr)
      FairRunAna::Instance()->AddTask(task);
   else
      LOG(error) << "You cannot add a AtTabTask! This is an implementation specific task that is soley managed by the "
                    "AtViewerManager class.";
}
void AtViewerManager::AddTab(std::unique_ptr<AtTabBase> tab)
{
   fTabTask->AddTab(std::move(tab));
}

void AtViewerManager::Init()
{
   gStyle->SetOptTitle(0);
   gStyle->SetPalette(55);

   // Add the AtTabTask as the last task in the run so it can access
   // what is created in earlier tasks in the run
   FairRunAna::Instance()->AddTask(fTabTask.get());
   FairRunAna::Instance()->Init();

   // Everything is loaded so construct the list of branch names
   GenerateBranchLists();

   fSidebar->FillFrames(); // Creates the entire sidebar GUI

   GotoEvent(0);
   std::cout << "End of AtViewerManager" << std::endl;
}

void AtViewerManager::GenerateBranchLists()
{
   GotoEvent(0);

   auto ioMan = FairRootManager::Instance();

   // Loop through the entire branch list and map class type to branch name in fBranchNames
   for (int i = 0; i < ioMan->GetBranchNameList()->GetSize(); i++) {

      auto branchName = ioMan->GetBranchName(i);
      auto branchArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(branchName));
      if (branchArray == nullptr)
         continue;

      // Loop until there is something in this branch
      while (branchArray->GetSize() == 0)
         NextEvent();

      auto type = branchArray->At(0)->ClassName();
      fBranchNames[type].push_back(branchName);
      LOG(info) << "Found " << branchName << " with type " << type;
   }
}

void AtViewerManager::GotoEvent(Int_t event)
{
   fEntry.Set(event);
   LOG(info) << cWHITERED << " Event number : " << fEntry.Get() << cNORMAL;
   FairRunAna::Instance()->Run((Long64_t)event);
   DrawPad(fPadNum);
}

// Runs on any interaction with pad plane
void AtViewerManager::SelectPad()
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

   AtMap *tmap = AtViewerManager::Instance()->GetMap();
   if (tmap == nullptr) {
      LOG(fatal) << "AtMap not set! Pass a valid map to the constructor of AtViewerManager!";
   } else {
      Int_t tPadNum = tmap->BinToPad(bin);
      std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
      std::cout << " Electronic mapping: " << tmap->GetPadRef(tPadNum) << std::endl;
      std::cout << " Raw Event Pad Num " << tPadNum << std::endl;
      AtViewerManager::Instance()->DrawPad(tPadNum);
   }
}

void AtViewerManager::DrawPad(Int_t padNum)
{
   fPadNum = padNum;
   fTabTask->DrawTabPads(padNum);
}

void AtViewerManager::Update(DataHandling::Subject *subject)
{
   if (subject == &fEntry)
      GotoEvent(fEntry.Get());
}
