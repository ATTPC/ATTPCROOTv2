#include "AtViewerManager.h"

#include "AtMap.h"
#include "AtSidebarFrames.h"
#include "AtTabBase.h"

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

AtViewerManager::AtViewerManager(std::shared_ptr<AtMap> map) : fMap(map)
{
   if (fInstance != nullptr)
      LOG(fatal) << "Attempting to create a second instance of AtViewerManager! Only one is allowed!";
   fInstance = this;

   // In order to get the sidebar to actually be *in* the sidebar we need to create the EveManager
   // and embedd it there.
   TEveManager::Create(true, "IV"); // Create a mapped manager without the file viewer
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
   FairRunAna::Instance()->AddTask(task);
}
void AtViewerManager::AddTab(std::unique_ptr<AtTabBase> tab)
{
   fTabs.push_back(std::move(tab));
}

void AtViewerManager::Init()
{
   gStyle->SetOptTitle(0);
   gStyle->SetPalette(55);

   FairRunAna::Instance()->Init();
   for (auto &tab : fTabs)
      tab->Init();

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
   FairRunAna::Instance()->Run((Long64_t)event);
   for (auto &tab : fTabs)
      tab->Exec();

   DrawPad(fPadNum);
}

// Runs on any interaction with pad plane

void AtViewerManager::DrawPad(Int_t padNum)
{
   fPadNum = padNum;
   for (auto &tab : fTabs)
      tab->DrawPad(padNum);
}

void AtViewerManager::Update(DataHandling::Subject *subject)
{
   if (subject == &fEntry)
      GotoEvent(fEntry.Get());
}
