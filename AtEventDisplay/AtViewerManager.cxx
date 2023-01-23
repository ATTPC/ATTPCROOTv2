#include "AtViewerManager.h"

#include "AtEventSidebar.h" // for AtEventSidebar
#include "AtTabBase.h"      // for AtTabBase

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager
#include <FairRunAna.h>      // for FairRunAna

#include <Rtypes.h>       // for ClassImp, Long64_t, TGenericClassInfo
#include <TClonesArray.h> // for TClonesArray
#include <TEveBrowser.h>  // for TEveBrowser
#include <TEveManager.h>  // for TEveManager, gEve
#include <TList.h>        // for TList
#include <TObject.h>      // for TObject
#include <TRootBrowser.h> // for TRootBrowser, TRootBrowser::kLeft
#include <TString.h>      // for TString, operator<<, operator<
#include <TStyle.h>       // for TStyle, gStyle

#include <iostream> // for operator<<, endl, basic_ostream, cout
#include <utility>  // for move

namespace DataHandling {
class AtSubject;
}

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";
constexpr auto cWHITERED = "\033[37;41m";

using namespace std;

ClassImp(AtViewerManager);

AtViewerManager *AtViewerManager::fInstance = nullptr;
AtViewerManager *AtViewerManager::Instance()
{
   return fInstance;
}

AtViewerManager::AtViewerManager(std::shared_ptr<AtMap> map) : fMap(std::move(map))
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
   fEntry.Attach(this);
}

AtViewerManager::~AtViewerManager()
{
   fInstance = nullptr;
   fEntry.Detach(this);
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
   LOG(info) << "Generating branch list";
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
   LOG(info) << "Done generating branch list";
}

void AtViewerManager::GotoEventImpl()
{
   FairRunAna::Instance()->Run((Long64_t)fEntry.Get());
   for (auto &tab : fTabs)
      tab->Exec();

   fPadNum.Notify(); // Inform everyone they should act as is the pad changed
}

void AtViewerManager::Update(DataHandling::AtSubject *subject)
{
   if (subject == &fEntry) {
      GotoEventImpl();
   }
}
