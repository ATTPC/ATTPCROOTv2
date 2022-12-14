#include "AtEventTabTask.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtRawEvent.h"     // for AtRawEvent, AuxPadMap
#include "AtEvent.h"
#include "AtEventManagerNew.h" // for AtEventManager

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

#include <TClonesArray.h>
#include <TROOT.h>           // for TROOT, gROOT
#include <TStyle.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

using namespace std;

ClassImp(AtEventTabTask);

AtEventTabTask::AtEventTabTask(std::unique_ptr<AtEventTab> eventTab)
   : fEventManager(nullptr), fRawEvent(nullptr), fEvent(nullptr),
     fTaskNumber(0), fRawEventBranchName("AtRawEvent"), fEventBranchName("AtEvent"), fEventTab(std::move(eventTab))
{

}

AtEventTabTask::~AtEventTabTask() = default;

InitStatus AtEventTabTask::Init()
{

   std::cout << " =====  AtEventTabTask::Init =====" << std::endl;

   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerNew::Instance();

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray) 
      LOG(INFO) << cGREEN << "Raw Event Array Found in branch " << fRawEventBranchName << "." << cNORMAL << std::endl;

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fEventBranchName));
   if (fEventArray)
      LOG(INFO) << cGREEN << "Event Array Found in branch " << fEventBranchName << "." << cNORMAL << std::endl;

   gStyle->SetPalette(55);

   fEventTab->SetTaskNumber(fTaskNumber);
   fEventTab->Init();
   fEventTab->MakeTab();

   std::cout << " AtEventTabTask::Init : Initialization complete! "
             << "\n";
   return kSUCCESS;
}

void AtEventTabTask::Exec(Option_t *option)
{
   Reset();
   
   fRawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
   fEvent = dynamic_cast<AtEvent *>(fEventArray->At(0));
   fEventTab->DrawEvent(fRawEvent, fEvent);
}

void AtEventTabTask::Reset() {
   fEventTab->Reset();
}

void AtEventTabTask::MakeTab() {
   fEventTab->MakeTab();
}

void AtEventTabTask::DrawPad(Int_t PadNum)
{

   fEventTab->DrawPad(PadNum);

}

void AtEventTabTask::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtEventTabTask::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}
