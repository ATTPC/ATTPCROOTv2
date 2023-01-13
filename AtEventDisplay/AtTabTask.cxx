#include "AtTabTask.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEventManagerNew.h" // for AtEventManager

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

#include <TROOT.h> // for TROOT, gROOT
#include <TStyle.h>

#include <iostream>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

// using namespace std;

ClassImp(AtTabTask);

AtTabTask::AtTabTask()
{

}

AtTabTask::~AtTabTask() = default;

InitStatus AtTabTask::Init()
{

   std::cout << " =====  AtTabTask::Init =====" << std::endl;

   for(int i = 0; i < fTabs.size(); i++) {
      fTabs[i]->SetTabNumber(i);
      fTabs[i]->Init();
      fTabs[i]->MakeTab();
   }

   std::cout << " AtTabTask::Init : Initialization complete! "
             << "\n";
   return kSUCCESS;
}

void AtTabTask::Exec(Option_t *option)
{
   Reset();

   for(int i = 0; i < fTabs.size(); i++) {
      fTabs[i]->Update();
      fTabs[i]->DrawEvent();
   }
}

void AtTabTask::Reset()
{
   for(int i = 0; i < fTabs.size(); i++) {
      fTabs[i]->Reset();
   }
}

void AtTabTask::DrawPad(Int_t PadNum)
{
   for(int i = 0; i < fTabs.size(); i++) {
      fTabs[i]->DrawPad(PadNum);
   }
}