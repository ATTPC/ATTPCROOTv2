#include "AtTabTask.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtEventManagerNew.h" // for AtEventManager

#include <iostream>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

// using namespace std;

ClassImp(AtTabTask);

void AtTabTask::AddTab(std::unique_ptr<AtTabBase> tab)
{
   tab->SetTabNumber(fTabs.size());
   fTabs.push_back(std::move(tab));
}

InitStatus AtTabTask::Init()
{

   std::cout << " =====  AtTabTask::Init =====" << std::endl;

   for (int i = 0; i < fTabs.size(); i++) {
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
   LOG(debug) << "Executing all tabs" << std::endl;
   for (auto &fTab : fTabs) {
      fTab->Update();
      fTab->DrawEvent();
   }
}

void AtTabTask::Reset()
{
   for (auto &fTab : fTabs) {
      fTab->Reset();
   }
}

void AtTabTask::DrawTabPads(Int_t PadNum)
{
   LOG(debug) << "Drawing all tabs" << std::endl;
   for (auto &fTab : fTabs) {
      fTab->DrawPad(PadNum);
   }
}
void AtTabTask::AddDataSourceToTabs(DataHandling::Subject *subject)
{
   for (auto &tab : fTabs)
      subject->Attach(tab->GetTabInfo());
}
