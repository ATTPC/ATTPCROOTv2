#include "AtTabMacro.h"

#include "AtViewerManager.h"

#include <FairLogger.h> // for LOG

#include <TCanvas.h>
#include <TChain.h>
#include <TEveWindow.h>

#include <cstdio> // for sprintf
#include <iostream>
#include <utility> // for move, pair

namespace DataHandling {
class AtSubject;
}
constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabMacro);

AtTabMacro::AtTabMacro(int nRow, int nCol, TString name) : AtTabCanvas(name, nRow, nCol), fTree(nullptr)
{
   if (AtViewerManager::Instance() == nullptr)
      throw "AtViewerManager must be initialized before creating tabs!";

   fPadNum = &AtViewerManager::Instance()->GetPadNum();
   fPadNum->Attach(this);
}
AtTabMacro::~AtTabMacro()
{
   fPadNum->Detach(this);
}
void AtTabMacro::InitTab()
{

   std::cout << " =====  AtTabMacro::Init =====" << std::endl;

   if (fTabName == "Macro") {
      fTabName = TString::Format(fTabName + " %d", fTabId);
   }

   std::cout << " AtTabMacro::Init : Initialization complete! "
             << "\n";
}

void AtTabMacro::MakeTab(TEveWindowSlot *slot)
{
   AtTabCanvas::MakeTab(slot);

   DrawTree();
}

void AtTabMacro::DrawTree()
{
   for (int i = 0; i < fCols * fRows; i++) {
      fCanvas->cd(i + 1);

      auto it = fDrawTreeMap.find(i);
      if (it == fDrawTreeMap.end()) {
         return;
      } else {
         // std::cout << "Processing treedraw function" << std::endl;
         (it->second)(fTree);
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::Exec()
{
   for (int i = 0; i < fCols * fRows; i++) {
      fCanvas->cd(i + 1);
      auto it = fDrawEventMap.find(i);
      if (it == fDrawEventMap.end()) {
         return;
      } else {
         (it->second)(fTabInfo.get());
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::Update(DataHandling::AtSubject *sub)
{

   if (sub != fPadNum)
      return;

   for (int i = 0; i < fCols * fRows; i++) {
      fCanvas->cd(i + 1);
      auto it = fDrawPadMap.find(i);
      if (it == fDrawPadMap.end()) {
         return;
      } else {
         (it->second)(fTabInfo.get(), fPadNum->Get());
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::SetInputTree(TString fileName, TString treeName)
{
   if (fTree != nullptr) {
      LOG(error) << "Tree already set. Cannot set again.";
      return;
   }
   fTree = new TChain(treeName);
   std::cout << treeName << " chain set" << std::endl;
   fTree->Add(fileName);
   std::cout << fileName << " added" << std::endl;
}

void AtTabMacro::SetDrawTreeFunction(Int_t pos, std::function<void(TTree(*))> function)
{
   fDrawTreeMap.emplace(pos, function);
}

void AtTabMacro::SetDrawEventFunction(Int_t pos, std::function<void(AtTabInfo(*))> function)
{
   fDrawEventMap.emplace(pos, function);
}

void AtTabMacro::SetDrawPadFunction(Int_t pos, std::function<void(AtTabInfo(*), Int_t)> function)
{
   fDrawPadMap.emplace(pos, function);
}

void AtTabMacro::UpdateCvsMacro()
{
   fCanvas->Modified();
   fCanvas->Update();
}
