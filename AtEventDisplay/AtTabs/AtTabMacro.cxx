#include "AtTabMacro.h"

#include "AtTabInfo.h"
#include "AtViewerManager.h"

#include <FairLogger.h> // for LOG

#include <TCanvas.h>
#include <TChain.h>
#include <TEveBrowser.h>
#include <TEveManager.h>
#include <TEveWindow.h>
#include <TVirtualPad.h> // for TVirtualPad, gPad

#include <stdio.h> // for sprintf

#include <iostream>
#include <utility> // for move, pair
#include <vector>  // for allocator

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabMacro);

AtTabMacro::AtTabMacro() : fDetmap(nullptr), fCvsMacro(nullptr), fCols(1), fRows(1), fTabName("Macro"), fTree(nullptr)
{
}

void AtTabMacro::InitTab()
{

   std::cout << " =====  AtTabMacro::Init =====" << std::endl;

   //******* NO CALLS TO TCANVAS BELOW THIS ONE

   if (fTabName == "Macro") {
      char name[20];
      sprintf(name, "Macro %i", fTabNumber);
      fTabName = name;
   }

   std::cout << " AtTabMacro::Init : Initialization complete! "
             << "\n";
}

void AtTabMacro::MakeTab()
{
   TEveWindowSlot *slot = nullptr;
   TEveWindowPack *pack = nullptr;

   char name[20];

   // 3D
   slot = TEveWindow::CreateWindowInTab(gEve->GetBrowser()->GetTabRight());
   pack = slot->MakePack();
   pack->SetElementName(fTabName);
   pack->SetHorizontal();
   // pack->SetVertical();
   pack->SetShowTitleBar(kFALSE);

   sprintf(name, "Macro Canvas %i", fTabNumber);
   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsMacro = new TCanvas("name");
   fCvsMacro->ToggleEditor();
   slot->StopEmbedding();

   DrawTree();
}

void AtTabMacro::DrawTree()
{
   for (int i = 0; i < fCols * fRows; i++) {
      fCvsMacro->cd(i + 1);
      // gPad->Clear();
      auto it = fDrawTreeMap.find(i);
      if (it == fDrawTreeMap.end()) {
         return;
      } else {
         std::cout << "Processing treedraw function" << std::endl;
         (it->second)(fTree);
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::DrawEvent()
{
   for (int i = 0; i < fCols * fRows; i++) {
      fCvsMacro->cd(i + 1);
      // gPad->Clear();
      auto it = fDrawEventMap.find(i);
      if (it == fDrawEventMap.end()) {
         return;
      } else {
         (it->second)(fTabInfo.get());
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::DrawPad(Int_t padNum)
{

   for (int i = 0; i < fCols * fRows; i++) {
      fCvsMacro->cd(i + 1);
      // gPad->Clear();
      auto it = fDrawPadMap.find(i);
      if (it == fDrawPadMap.end()) {
         return;
      } else {
         (it->second)(fTabInfo.get(), padNum);
         UpdateCvsMacro();
      }
   }
}

void AtTabMacro::Reset()
{
   // fCvsMacro->Clear();
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
   fCvsMacro->Modified();
   fCvsMacro->Update();
}
