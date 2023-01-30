#include "AtTabMacro.h"

#include "AtViewerManager.h"

#include <TCanvas.h>

#include <iostream>
#include <memory> // for allocator, unique_ptr

namespace DataHandling {
class AtSubject;
}
constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabMacro);

AtTabMacro::AtTabMacro(int nRow, int nCol, TString name) : AtTabCanvas(name, nRow, nCol)
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
   for (auto &[pos, func] : fDrawTreeMap) {
      fCanvas->cd(pos + 1);
      func(fTabInfo.get());
   }
   UpdateCanvas();
}

void AtTabMacro::Exec()
{
   for (auto &[pos, func] : fDrawEventMap) {
      fCanvas->cd(pos + 1);
      func(fTabInfo.get());
   }
   UpdateCanvas();
}

void AtTabMacro::Update(DataHandling::AtSubject *sub)
{

   if (sub != fPadNum)
      return;
   for (auto &[pos, func] : fDrawPadMap) {
      fCanvas->cd(pos + 1);
      func(fTabInfo.get(), fPadNum->Get());
   }
   UpdateCanvas();
}

void AtTabMacro::SetDrawTreeFunction(std::function<void(AtTabInfo(*))> function, int row, int col)
{
   fDrawTreeMap.emplace(row * fCols + col, function);
}

void AtTabMacro::SetDrawEventFunction(std::function<void(AtTabInfo(*))> function, int row, int col)
{
   fDrawEventMap.emplace(row * fCols + col, function);
}

void AtTabMacro::SetDrawPadFunction(std::function<void(AtTabInfo(*), Int_t)> function, int row, int col)
{
   fDrawPadMap.emplace(row * fCols + col, function);
}
