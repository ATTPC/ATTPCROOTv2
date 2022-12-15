#include "AtEventTabMacro.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"
#include "AtMap.h"
#include "AtRawEvent.h"

#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveManager.h>
#include <TEveWindow.h>
#include <TGTab.h>
#include <TROOT.h> // for TROOT, gROOT
#include <TRootEmbeddedCanvas.h>
#include <TStyle.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtEventTabMacro);

AtEventTabMacro::AtEventTabMacro()
   : fRawEvent(nullptr), fEvent(nullptr), fDetmap(nullptr), fCvsMacro(nullptr), fCols(1), fRows(1),
     fTabName("Macro")
{
}

void AtEventTabMacro::Init()
{

   std::cout << " =====  AtEventTabMacro::Init =====" << std::endl;

   // gROOT->Reset();
   fEventManager = AtEventManagerNew::Instance();

   if (fDetmap == nullptr) {
      LOG(fatal) << "Map was never set using the function SetMap() in AtEventTabMacro!";
   }
   //******* NO CALLS TO TCANVAS BELOW THIS ONE

   if (fTabName == "Macro") {
      char name[20];
      sprintf(name, "Macro %i", fTaskNumber);
      fTabName = name;
   }

   std::cout << " AtEventTabMacro::Init : Initialization complete! "
             << "\n";
}

void AtEventTabMacro::MakeTab()
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

   sprintf(name, "Macro Canvas %i", fTaskNumber);
   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsMacro = new TCanvas("name");
   fCvsMacro->ToggleEditor();
   slot->StopEmbedding();
}

void AtEventTabMacro::DrawEvent(AtRawEvent *rawEvent, AtEvent *event)
{
   fRawEvent = rawEvent;
   fEvent = event;
   for (int i = 0; i < fCols * fRows; i++) {
      fCvsMacro->cd(i + 1);
      gPad->Clear();
      auto it = fDrawEventMap.find(i);
      if (it == fDrawEventMap.end()) {
         return;
      }
      else {
         (it->second)(fRawEvent, fEvent);
         UpdateCvsMacro();
      }
   }
}

void AtEventTabMacro::DrawPad(Int_t padNum)
{
   for (int i = 0; i < fCols * fRows; i++) {
      fCvsMacro->cd(i + 1);
      gPad->Clear();
      auto it = fDrawPadMap.find(i);
      if (it == fDrawPadMap.end()) {
         return;
      }
      else {
         (it->second)(fRawEvent, fEvent, padNum);
         UpdateCvsMacro();
      }
   }
}

void AtEventTabMacro::Reset() {
   fRawEvent = nullptr;
   fEvent = nullptr;
   fCvsMacro->Clear();
}

void AtEventTabMacro::SetDrawEventFunction(Int_t pos, std::function<void(AtRawEvent (*), AtEvent (*))> function) {
   fDrawEventMap.emplace(pos, function);
}

void AtEventTabMacro::SetDrawPadFunction(Int_t pos, std::function<void(AtRawEvent (*), AtEvent (*), Int_t)> function) {
   fDrawPadMap.emplace(pos, function);
}

void AtEventTabMacro::UpdateCvsMacro()
{
   fCvsMacro->Modified();
   fCvsMacro->Update();
}
