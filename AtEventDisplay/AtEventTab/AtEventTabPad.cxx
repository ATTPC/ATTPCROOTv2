#include "AtEventTabPad.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtRawEvent.h"

#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveManager.h>
#include <TEveWindow.h>
#include <TGTab.h>
#include <TH1D.h>
#include <TROOT.h> // for TROOT, gROOT
#include <TRootEmbeddedCanvas.h>
#include <TStyle.h>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtEventTabPad);

AtEventTabPad::AtEventTabPad()
   : fRawEvent(nullptr), fEvent(nullptr), fPad(nullptr), fDetmap(nullptr), fCvsPad(nullptr), fCols(1), fRows(1),
     fTabName("AtPad")
{
}

void AtEventTabPad::Init()
{

   std::cout << " =====  AtEventTabPad::Init =====" << std::endl;

   // gROOT->Reset();
   fEventManager = AtEventManagerNew::Instance();

   if (fDetmap == nullptr) {
      LOG(fatal) << "Map was never set using the function SetMap() in AtEventTabPad!";
   }
   //******* NO CALLS TO TCANVAS BELOW THIS ONE

   if (fTabName == "AtPad") {
      char name[20];
      sprintf(name, "AtPad %i", fTaskNumber);
      fTabName = name;
   }

   std::cout << " AtEventTabPad::Init : Initialization complete! "
             << "\n";
}

void AtEventTabPad::MakeTab()
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

   sprintf(name, "AtPad Canvas %i", fTaskNumber);
   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();
   fCvsPad = new TCanvas("AtPad Canvas");
   fCvsPad->ToggleEditor();
   slot->StopEmbedding();
}

void AtEventTabPad::DrawEvent(AtRawEvent *rawEvent, AtEvent *event)
{
   fRawEvent = rawEvent;
   fEvent = event;
}

void AtEventTabPad::DrawPad(Int_t padNum)
{
   fPad = nullptr;
   fCvsPad->Clear();
   if (fRawEvent == nullptr) {
      std::cout << "fRawEvent is nullptr for tab " << fTaskNumber << "! Please set the raw event branch." << std::endl;
      return;
   } else {
      fPad = fRawEvent->GetPad(padNum);
      if (fPad == nullptr) {
         std::cout << "Pad " << padNum << " does not exist in raw event for tab " << fTaskNumber << "!" << std::endl;
         return;
      } else {
         fCvsPad->Divide(fCols, fRows);
         for (int i = 0; i < fCols * fRows; i++) {
            fCvsPad->cd(i + 1);
            DrawPosition(i);
            UpdateCvsPad();
         }
         // DrawWave(PadNum);
         UpdateCvsPad();
      }
   }
}

void AtEventTabPad::Reset() {
   fRawEvent = nullptr;
}

void AtEventTabPad::DrawPosition(Int_t pos)
{
   std::array<Double_t, 512> array = {};
   auto it = fDrawMap.find(pos);
   if (it == fDrawMap.end()) {
      return;
   } else {
      switch (it->second.first) {
      case PadDrawType::kADC: array = fPad->GetADC(); break;

      case PadDrawType::kRawADC:
         { 
            auto padArray = fPad->GetRawADC();
            for (int i = 0; i < padArray.size(); i++)
            array[i] = padArray[i];
            break;
         }

      case PadDrawType::kArrAug:
         {
            auto augIt = fAugNames.find(pos);
            if (augIt == fAugNames.end()) {
               return;
            } else {
               array = dynamic_cast<AtPadArray *>(fPad->GetAugment(augIt->second.Data()))->GetArray();
            }
            break;
         }

      default: return;
      }
      auto padHist = it->second.second;
      padHist->Reset();
      for (int i = 0; i < 512; i++) {
         padHist->SetBinContent(i + 1, array[i]);
         padHist->Draw();
      }
   }
}

void AtEventTabPad::SetDraw(Int_t pos, PadDrawType type)
{
   char name[20];
   sprintf(name, "padHist_Tab%i_%i", fTaskNumber, pos);
   TH1D *padHist = new TH1D(name, name, 512, 0, 512);
   fDrawMap.emplace(pos, std::make_pair(type, padHist));
}

void AtEventTabPad::SetDrawADC(Int_t pos)
{
   SetDraw(pos, PadDrawType::kADC);
}

void AtEventTabPad::SetDrawRawADC(Int_t pos)
{
   SetDraw(pos, PadDrawType::kRawADC);
}

void AtEventTabPad::SetDrawArrayAug(Int_t pos, TString augName)
{
   SetDraw(pos, PadDrawType::kArrAug);
   fAugNames.emplace(pos, augName);
}

void AtEventTabPad::UpdateCvsPad()
{
   fCvsPad->Modified();
   fCvsPad->Update();
}
