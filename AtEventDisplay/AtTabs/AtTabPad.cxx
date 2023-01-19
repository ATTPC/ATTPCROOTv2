#include "AtTabPad.h"

#include "AtEvent.h"
#include "AtEventManagerNew.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtRawEvent.h"
#include "AtTabInfo.h"

#include <FairLogger.h> // for LOG

#include <TCanvas.h>
#include <TEveBrowser.h>
#include <TEveManager.h>
#include <TEveWindow.h>
#include <TH1.h> // for TH1D

#include <array>    // for array
#include <cstdio>   // for sprintf
#include <iostream> // for operator<<, basic_ostream::operator<<

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabPad);

void AtTabPad::InitTab()
{

   std::cout << " =====  AtEventTabPad::Init =====" << std::endl;

   if (fTabName == "AtPad")
      fTabName = TString::Format(fTabName + " %d", fTabNumber);

   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtEvent>>(fEventBranch));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtRawEvent>>(fRawEventBranch));

   std::cout << " AtEventTabPad::Init : Initialization complete! "
             << "\n";
}

void AtTabPad::MakeTab()
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

   sprintf(name, "AtPad Canvas %i", fTabNumber);
   slot = pack->NewSlot();
   TEveWindowPack *pack2 = slot->MakePack();
   pack2->SetShowTitleBar(kFALSE);
   pack2->SetVertical();
   slot = pack2->NewSlot();
   slot->StartEmbedding();

   // Doing this here so it is only done once. Repeated Clear() and Divide() calls were causing
   // a seg fault for reasons I do not understand.
   fCvsPad = new TCanvas(name);
   fCvsPad->Divide(fCols, fRows);

   fCvsPad->ToggleEditor();
   slot->StopEmbedding();
}

void AtTabPad::UpdateTab() {}

void AtTabPad::DrawPad(Int_t padNum)
{

   AtRawEvent *fRawEvent = GetFairRootInfo<AtRawEvent>();

   if (fRawEvent == nullptr) {
      std::cout << "fRawEvent is nullptr for tab " << fTabNumber << "! Please set the raw event branch." << std::endl;
      return;
   }

   auto fPad = fRawEvent->GetPad(padNum);
   if (fPad == nullptr) {
      std::cout << "Pad " << padNum << " does not exist in raw event for tab " << fTabNumber << "!" << std::endl;
      return;
   }

   fCvsPad->Clear("D");
   for (int i = 0; i < fCols * fRows; i++) {
      fCvsPad->cd(i + 1);
      DrawPosition(i, fPad);
      // UpdateCvsPad();
   }
   // DrawWave(PadNum);
   UpdateCvsPad();
}

void AtTabPad::Reset()
{
   // fRawEvent = nullptr;
}

void AtTabPad::DrawPosition(Int_t pos, AtPad *fPad)
{
   LOG(debug) << "Drawing position in AtTabPad";
   std::array<Double_t, 512> array = {};
   auto it = fDrawMap.find(pos);
   if (it == fDrawMap.end()) {
      return;
   } else {
      switch (it->second.first) {
      case PadDrawType::kADC: array = fPad->GetADC(); break;

      case PadDrawType::kRawADC: {
         auto padArray = fPad->GetRawADC();
         for (int i = 0; i < padArray.size(); i++)
            array[i] = padArray[i];
         break;
      }

      case PadDrawType::kArrAug: {
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

void AtTabPad::SetDraw(Int_t pos, PadDrawType type)
{
   char name[20];
   sprintf(name, "padHist_Tab%i_%i", fTabNumber, pos);
   TH1D *padHist = new TH1D(name, name, 512, 0, 512);
   fDrawMap.emplace(pos, std::make_pair(type, padHist));
}

void AtTabPad::SetDrawADC(Int_t pos)
{
   SetDraw(pos, PadDrawType::kADC);
}

void AtTabPad::SetDrawRawADC(Int_t pos)
{
   SetDraw(pos, PadDrawType::kRawADC);
}

void AtTabPad::SetDrawArrayAug(Int_t pos, TString augName)
{
   SetDraw(pos, PadDrawType::kArrAug);
   fAugNames.emplace(pos, augName);
}

void AtTabPad::UpdateCvsPad()
{
   fCvsPad->Modified();
   fCvsPad->Update();
}
