#include "AtTabPad.h"

#include "AtAuxPad.h" // for AtAuxPad
#include "AtDataManip.h"
#include "AtEvent.h"
#include "AtHit.h" // for AtHit
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtRawEvent.h"
#include "AtTabInfo.h"
#include "AtViewerManager.h"

#include <FairLogger.h>

#include <TCanvas.h>
#include <TF1.h>
#include <TH1.h> // for TH1D

#include <iostream> // for operator<<, basic_ostream::operator<<
#include <memory>   // for make_unique, allocator, unique_ptr
namespace DataHandling {
class AtSubject;
}

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

ClassImp(AtTabPad);

AtTabPad::AtTabPad(int nRow, int nCol, TString name) : AtTabCanvas(name, nRow, nCol)
{
   if (AtViewerManager::Instance() == nullptr)
      throw "AtViewerManager must be initialized before creating tabs!";

   fPadNum = &AtViewerManager::Instance()->GetPadNum();
   fPadNum->Attach(this);
}
AtTabPad::~AtTabPad()
{
   fPadNum->Detach(this);
}

void AtTabPad::InitTab()
{
   std::cout << " =====  AtEventTabPad::Init =====" << std::endl;

   if (fTabName == "AtPad")
      fTabName = TString::Format(fTabName + " %d", fTabId);

   auto man = AtViewerManager::Instance();
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtEvent>>(man->GetEventBranch()));
   fTabInfo->AddAugment(std::make_unique<AtTabInfoFairRoot<AtRawEvent>>(man->GetRawEventBranch()));

   std::cout << " AtEventTabPad::Init : Initialization complete! "
             << "\n";
}

void AtTabPad::MakeTab(TEveWindowSlot *slot)
{
   AtTabCanvas::MakeTab(slot);
}

void AtTabPad::Exec()
{
   auto fRawEvent = GetFairRootInfo<AtRawEvent>();
   if (fRawEvent == nullptr) {
      LOG(debug) << "fRawEvent is nullptr for tab " << fTabId << "! Please set the raw event branch.";
      return;
   }

   // Redraw any Auxiliary channels
   for (auto &[pos, toDraw] : fDrawMap) {
      fCanvas->cd(pos + 1);
      auto hist = toDraw.second;
      if (toDraw.first == PadDrawType::kAuxPad) {
         hist->Reset();

         auto auxPad = fRawEvent->GetAuxPad(fAugNames[pos]);
         if (auxPad != nullptr)
            DrawAdc(hist, *auxPad);
         hist->Draw();
      }
   }
   UpdateCvsPad();
}

void AtTabPad::Update(DataHandling::AtSubject *sub)
{
   if (sub == fPadNum)
      DrawPad();
}

void AtTabPad::DrawPad()
{

   auto fRawEvent = GetFairRootInfo<AtRawEvent>();

   if (fRawEvent == nullptr) {
      LOG(debug) << "fRawEvent is nullptr for tab " << fTabId << "! Please set the raw event branch.";
      return;
   }

   auto fPad = fRawEvent->GetPad(fPadNum->Get());

   for (auto &[pos, toDraw] : fDrawMap) {
      if (toDraw.first == PadDrawType::kAuxPad)
         continue;

      fCanvas->cd(pos + 1);
      auto hist = toDraw.second;

      if (fPad == nullptr) {
         hist->Reset();
         hist->Draw();
         continue;
      }

      switch (toDraw.first) {
      case PadDrawType::kADC: DrawAdc(hist, *fPad); break;
      case PadDrawType::kRawADC: DrawRawAdc(hist, *fPad); break;
      case PadDrawType::kArrAug: DrawArrayAug(hist, *fPad, fAugNames[pos]); break;
      case PadDrawType::kFPN: DrawFPN(hist, *fPad); break;
      case PadDrawType::kAuxPad:
         auto auxPad = fRawEvent->GetAuxPad(fAugNames[pos]);
         if (auxPad != nullptr)
            DrawAdc(hist, *auxPad);
         break;
      }

      if (fDrawHits.find(pos) != fDrawHits.end())
         DrawHit(*fPad, fDrawHits[pos]);
   }

   UpdateCvsPad();
}

void AtTabPad::DrawAdc(TH1D *hist, const AtPad &pad)
{
   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, pad.GetADC()[i]);
   }
   hist->Draw();
}

void AtTabPad::DrawFPN(TH1D *hist, const AtPad &pad)
{

   auto fRawEvent = GetFairRootInfo<AtRawEvent>();
   if (fRawEvent == nullptr) {
      LOG(error) << "fRawEvent is nullptr for tab " << fTabId << "! Please set the raw event branch.";
      return;
   }
   auto map = AtViewerManager::Instance()->GetMap();
   auto fpnPad = fRawEvent->GetFpn(map->GetNearestFPN(pad.GetPadNum()));

   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, fpnPad->GetRawADC()[i]);
   }
   hist->Draw();
}

void AtTabPad::DrawRawAdc(TH1D *hist, const AtPad &pad)
{
   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, pad.GetRawADC()[i]);
   }
   hist->Draw();
}

void AtTabPad::DrawArrayAug(TH1D *hist, const AtPad &pad, TString augName)
{
   auto aug = dynamic_cast<const AtPadArray *>(pad.GetAugment(augName.Data()));
   if (aug == nullptr)
      return;

   for (int i = 0; i < 512; i++) {
      hist->SetBinContent(i + 1, aug->GetArray()[i]);
   }
   hist->Draw();
}

void AtTabPad::DrawHit(const AtPad &pad, TF1Vec &vec)
{
   vec.clear();

   // Loop through all hits and
   auto event = GetFairRootInfo<AtEvent>();
   for (auto &hit : event->GetHits()) {
      if (hit->GetPadNum() != pad.GetPadNum())
         continue;

      LOG(debug) << "Drawing hit with charge " << hit->GetCharge();
      LOG(debug) << hit->GetPosition().Z() << " " << hit->GetPositionSigma().Z();
      auto func = AtTools::GetHitFunctionTB(*hit);
      if (func != nullptr)
         vec.push_back(std::move(func));
   }

   for (auto &func : vec) {
      LOG(debug) << "Drawing hit function";
      func->Draw("SAME");
   }
}
std::string AtTabPad::GetName(int pos, PadDrawType type)
{
   switch (type) {
   case PadDrawType::kADC: return "ADC";
   case PadDrawType::kRawADC: return "Raw ADC";
   case PadDrawType::kArrAug: return fAugNames[pos];
   case PadDrawType::kFPN: return "Closest FPN";
   case PadDrawType::kAuxPad: return fAugNames[pos];
   }
   return "";
}
void AtTabPad::SetDraw(Int_t pos, PadDrawType type)
{
   auto name = TString::Format("padHist_Tab%i_%i", fTabId, pos);
   TH1D *padHist = new TH1D(name, GetName(pos, type).c_str(), 512, 0, 512);
   fDrawMap.emplace(pos, std::make_pair(type, padHist));
}

void AtTabPad::DrawHits(int row, int col)
{
   fDrawHits[row * fCols + col] = TF1Vec();
}
void AtTabPad::DrawADC(int row, int col)
{
   SetDraw(row * fCols + col, PadDrawType::kADC);
}

void AtTabPad::DrawFPN(int row, int col)
{
   SetDraw(row * fCols + col, PadDrawType::kFPN);
}

void AtTabPad::DrawRawADC(int row, int col)
{
   SetDraw(row * fCols + col, PadDrawType::kRawADC);
}

void AtTabPad::DrawArrayAug(TString augName, int row, int col)
{
   fAugNames.emplace(row * fCols + col, augName);
   SetDraw(row * fCols + col, PadDrawType::kArrAug);
}

void AtTabPad::DrawAuxADC(TString auxName, int row, int col)
{
   fAugNames.emplace(row * fCols + col, auxName);
   SetDraw(row * fCols + col, PadDrawType::kAuxPad);
}

void AtTabPad::UpdateCvsPad()
{
   fCanvas->Modified();
   fCanvas->Update();
}
