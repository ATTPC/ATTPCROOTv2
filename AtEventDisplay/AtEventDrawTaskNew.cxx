/**
 * @brief Event display task
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */

#include "AtEventDrawTaskNew.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtAuxPad.h"          // for AtAuxPad
#include "AtEvent.h"           // for AtEvent, hitVector
#include "AtEventManagerNew.h" // for AtEventManager
#include "AtFindVertex.h"      //for vertex
#include "AtHit.h"             // for AtHit
#include "AtHitCluster.h"      // for AtHitCluster
#include "AtMap.h"             // for AtMap
#include "AtPad.h"             // for AtPad
#include "AtPadReference.h"
#include "AtPattern.h"
#include "AtPatternEvent.h" // for AtPatternEvent
#include "AtRawEvent.h"     // for AtRawEvent, AuxPadMap
#include "AtTrack.h"        // for AtTrack, operator<<

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

#include <Math/Point3D.h>    // for PositionVector3D
#include <TAttMarker.h>      // for kFullDotMedium
#include <TAxis.h>           // for TAxis
#include <TCanvas.h>         // for TCanvas
#include <TClonesArray.h>    // for TClonesArray
#include <TColor.h>          // for TColor
#include <TEveBoxSet.h>      // for TEveBoxSet, TEveBoxSet::kBT_AABox
#include <TEveElement.h>     // for TEveElement
#include <TEveLine.h>        // for TEveLine
#include <TEveManager.h>     // for TEveManager, gEve
#include <TEvePointSet.h>    // for TEvePointSet
#include <TEveRGBAPalette.h> // for TEveRGBAPalette
#include <TEveTrans.h>       // for TEveTrans
#include <TEveTreeTools.h>   // for TEvePointSelectorConsumer, TEvePoint...
#include <TGraph.h>          // for TGraph
#include <TH1.h>             // for TH1D, TH1I, TH1F
#include <TH2.h>             // for TH2F
#include <TH2Poly.h>         // for TH2Poly
#include <TH3.h>             // for TH3F
#include <TList.h>           // for TList
#include <TNamed.h>          // for TNamed
#include <TObject.h>         // for TObject
#include <TPad.h>            // for TPad
#include <TPaletteAxis.h>    // for TPaletteAxis
#include <TROOT.h>           // for TROOT, gROOT
#include <TRandom.h>         // for TRandom
#include <TSeqCollection.h>  // for TSeqCollection
#include <TStyle.h>          // for TStyle, gStyle
#include <TVirtualPad.h>     // for TVirtualPad, gPad
#include <TVirtualX.h>       // for TVirtualX

#include "S800Ana.h"
#include "S800Calc.h" // for S800Calc, CRDC, MultiHitTOF, IC

#include <algorithm> // for max
#include <array>     // for array
#include <cstdio>    // for sprintf
#include <exception> // for exception
#include <iostream>  // for cout
#include <map>       // for operator!=, _Rb_tree_const_iterator
#include <string>    // for allocator, operator<<
#include <utility>   // for pair

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

using namespace std;

ClassImp(AtEventDrawTaskNew);

AtEventDrawTaskNew::AtEventDrawTaskNew()
   : fEventArray(nullptr), fEventManager(nullptr), fRawevent(nullptr), fDetmap(nullptr), fThreshold(0),
     fHitSet(nullptr), fPadPlanePal(nullptr), fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium),
     fCvsPadPlane(nullptr), fPadPlane(nullptr), fCvsPadWave(nullptr), fPadWave(nullptr), fAtMapPtr(nullptr),
     fMultiHit(0), fTaskNumber(0), fRawEventBranchName("AtRawEvent"), fEventBranchName("AtEventH")
{

   fIsRawData = kFALSE;

   fRGBAPalette = new TEveRGBAPalette(0, 4096);
}

AtEventDrawTaskNew::~AtEventDrawTaskNew() = default;

InitStatus AtEventDrawTaskNew::Init()
{

   std::cout << " =====  AtEventDrawTaskNew::Init =====" << std::endl;

   gROOT->Reset();
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerNew::Instance();

   if (fDetmap == nullptr) {
      LOG(fatal) << "Map was never set using the function SetMap() in AtEventDrawTaskNew!";
      return kFATAL;
   }

   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap.get());

   fEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fEventBranchName));
   if (fEventArray)
      LOG(INFO) << cGREEN << "Event Array Found in branch " << fEventBranchName << "." << cNORMAL << std::endl;

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray) {
      LOG(INFO) << cGREEN << "Raw Event Array Found in branch " << fRawEventBranchName << "." << cNORMAL << std::endl;
      fIsRawData = kTRUE;
   }

   gStyle->SetPalette(55);

   fCvsPadWave = fEventManager->GetCvsPadWave();
   char name[20];
   sprintf(name, "fCvsPadWave_DT%i", fTaskNumber);
   fCvsPadWave->SetName(name);
   gROOT->GetListOfSpecials()->Add(fCvsPadWave);
   DrawPadWave();
   fCvsPadPlane = fEventManager->GetCvsPadPlane(); // There is a problem if the pad plane is drawn first
   fCvsPadPlane->ToggleEventStatus();
   // fCvsPadPlane->AddExec("ex", "AtEventDrawTaskNew::SelectPad(\"fRawEvent\")");
   DrawPadPlane();
   /*fCvsQuadrant1 = fEventManager->GetCvsQuadrant1();
    fCvsQuadrant2 = fEventManager->GetCvsQuadrant2();
    fCvsQuadrant3 = fEventManager->GetCvsQuadrant3();
    fCvsQuadrant4 = fEventManager->GetCvsQuadrant4();
    DrawHoughSpaceProto();*/

   //******* NO CALLS TO TCANVAS BELOW THIS ONE

   std::cout << " AtEventDrawTaskNew::Init : Initialization complete! "
             << "\n";
   return kSUCCESS;
}

void AtEventDrawTaskNew::Exec(Option_t *option)
{
   Reset();

   // if (fRawEventArray)
   //  DrawPadAll();

   if (fEventArray) {
      DrawHitPoints();
   }

   gEve->Redraw3D(kFALSE);

   UpdateCvsPadPlane();
   UpdateCvsPadWave();
}

void AtEventDrawTaskNew::DrawHitPoints()
{
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   auto *event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   if (fIsRawData) {
      fRawevent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
      char name[20];
      sprintf(name, "fRawEvent_DT%i", fTaskNumber);
      fRawevent->SetName(name);
      gROOT->GetListOfSpecials()->Add(fRawevent);
   }

   Int_t eventID = event->GetEventID();
   TString TSevt = " Event ID : ";
   TString TSpad = " Pad ID : ";
   dumpEvent << TSevt << eventID << std::endl;

   //////////////////////////////////////////////

   Int_t nHits = event->GetNumHits();
   fHitSet = new TEvePointSet("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   fHitSet->SetOwnIds(kTRUE);
   fHitSet->SetMarkerColor(fHitColor);
   fHitSet->SetMarkerSize(fHitSize);
   fHitSet->SetMarkerStyle(fHitStyle);
   std::cout << cBLUE << " Number of hits : " << nHits << cNORMAL << std::endl;

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *event->GetHits().at(iHit);
      Int_t PadNumHit = hit.GetPadNum();
      Int_t PadMultHit = event->GetHitPadMult(PadNumHit);

      if (hit.GetCharge() < fThreshold)
         continue;
      if (PadMultHit > fMultiHit)
         continue;
      auto position = hit.GetPosition();

      fHitSet->SetMarkerColor(fHitColor);
      fHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      fHitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
      fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());
   }

   // Adding raw data points
   gEve->AddElement(fHitSet);

   dumpEvent.close();
}

void AtEventDrawTaskNew::Reset()
{
   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, fEventManager);
   }

   if (fPadPlane != nullptr)
      fPadPlane->Reset(nullptr);
}

void AtEventDrawTaskNew::DrawPadPlane()
{
   if (fPadPlane) {
      fPadPlane->Reset(nullptr);
      return;
   }

   fDetmap->GeneratePadPlane();
   fPadPlane = fDetmap->GetPadPlane();
   fCvsPadPlane->cd();
   // fPadPlane -> Draw("COLZ L0"); //0  == bin lines adre not drawn
   fPadPlane->Draw("COL L0");
   fPadPlane->SetMinimum(1.0);
   gStyle->SetOptStat(0);
   gStyle->SetPalette(103);
   gPad->Update();
}

void AtEventDrawTaskNew::DrawPadWave()
{
   char name[20];
   sprintf(name, "fPadWave_DT%i", fTaskNumber);
   fPadWave = new TH1I(name, name, 512, 0, 511);
   gROOT->GetListOfSpecials()->Add(fPadWave);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

void AtEventDrawTaskNew::UpdateCvsPadPlane()
{
   fCvsPadPlane->Modified();
   fCvsPadPlane->Update();

   /*if (paxis) {
    if(fIs2DPlotRange) {
    paxis -> SetX1NDC(0.940);
    paxis -> SetX2NDC(0.955);
    paxis -> SetLabelSize(0.08);
    paxis -> GetAxis() -> SetTickSize(0.01);
    } else {
    paxis -> SetX1NDC(0.905);
    paxis -> SetX2NDC(0.94);
    }

    fCvsPadPlane -> Modified();
    fCvsPadPlane -> Update();
    }*/
}

void AtEventDrawTaskNew::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();

   //  TPaletteAxis *paxis
   //  = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");
}

void AtEventDrawTaskNew::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

void AtEventDrawTaskNew::SelectPad(const char *rawevt)
{

   try {
      int event = gPad->GetEvent();
      if (event != 11)
         return; // may be comment this line
      TObject *select = gPad->GetSelected();
      if (!select)
         return;
      if (select->InheritsFrom(TH2Poly::Class())) {
         auto *h = dynamic_cast<TH2Poly *>(select);
         gPad->GetCanvas()->FeedbackMode(kTRUE);
         AtRawEvent *tRawEvent = nullptr;
         tRawEvent = dynamic_cast<AtRawEvent *>(gROOT->GetListOfSpecials()->FindObject(rawevt));
         if (tRawEvent == nullptr) {
            std::cout
               << " = AtEventDrawTaskNew::SelectPad NULL pointer for the AtRawEvent! Please select an event first "
               << std::endl;
            return;
         }

         int pyold = gPad->GetUniqueID();
         int px = gPad->GetEventX();
         int py = gPad->GetEventY();
         float uxmin = gPad->GetUxmin();
         float uxmax = gPad->GetUxmax();
         int pxmin = gPad->XtoAbsPixel(uxmin);
         int pxmax = gPad->XtoAbsPixel(uxmax);
         if (pyold)
            TVirtualX::Instance()->DrawLine(pxmin, pyold, pxmax, pyold);
         TVirtualX::Instance()->DrawLine(pxmin, py, pxmax, py);
         gPad->SetUniqueID(py);
         Float_t upx = gPad->AbsPixeltoX(px);
         Float_t upy = gPad->AbsPixeltoY(py);
         Double_t x = gPad->PadtoX(upx);
         Double_t y = gPad->PadtoY(upy);
         Int_t bin = h->FindBin(x, y);
         const char *bin_name = h->GetBinName(bin);
         // std::cout<<" X : "<<x<<"  Y: "<<y<<std::endl;
         // std::cout<<bin_name<<std::endl;
         std::cout << " ==========================" << std::endl;
         std::cout << " Bin number selected : " << bin << " Bin name :" << bin_name << std::endl;

         AtMap *tmap = nullptr;
         tmap = dynamic_cast<AtMap *>(gROOT->GetListOfSpecials()->FindObject("fMap"));
         Int_t tPadNum = tmap->BinToPad(bin);

         std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
         std::cout << " Electronic mapping: " << tmap->GetPadRef(tPadNum) << std::endl;
         AtPad *tPad = tRawEvent->GetPad(tPadNum);

         if (tPad == nullptr)
            return;

         std::cout << " Event ID (Select Pad) : " << tRawEvent->GetEventID() << std::endl;
         std::cout << " Raw Event Pad Num " << tPad->GetPadNum() << std::endl;
         std::cout << std::endl;

         TH1I *tPadWave = nullptr;
         tPadWave = dynamic_cast<TH1I *>(gROOT->GetListOfSpecials()->FindObject("fPadWave"));
         auto rawadc = tPad->GetRawADC();
         auto adc = tPad->GetADC();
         if (tPadWave == nullptr) {
            std::cout << " = AtEventDrawTaskNew::SelectPad NULL pointer for the TH1I! Please enable SetPersistance for "
                         "Unpacking task or select an event first "
                      << std::endl;
            return;
         }
         tPadWave->Reset();
         // tPadWaveSub->Reset();
         for (Int_t i = 0; i < 512; i++) {

            // tPadWave->SetBinContent(i,rawadc[i]);
            tPadWave->SetBinContent(i, adc[i]);
            // tPadWaveSub->SetBinContent(i,adc[i]);
         }

         TCanvas *tCvsPadWave = nullptr;
         tCvsPadWave = dynamic_cast<TCanvas *>(gROOT->GetListOfSpecials()->FindObject("fCvsPadWave"));
         if (tCvsPadWave == nullptr) {
            std::cout << " = AtEventDrawTaskNew::SelectPad NULL pointer for the TCanvas! Please select an event first "
                      << std::endl;
            return;
         }
         tCvsPadWave->cd();
         tPadWave->Draw();
         // tPadWaveSub->Draw("SAME");
         tCvsPadWave->Update();
      }

   } catch (const std::exception &e) {

      std::cout << cRED << " Exception caught in Select Pad " << e.what() << cNORMAL << "\n";
   }
}

void AtEventDrawTaskNew::DrawPad(Int_t taskNum, Int_t PadNum)
{
   char name[20];
   sprintf(name, "fRawEvent_DT%i", taskNum);
   std::cout << "checking fRawevent" << std::endl;
   AtRawEvent *tRawEvent = dynamic_cast<AtRawEvent *>(gROOT->GetListOfSpecials()->FindObject(name));
   if (tRawEvent == nullptr) {
      std::cout << "tRawEvent is NULL!" << std::endl;
   } else {
      std::cout << "tRawEvent is not nullptr" << std::endl;
      AtPad *tPad = tRawEvent->GetPad(PadNum);
      if (tPad == nullptr)
         return;
      auto rawadc = tPad->GetRawADC();
      auto adc = tPad->GetADC();
      sprintf(name, "fPadWave_DT%i", taskNum);
      TH1I *tPadWave = nullptr;
      tPadWave = dynamic_cast<TH1I *>(gROOT->GetListOfSpecials()->FindObject(name));
      if (tPadWave == nullptr) {
         std::cout << " = AtEventDrawTaskNew::SelectPad NULL pointer for the TH1I! Please enable SetPersistance for "
                      "Unpacking task or select an event first "
                   << std::endl;
         return;
      }
      tPadWave->Reset();
      for (Int_t i = 0; i < 512; i++) {
         tPadWave->SetBinContent(i, adc[i]);
      }

      sprintf(name, "fCvsPadWave_DT%i", taskNum);
      TCanvas *tCvsPadWave = nullptr;
      tCvsPadWave = dynamic_cast<TCanvas *>(gROOT->GetListOfSpecials()->FindObject(name));
      if (tCvsPadWave == nullptr) {
         std::cout << " = AtEventDrawTaskNew::SelectPad NULL pointer for the TCanvas! Please select an event first "
                   << std::endl;
         return;
      }
      tCvsPadWave->cd();
      tPadWave->Draw();
      tCvsPadWave->Update();
   }
}

void AtEventDrawTaskNew::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

EColor AtEventDrawTaskNew::GetTrackColor(int i)
{
   std::vector<EColor> colors = {kAzure, kOrange, kViolet, kTeal, kMagenta, kBlue, kViolet, kYellow, kCyan, kAzure};
   if (i < 10) {
      return colors.at(i);
   } else
      return kAzure;
}

void AtEventDrawTaskNew::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtEventDrawTaskNew::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}
