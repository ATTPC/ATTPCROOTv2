/**
 * @brief Event display task
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */

#include "AtEventDrawTaskNew.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtAuxPad.h"       // for AtAuxPad
#include "AtEvent.h"        // for AtEvent, hitVector
#include "AtEventManagerNew.h" // for AtEventManager
#include "AtFindVertex.h"   //for vertex
#include "AtHit.h"          // for AtHit
#include "AtHitCluster.h"   // for AtHitCluster
#include "AtMap.h"          // for AtMap
#include "AtPad.h"          // for AtPad
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
   : fIs2DPlotRange(kFALSE), fUnpackHough(kFALSE), fEventArray(nullptr), fEventManager(nullptr), fRawevent(nullptr),
     fDetmap(nullptr), fThreshold(0), fHitSet(nullptr), fCorrectedHitSet(nullptr), fhitBoxSet(nullptr),
     fPadPlanePal(nullptr), fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium), fCvsPadPlane(nullptr),
     fPadPlane(nullptr), fCvsPadWave(nullptr), fPadWave(nullptr),
     fAtMapPtr(nullptr), fMinZ(0), fMaxZ(1344), fMinX(432), fMaxX(-432), f3DHitStyle(0), fMultiHit(0),
     fSaveTextData(false), f3DThreshold(0), fRawEventBranchName("AtRawEvent"), fEventBranchName("AtEventH"),
     fPatternEventBranchName("AtPatternEvent"), fVertexSize(1.5), fVertexStyle(kFullCircle), fCvsPID(nullptr),
     fPID(nullptr), fCvsPID2(nullptr), fPID2(nullptr)
{

   fIsRawData = kFALSE;

   fIniHit = new AtHit();
   fIniHitRansac = new AtHit();
   fTrackNum = 0;

   fMinTracksPerVertex = 1;

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

   fCorrectedEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fCorrectedEventBranchName));
   if (fCorrectedEventArray)
      LOG(INFO) << cGREEN << "Corrected Event Array Found in branch " << fCorrectedEventBranchName << "." << cNORMAL
                << std::endl;

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray) {
      LOG(INFO) << cGREEN << "Raw Event Array Found in branch " << fRawEventBranchName << "." << cNORMAL << std::endl;
      fIsRawData = kTRUE;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fPatternEventBranchName));
   if (fPatternEventArray)
      LOG(INFO) << cGREEN << "Pattern Event Array Found in branch " << fPatternEventBranchName << "." << cNORMAL
                << std::endl;


   gStyle->SetPalette(55);

   fCvsPadWave = fEventManager->GetCvsPadWave();
   fCvsPadWave->SetName("fCvsPadWave");
   gROOT->GetListOfSpecials()->Add(fCvsPadWave);
   DrawPadWave();
   fCvsPadPlane = fEventManager->GetCvsPadPlane(); // There is a problem if the pad plane is drawn first
   fCvsPadPlane->ToggleEventStatus();
   fCvsPadPlane->AddExec("ex", "AtEventDrawTaskNew::SelectPad(\"fRawEvent\")");
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

void AtEventDrawTaskNew::DrawRecoHits()
{
   if (!fPatternEventArray)
      return;

   auto *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
   if (!patternEvent) {
      LOG(info) << "No pattern event";
      return;
   }

   auto TrackCand = patternEvent->GetTrackCand();
   fHitLine.clear();
   fVertex.clear();

   if (fDrawVertexFromLines) {
      AtFindVertex findVtx(12);
      // findVtx.FindVertexMultipleLines(TrackCand, 2);
      findVtx.FindVertex(TrackCand, fMinTracksPerVertex);
      std::vector<tracksFromVertex> tv;
      tv = findVtx.GetTracksVertex();

      for (size_t i = 0; i < tv.size(); i++) {
         fVertex.push_back(new TEvePointSet(Form("Vertex_%zu", i), 0, TEvePointSelectorConsumer::kTVT_XYZ));
         fVertex[i]->SetMarkerColor(kViolet);
         fVertex[i]->SetMarkerSize(fVertexSize);
         fVertex[i]->SetMarkerStyle(fVertexStyle);
         fVertex[i]->SetNextPoint(tv.at(i).vertex.X() / 10., tv.at(i).vertex.Y() / 10., tv.at(i).vertex.Z() / 10.);
      }
   }
   for (Int_t i = 0; i < TrackCand.size(); i++) {

      Color_t trackColor = GetTrackColor(i) + 1;

      // Get the line to draw
      AtTrack track = TrackCand.at(i);

      if (track.GetPattern() != nullptr && track.GetPattern()->GetEveElement() != nullptr) {
         fHitLine.push_back(track.GetPattern()->GetEveElement());
         fHitLine.back()->SetMainColor(trackColor);
         fHitLine.back()->SetElementName(Form("line_%i", (int)fHitLine.size() - 1));
      }

      std::vector<AtHit> trackHits = track.GetHitArray();

      /*      fHitSetTFHC.push_back(
               std::make_unique<TEvePointSet>(Form("HitMC_%d", i), 0, TEvePointSelectorConsumer::kTVT_XYZ));
      */
      fHitSetTFHC.push_back(new TEvePointSet(Form("HitMC_%d", i), 0, TEvePointSelectorConsumer::kTVT_XYZ));
      fHitSetTFHC[i]->SetMarkerColor(trackColor);
      fHitSetTFHC[i]->SetMarkerSize(fHitSize);
      fHitSetTFHC[i]->SetMarkerStyle(fHitStyle);

      for (auto &trackHit : trackHits) {
         auto position = trackHit.GetPosition();
         fHitSetTFHC[i]->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.);
      }

      std::vector<AtHitCluster> *hitClusters = track.GetHitClusterArray();
      // fHitClusterSet.push_back(std::make_unique<TEveBoxSet>(Form("HitCluster_%d", i)));
      fHitClusterSet.push_back(new TEveBoxSet(Form("HitCluster_%d", i)));
      fHitClusterSet[i]->Reset(TEveBoxSet::kBT_AABox, kFALSE, 64);
      // fHitClusterSet[i]->SetPalette(fRGBAPalette);
      // fHitClusterSet[i]->DigitValue(2000);

      if (hitClusters->size() > 0) {

         for (auto hitCluster : *hitClusters) {
            auto clusPos = hitCluster.GetPosition();
            double boxSize = 0.6;

            fHitClusterSet[i]->AddBox(clusPos.X() / 10. - boxSize / 2.0, clusPos.Y() / 10. - boxSize / 2.0,
                                      clusPos.Z() / 10. - boxSize / 2.0, boxSize, boxSize, boxSize);
         }
      }

      fHitClusterSet[i]->UseSingleColor();
      fHitClusterSet[i]->SetMainColor(trackColor);
      fHitClusterSet[i]->SetMainTransparency(50);

      fHitClusterSet[i]->RefitPlex();
      TEveTrans &tHitClusterBoxPos = fHitClusterSet[i]->RefMainTrans();
      tHitClusterBoxPos.SetPos(0.0, 0.0, 0.0);
   } // End loop over tracks

   // Add noise hits
   // fHitSetTFHC.push_back(std::make_unique<TEvePointSet>("HitsNoise", 0, TEvePointSelectorConsumer::kTVT_XYZ));
   fHitSetTFHC.push_back(new TEvePointSet("HitsNoise", 0, TEvePointSelectorConsumer::kTVT_XYZ));
   fHitSetTFHC.back()->SetMainColor(kRed);
   fHitSetTFHC.back()->SetMarkerSize(fHitSize);
   fHitSetTFHC.back()->SetMarkerStyle(fHitStyle);

   for (auto &hit : patternEvent->GetNoiseHits()) {
      auto position = hit.GetPosition();
      fHitSetTFHC.back()->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.);
   }

   std::cout << cRED << " Found " << TrackCand.size() << " track candidates " << cNORMAL << std::endl;

   for (auto &elem : fHitSetTFHC)
      gEve->AddElement(elem);
   for (auto &elem : fHitClusterSet)
      gEve->AddElement(elem);
   for (auto &elem : fHitLine)
      gEve->AddElement(elem);
   for (auto &elem : fVertex)
      gEve->AddElement(elem);
}

void AtEventDrawTaskNew::DrawHitPoints()
{
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   auto *event = dynamic_cast<AtEvent *>(fEventArray->At(0));

   if (fIsRawData) {
      fRawevent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
      fRawevent->SetName("fRawEvent");
      gROOT->GetListOfSpecials()->Add(fRawevent);
   }

   Double_t Qevent = event->GetEventCharge();
   Double_t RhoVariance = event->GetRhoVariance();
   auto MeshArray = event->GetMesh();
   Int_t eventID = event->GetEventID();
   TString TSevt = " Event ID : ";
   TString TSpad = " Pad ID : ";
   dumpEvent << TSevt << eventID << std::endl;

   if (fEventManager->GetDrawReconstruction())
      DrawRecoHits();

   //////////////////////////////////////////////

   fhitBoxSet = new TEveBoxSet("hitBox");
   fhitBoxSet->Reset(TEveBoxSet::kBT_AABox, kTRUE, 64);

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

      if (!fEventManager->GetToggleCorrData()) {
         fHitSet->SetMarkerColor(fHitColor);
         fHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
         fHitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
         fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());
      }

      if (fIsRawData) {
         AtPad *RawPad = fRawevent->GetPad(PadNumHit);
         if (RawPad != nullptr) {
            auto adc = RawPad->GetADC();
            for (Int_t i = 0; i < 512; i++) {

               f3DThreshold = fEventManager->Get3DThreshold();
            }
         }
      }

      if (fSaveTextData) {
         if (!fEventManager->GetToggleCorrData())
            dumpEvent << position.X() << " " << position.Y() << " " << position.Z() << " " << hit.GetTimeStamp() << " "
                      << hit.GetCharge() << std::endl;
      }
   }

   if (fCorrectedEventArray != nullptr) {
      std::cout << "Adding corrected hits" << std::endl;
      auto *eventCorr = dynamic_cast<AtEvent *>(fCorrectedEventArray->At(0));
      fCorrectedHitSet = new TEvePointSet("Hit2", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
      fCorrectedHitSet->SetOwnIds(kTRUE);
      fCorrectedHitSet->SetMarkerColor(kBlue);
      fCorrectedHitSet->SetMarkerSize(fHitSize);
      fCorrectedHitSet->SetMarkerStyle(fHitStyle);

      for (Int_t iHit = 0; iHit < nHits; iHit++) {
         if (eventCorr == nullptr) {
            std::cout << "Corrected event was empty!" << std::endl;
            break;
         }
         AtHit hit = *eventCorr->GetHits().at(iHit);
         auto position = hit.GetPosition();
         if (hit.GetCharge() < fThreshold)
            continue;
         fCorrectedHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
         fCorrectedHitSet->SetPointId(new TNamed(Form("HitCorr %d", iHit), ""));
      }
   }

   //////////////////////// Colored Box Drawing ////////////////

   fPadPlane->Draw("zcol");
   gPad->Update();
   fPadPlanePal = dynamic_cast<TPaletteAxis *>(fPadPlane->GetListOfFunctions()->FindObject("palette"));

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *event->GetHits().at(iHit);
      auto position = hit.GetPosition();

      if (f3DHitStyle == 0) {

         Float_t HitBoxYDim = hit.GetCharge() * 0.001;
         Float_t HitBoxZDim = 0.05;
         Float_t HitBoxXDim = 0.05;

         if (!fEventManager->GetToggleCorrData()) {
            fhitBoxSet->AddBox(position.X() / 10. - HitBoxXDim / 2.0, position.Y() / 10.,
                               position.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
                               HitBoxZDim); // This coordinates are x,y,z in our system
         }

      } else if (f3DHitStyle == 1) {

         Float_t HitBoxYDim = hit.GetCharge() * 0.0002;
         Float_t HitBoxZDim = hit.GetCharge() * 0.0002;
         Float_t HitBoxXDim = hit.GetCharge() * 0.0002;

         if (!fEventManager->GetToggleCorrData()) {
            fhitBoxSet->AddBox(position.X() / 10. - HitBoxXDim / 2.0, position.Y() / 10. - HitBoxYDim / 2.0,
                               position.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
                               HitBoxZDim); // This coordinates are x,y,z in our system
         }
      }

      Float_t xrgb = 255, yrgb = 0, zrgb = 0;
      if (fPadPlanePal) {

         Int_t cHit = fPadPlanePal->GetValueColor(hit.GetCharge());
         TColor *hitBoxColor = gROOT->GetColor(cHit);
         hitBoxColor->GetRGB(xrgb, yrgb, zrgb);

         // std::cout<<" xrgb : "<<xrgb<<std::endl;
         // std::cout<<" yrgb : "<<yrgb<<std::endl;
         // std::cout<<" zrgb : "<<zrgb<<std::endl;
         // std::cout<<fPadPlanePal<<std::endl;
      }

      fhitBoxSet->DigitColor(xrgb * 255, yrgb * 255, zrgb * 255, 0);
   }

   /////////////////////// End of colored box drawing ////////////////////////////

   fhitBoxSet->RefitPlex();
   TEveTrans &tHitBoxPos = fhitBoxSet->RefMainTrans();
   tHitBoxPos.SetPos(0.0, 0.0, 0.0);

   // for(Int_t i=0;i<hitSphereArray.size();i++) gEve->AddElement(hitSphereArray[i]);

   if (fIsRawData) {
      Int_t nPads = fRawevent->GetNumPads();
      std::cout << "Num of pads : " << nPads << std::endl;

      if (fEventManager->GetDrawAllPad()) {

         std::cout << "Setting pads for Draw All Pads." << std::endl;

         for (Int_t iPad = 0; iPad < nPads; iPad++) {

            AtPad *fPad = fRawevent->GetPad(iPad);
            // std::cout<<"Pad num : "<<iPad<<" Is Valid? : "<<fPad->GetValidPad()
            //	     << std::endl;

            if (!fPad->GetValidPad())
               continue;

            auto rawadc = fPad->GetRawADC();
            auto adc = fPad->GetADC();
            // dumpEvent<<TSpad<<fPad->GetPadNum()<<std::endl;

         }
      }
   }

   // Adding raw data points
   if (!fEventManager->GetDrawReconstruction()) {

      if (fCorrectedHitSet)
         gEve->AddElement(fCorrectedHitSet);
      gEve->AddElement(fHitSet);
      gEve->AddElement(fhitBoxSet);
   }

   dumpEvent.close();
}


void AtEventDrawTaskNew::Reset()
{
   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, fEventManager);
   }

   if (fCorrectedHitSet) {
      fCorrectedHitSet->Reset();
      gEve->RemoveElement(fCorrectedHitSet, fEventManager);
   }

   if (fhitBoxSet) {
      fhitBoxSet->Reset();
      gEve->RemoveElement(fhitBoxSet, fEventManager);
   }

   if (fEventManager->GetDrawReconstruction()) {

      if (fPatternEventArray) {
         for (auto elem : fHitClusterSet)
            gEve->RemoveElement(elem, fEventManager);
         for (auto elem : fHitSetTFHC)
            gEve->RemoveElement(elem, fEventManager);
         for (auto elem : fHitLine)
            gEve->RemoveElement(elem, fEventManager);
         for (auto elem : fVertex)
            gEve->RemoveElement(elem, fEventManager);
         fHitClusterSet.clear();
         fHitSetTFHC.clear();
         fHitLine.clear();
         fVertex.clear();
      }

   } // Draw Minimization

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
   fPadWave = new TH1I("fPadWave", "fPadWave", 512, 0, 511);
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

/*void
 AtEventDrawTaskNew::SetHitClusterAttributes(Color_t color, Size_t size, Style_t style)
 {
 fHitClusterColor = color;
 fHitClusterSize = size;
 fHitClusterStyle = style;
 }*/

/*void
 AtEventDrawTaskNew::SetRiemannAttributes(Color_t color, Size_t size, Style_t style)
 {
 fRiemannColor = color;
 fRiemannSize = size;
 fRiemannStyle = style;
 }*/

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
            std::cout << " = AtEventDrawTaskNew::SelectPad NULL pointer for the AtRawEvent! Please select an event first "
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

void AtEventDrawTaskNew::DrawWave(Int_t PadNum)
{

   // Bool_t IsValid=kFALSE;
   // AtPad *pad = fRawevent->GetPad(0);
   // AtPad *pad= fRawevent->GetPad(PadNum,IsValid);
   // std::cout<<" Raw Event Pad Num "<<pad->GetPadNum()<<" Is Valid? : "<<IsValidPad<<std::endl;
}

void AtEventDrawTaskNew::Set3DHitStyleBar()
{
   f3DHitStyle = 0;
}

void AtEventDrawTaskNew::Set3DHitStyleBox()
{
   f3DHitStyle = 1;
}

void AtEventDrawTaskNew::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

void AtEventDrawTaskNew::SetSaveTextData()
{
   fSaveTextData = kTRUE;
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
