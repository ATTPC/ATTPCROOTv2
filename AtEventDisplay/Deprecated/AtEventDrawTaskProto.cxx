#include "AtEventDrawTaskProto.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtAuxPad.h"            // for AtAuxPad
#include "AtBaseEvent.h"         // for AtBaseEvent::AuxPadMap
#include "AtEvent.h"             // for AtEvent, hitVector
#include "AtEventManagerProto.h" // for AtEventManagerProto
#include "AtHit.h"               // for AtHit
#include "AtMap.h"               // for AtMap
#include "AtPad.h"               // for AtPad
#include "AtPatternEvent.h"      // for AtPatternEvent
#include "AtProtoEvent.h"        // for AtProtoEvent
#include "AtProtoEventAna.h"     // for AtProtoEventAna
#include "AtProtoQuadrant.h"     // for AtProtoQuadrant
#include "AtRawEvent.h"          // for AtRawEvent, AuxPadMap
#include "AtTpcProtoMap.h"       // for AtTpcProtoMap
#include "AtTrack.h"             // for AtTrack

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager
#include <FairTask.h>        // for InitStatus, kSUCCESS

#include <Math/Point3D.h>   // for PositionVector3D
#include <TAttMarker.h>     // for kFullDotMedium
#include <TAxis.h>          // for TAxis
#include <TCanvas.h>        // for TCanvas
#include <TClonesArray.h>   // for TClonesArray
#include <TColor.h>         // for TColor
#include <TEveBoxSet.h>     // for TEveBoxSet, TEveBoxSet::kBT_AABox
#include <TEveLine.h>       // for TEveLine
#include <TEveManager.h>    // for TEveManager, gEve
#include <TEvePointSet.h>   // for TEvePointSet
#include <TEveTrans.h>      // for TEveTrans
#include <TEveTreeTools.h>  // for TEvePointSelectorConsumer, TEvePoin...
#include <TF1.h>            // for TF1
#include <TGraph.h>         // for TGraph
#include <TH1.h>            // for TH1I, TH1D, TH1F
#include <TH2.h>            // for TH2F
#include <TH2Poly.h>        // for TH2Poly
#include <TList.h>          // for TList
#include <TMath.h>          // for Power, Sqrt
#include <TNamed.h>         // for TNamed
#include <TObject.h>        // for TObject
#include <TPad.h>           // for TPad
#include <TPaletteAxis.h>   // for TPaletteAxis
#include <TROOT.h>          // for TROOT, gROOT
#include <TSeqCollection.h> // for TSeqCollection
#include <TStyle.h>         // for TStyle, gStyle
#include <TVirtualPad.h>    // for TVirtualPad, gPad
#include <TVirtualX.h>      // for TVirtualX, gVirtualX

#include <algorithm> // for max
#include <array>     // for array
#include <cstdio>    // for sprintf
#include <exception> // for exception
#include <iostream>  // for operator<<, basic_ostream, endl
#include <map>       // for operator!=, _Rb_tree_const_iterator
#include <memory>    // for allocator, allocator_traits<>::valu...
#include <string>    // for char_traits, operator<<
#include <utility>   // for pair
#include <vector>    // for vector

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using namespace std;

ClassImp(AtEventDrawTaskProto);

AtEventDrawTaskProto::AtEventDrawTaskProto()
   : fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium), fHitSet(nullptr), fSaveTextData(false),
     fhitBoxSet(nullptr)
{
   Char_t padhistname[256];

   for (Int_t i = 0; i < 2015; i++) {
      sprintf(padhistname, "pad_%d", i);
      fPadAll[i] = new TH1I(padhistname, padhistname, 512, 0, 511);
   }

   Char_t phihistname[256];

   for (Int_t i = 0; i < 5; i++) {
      sprintf(phihistname, "PhiDistr_%d", i);
      fPhiDistr[i] = new TH1D(phihistname, phihistname, 180.0, -180.0, 180.0);
      if (i == 0)
         fPhiDistr[i]->SetLineColor(kRed);
      else if (i == 1)
         fPhiDistr[i]->SetLineColor(kBlue);
      else if (i == 2)
         fPhiDistr[i]->SetLineColor(kGreen);
      else if (i == 3)
         fPhiDistr[i]->SetLineColor(kCyan);
      else if (i == 4)
         fPhiDistr[i]->SetLineColor(kMagenta);
      fPhiDistr[i]->SetLineWidth(2);
      fPhiDistr[i]->GetYaxis()->SetRangeUser(0., 20.);
   }

   fIsCircularHough = kFALSE;
   fIsLinearHough = kTRUE;
   fIsRawData = kFALSE;
   kIsPRDrawn = kFALSE;
   fHoughLinearFit =
      new TF1("HoughLinearFit", " (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])", 0, 500);

   for (Int_t i = 0; i < 4; i++) {
      fHoughFit[i] =
         new TF1(Form("HoughFit%i", i), " (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])", 0, 120);
      fHoughFit[i]->SetLineColor(kRed);
      fFit[i] = new TF1(Form("HoughFit%i", i), "pol1", 0, 120);
      fFit[i]->SetLineColor(kBlue);
   }

   f3DHitStyle = 1;
}

AtEventDrawTaskProto::~AtEventDrawTaskProto()
{

   delete fHoughLinearFit;
}

InitStatus AtEventDrawTaskProto::Init()
{

   std::cout << " =====  AtEventDrawTaskProto::Init =====" << std::endl;

   gROOT->Reset();
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerProto::Instance();
   fDetmap = new AtTpcProtoMap();
   dynamic_cast<AtTpcProtoMap *>(fDetmap)->SetProtoMap(fMap.Data());
   dynamic_cast<AtTpcProtoMap *>(fDetmap)->SetGeoFile("proto20181201_geo_hires.root");
   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap);

   fHitArray = dynamic_cast<TClonesArray *>(
      ioMan->GetObject("AtEventH")); // TODO: Why this confusing name? It should be fEventArray
   if (fHitArray)
      LOG(info) << cGREEN << "Hit Array Found." << cNORMAL;

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtRawEvent"));
   if (fRawEventArray) {
      LOG(info) << cGREEN << "Raw Event Array  Found." << cNORMAL;
      fIsRawData = kTRUE;
   }

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtPatternEvent"));
   if (fPatternEventArray)
      LOG(info) << cGREEN << "Pattern Event Array Found." << cNORMAL;

   // fHoughSpaceArray =  (TClonesArray*) ioMan->GetObject("AtHough");
   // if(fHoughSpaceArray) LOG(info)<<cGREEN<<"Hough Array Found."<<cNORMAL<<FairLogger::endl;

   // fProtoEventArray =  (TClonesArray*) ioMan->GetObject("AtProtoEvent");
   // if(fProtoEventArray) LOG(info)<<cGREEN<<"Prototype Event Array Found."<<cNORMAL<<FairLogger::endl;

   fProtoEventAnaArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtProtoEventAna"));
   if (fProtoEventAnaArray)
      LOG(info) << cGREEN << "Prototype Event Analysis Array Found." << cNORMAL;

   // Drawing histograms

   gStyle->SetPalette(55);
   fCvsPadWave = fEventManager->GetCvsPadWave();
   fCvsPadWave->SetName("fCvsPadWave");
   gROOT->GetListOfSpecials()->Add(fCvsPadWave);
   DrawPadWave();
   fCvsPadPlane = fEventManager->GetCvsPadPlane(); // There is a problem if the pad plane is drawn first
   fCvsPadPlane->ToggleEventStatus();
   fCvsPadPlane->AddExec("ex", "AtEventDrawTaskProto::SelectPad(\"fRawEvent\")");
   DrawPadPlane();
   fCvsPadAll = fEventManager->GetCvsPadAll();
   DrawPadAll();
   fCvsMesh = fEventManager->GetCvsMesh();
   DrawMesh();

   /*fCvsQuadrant1 = fEventManager->GetCvsQuadrant1();
   fCvsQuadrant2 = fEventManager->GetCvsQuadrant2();
   fCvsQuadrant3 = fEventManager->GetCvsQuadrant3();
   fCvsQuadrant4 = fEventManager->GetCvsQuadrant4();
   DrawProtoSpace();
   fCvsELQuadrant1 = fEventManager->GetCvsELQuadrant1();
   fCvsELQuadrant2 = fEventManager->GetCvsELQuadrant2();
   fCvsELQuadrant3 = fEventManager->GetCvsELQuadrant3();
   fCvsELQuadrant4 = fEventManager->GetCvsELQuadrant4();
   DrawProtoEL();
   //if(fProtoEventAnaArray) DrawProtoELAna();
   fCvsVertex =  fEventManager->GetCvsVertex();
   fCvsKineAA =  fEventManager->GetCvsKineAA();
   DrawProtoVertex();
   DrawProtoKine();*/
   fCvsAux = fEventManager->GetCvsAux();
   DrawProtoAux();

   std::cout << " AtEventDrawTaskProto::Init -  End of initialization " << std::endl;
   return kSUCCESS;
}

void AtEventDrawTaskProto::Exec(Option_t *option)
{
   Reset();

   if (fHitArray)
      DrawHitPoints();
   // if(fProtoEventArray)    DrawProtoPattern();
   // if(fHoughSpaceArray)    DrawProtoHough();
   // if(fProtoEventAnaArray) DrawProtoPatternAna();

   gEve->Redraw3D(kFALSE);

   UpdateCvsPadWave();
   UpdateCvsPadPlane();
   UpdateCvsPadAll();
   UpdateCvsMesh();
   // UpdateCvsProtoQ();
   // UpdateCvsProtoEL();
   // UpdateCvsProtoVertex();
   // UpdateCvsProtoKine();
   UpdateCvsProtoAux();
}

void AtEventDrawTaskProto::Reset()
{

   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, fEventManager);
   }

   if (fhitBoxSet) {
      fhitBoxSet->Reset();
      gEve->RemoveElement(fhitBoxSet, fEventManager);
   }

   if (fEventManager->GetDrawPatternRecognition()) {

      if (fPatternEventArray != nullptr & kIsPRDrawn == kTRUE) {

         if (fLineNum > 0) {
            for (Int_t i = 0; i < fLineNum; i++) {
               if (fHitSetPR[i] != nullptr) {
                  gEve->RemoveElement(fHitSetPR[i], fEventManager);
               }
            }
         }
      }

      kIsPRDrawn = kFALSE;
   }

   if (fPadPlane != nullptr)
      fPadPlane->Reset(nullptr);
}

// Filling functions

void AtEventDrawTaskProto::DrawHitPoints()
{

   // Float_t *MeshArray;
   fMesh->Reset(nullptr);
   for (auto &fAuxChannel : fAuxChannels)
      fAuxChannel->Reset(nullptr);
   // f3DHist->Reset(0);
   // TRandom r(0);

   for (auto &i : fPadAll)
      i->Reset(nullptr);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   std::vector<Double_t> fPosXMin;
   std::vector<Double_t> fPosYMin;
   std::vector<Double_t> fPosZMin;

   // fQEventHist_H->Reset(0);
   auto *event = dynamic_cast<AtEvent *>(fHitArray->At(0)); // TODO: Why this confusing name? It should be fEventArray
   Int_t nHits = 0;

   if (event != nullptr) {
      auto MeshArray = event->GetMesh();
      Int_t eventID = event->GetEventID();
      nHits = event->GetNumHits();
      TString TSevt = " Event ID : ";
      TString TSpad = " Pad ID : ";
      dumpEvent << TSevt << eventID << std::endl;

      /* if(fEventManager->GetEraseQEvent()){
        fQEventHist->Reset();
        fRhoVariance->Reset();
       }

       fQEventHist->Fill(Qevent);
       fQEventHist_H->Fill(Qevent);
       fRhoVariance->Fill(RhoVariance);*/

      for (Int_t i = 0; i < 512; i++) {

         fMesh->SetBinContent(i, MeshArray[i]);
      }
   } // if(event=!NULL)

   fHitSet = new TEvePointSet("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   fHitSet->SetOwnIds(kTRUE);
   fHitSet->SetMarkerColor(fHitColor);
   fHitSet->SetMarkerSize(fHitSize);
   fHitSet->SetMarkerStyle(fHitStyle);
   std::cout << cYELLOW << " Number of hits : " << nHits << cNORMAL << std::endl;

   //////////////////////////////////////////////

   fhitBoxSet = new TEveBoxSet("hitBox");
   fhitBoxSet->Reset(TEveBoxSet::kBT_AABox, kTRUE, 64);

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *event->GetHits().at(iHit);

      // if(hit.GetCharge()<fThreshold) continue;
      // if(PadMultHit>fMultiHit) continue;
      auto position = hit.GetPosition();

      fHitSet->SetMarkerColor(fHitColor);
      fHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
      fHitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
      fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());

      Bool_t fValidPad;

      // if(fSaveTextData)
      // dumpEvent<<position.X()<<" "<<position.Y()<<" "<<position.Z()<<" "<<hit.GetTimeStamp()<<"
      // "<<hit.GetCharge()<<std::endl;
   }
   //////////////////////// Colored Box Drawing ////////////////

   // fPadPlane -> Draw("zcol");
   gPad->Update();
   fPadPlanePal = dynamic_cast<TPaletteAxis *>(fPadPlane->GetListOfFunctions()->FindObject("palette"));

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = *event->GetHits().at(iHit);
      auto position = hit.GetPosition();

      if (f3DHitStyle == 0) {

         Float_t HitBoxYDim = hit.GetCharge() * 0.001;
         Float_t HitBoxZDim = 0.05;
         Float_t HitBoxXDim = 0.05;

         fhitBoxSet->AddBox(position.X() / 10. - HitBoxXDim / 2.0, position.Y() / 10.,
                            position.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
                            HitBoxZDim); // This coordinates are x,y,z in our system

      } else if (f3DHitStyle == 1) {

         Float_t HitBoxYDim = hit.GetCharge() * 0.0002;
         Float_t HitBoxZDim = hit.GetCharge() * 0.0002;
         Float_t HitBoxXDim = hit.GetCharge() * 0.0002;

         fhitBoxSet->AddBox(position.X() / 10. - HitBoxXDim / 2.0, position.Y() / 10. - HitBoxYDim / 2.0,
                            position.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
                            HitBoxZDim); // This coordinates are x,y,z in our system
      }

      Float_t xrgb = 255, yrgb = 0, zrgb = 0;
      if (fPadPlanePal) {

         Int_t cHit = fPadPlanePal->GetValueColor(hit.GetCharge());
         TColor *hitBoxColor = gROOT->GetColor(cHit);
         hitBoxColor->GetRGB(xrgb, yrgb, zrgb);
      }

      fhitBoxSet->DigitColor(xrgb * 255, yrgb * 255, zrgb * 255, 0);

      dumpEvent << position.X() << " " << position.Y() << " " << position.Z() << " " << hit.GetTimeStamp() << " "
                << hit.GetCharge() << std::endl;
   }

   /////////////////////// End of colored box drawing ////////////////////////////

   fhitBoxSet->RefitPlex();
   TEveTrans &tHitBoxPos = fhitBoxSet->RefMainTrans();
   tHitBoxPos.SetPos(0.0, 0.0, 0.0);

   // for(Int_t i=0;i<hitSphereArray.size();i++) gEve->AddElement(hitSphereArray[i]);

   if (fIsRawData) {

      fRawevent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

      if (fRawevent) {
         fRawevent->SetName("fRawEvent");
         gROOT->GetListOfSpecials()->Add(fRawevent);

         Int_t numAux = 0;

         for (auto &padIt : fRawevent->GetAuxPads()) {
            const AtAuxPad &pad = padIt.second;
            if (numAux < 9) {
               std::cout << cYELLOW << " Auxiliary Channel " << numAux << " - Name " << pad.GetAuxName() << cNORMAL
                         << std::endl;
               auto rawAdc = pad.GetRawADC();
               for (int i = 0; i < 512; ++i)
                  fAuxChannels[numAux]->SetBinContent(i, rawAdc[i]);
               numAux++;
            }
         }
         if (numAux + 1 == 9)
            std::cout << cYELLOW << "Warning: More auxiliary channels than expected (max 9)" << cNORMAL << std::endl;

      } // if raw event
   }

   if (fIsRawData) {

      if (fRawevent != nullptr) {

         Int_t nPads = fRawevent->GetNumPads();
         std::cout << "Num of pads : " << nPads << std::endl;

         for (Int_t iPad = 0; iPad < nPads; iPad++) {

            AtPad *fPad = fRawevent->GetPad(iPad);
            // std::cout<<"Pad num : "<<iPad<<" Is Valid? : "<<fPad->GetValidPad()<<" Pad num in pad object
            // :"<<fPad->GetPadNum()<<std::endl;

            if (fPad != nullptr) {
               auto rawadc = fPad->GetRawADC();
               auto adc = fPad->GetADC();
               Int_t PadNum_temp = fPad->GetPadNum();
               // dumpEvent<<TSpad<<fPad->GetPadNum()<<std::endl;
               if (fPad->GetValidPad() && PadNum_temp < 2015 && PadNum_temp > -1) {

                  for (Int_t j = 0; j < 512; j++) {
                     fPadAll[PadNum_temp]->SetBinContent(j, adc[j]);
                  }
               }
            }

         } // Pads

      } // NULL

   } // Rawdata

   if (fEventManager->GetDrawPatternRecognition()) {

      for (auto &i : fLineArray)
         i = new TEveLine();
      int n = 100;
      double t0 = 0;
      double dt = 2000;
      std::vector<AtTrack> TrackCand;

      if (fPatternEventArray) {

         auto *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));

         if (patternEvent != nullptr) {
            TrackCand = patternEvent->GetTrackCand();
            for (auto &i : fHitSetPR)
               i = nullptr;

            fLineNum = TrackCand.size();
            std::cout << cRED << " Found " << TrackCand.size() << " track candidates " << cNORMAL << std::endl;

            if (TrackCand.size() < 20) {
               for (Int_t i = 0; i < TrackCand.size(); i++) {

                  AtTrack track = TrackCand.at(i);
                  std::vector<AtHit> trackHits = track.GetHitArrayObject();
                  int nHitsMin = trackHits.size();

                  fHitSetPR[i] = new TEvePointSet(Form("HitPR_%d", i), nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
                  fHitSetPR[i]->SetMarkerColor(GetTrackColor(i) + 1);
                  fHitSetPR[i]->SetMarkerSize(fHitSize);
                  fHitSetPR[i]->SetMarkerStyle(fHitStyle);

                  for (auto &trackHit : trackHits) {
                     auto position = trackHit.GetPosition();
                     fHitSetPR[i]->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.);
                  }

                  if (fEventManager->GetDrawPatternRecognition())
                     gEve->AddElement(fHitSetPR[i]);
                  kIsPRDrawn = kTRUE;
               }
            }
         }
      }
   }

   if (!fEventManager->GetDrawPatternRecognition()) {

      gEve->AddElement(fHitSet);
      gEve->AddElement(fhitBoxSet);

   } else if (fEventManager->GetDrawPatternRecognition()) {

      // if(fPatternEventArray)
      //  if(fLineNum>0) for(Int_t i=0;i<fLineNum;i++) gEve -> AddElement(fHitSetPR[i]);
   }

   dumpEvent.close();
}

void AtEventDrawTaskProto::DrawProtoPattern()
{

   for (Int_t i = 0; i < 4; i++) {
      fQHitPattern[i]->Set(0);
      fQELossPattern[i]->Set(0);
   }
   auto *protoevent = dynamic_cast<AtProtoEvent *>(fProtoEventArray->At(0));
   Int_t nQuads = protoevent->GetNumQuadrants();
   std::vector<AtProtoQuadrant> quadrantArray;

   // fHoughSpaceLine  = dynamic_cast<AtHoughSpaceLine*> (fHoughSpaceArray->At(0));
   // std::vector<std::pair<Double_t,Double_t>> HoughPar = fHoughSpaceLine->GetHoughPar();

   if (nQuads < 5) {
      for (Int_t iQ = 0; iQ < nQuads; iQ++) {

         AtProtoQuadrant quadrant = protoevent->GetQuadrantArray()->at(iQ);
         quadrantArray.push_back(protoevent->GetQuadrantArray()->at(iQ));
         std::vector<Double_t> *PhiArray = quadrantArray[iQ].GetPhiArray();

         for (double pval : *PhiArray) {
            fPhiDistr[iQ]->Fill(pval);
         }
         PhiArray->clear();

         Int_t qNumHit = quadrant.GetNumHits();

         for (Int_t j = 0; j < qNumHit; j++) {

            AtHit *qhit = quadrant.GetHit(j);
            auto position = qhit->GetPosition();

            Double_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));
            fQHitPattern[iQ]->SetPoint(fQHitPattern[iQ]->GetN(), radius, position.Z());
            fQELossPattern[iQ]->SetPoint(fQELossPattern[iQ]->GetN(), radius, qhit->GetCharge());
         }
      }
   }
}

void AtEventDrawTaskProto::DrawProtoHough() {}

void AtEventDrawTaskProto::DrawProtoPatternAna()
{

   for (auto &i : fQELossPatternAna)
      i->Set(0);

   fQVertex[2]->Reset(nullptr);
   fQVertex[3]->Reset(nullptr);
   fQKine[2]->Reset(nullptr);
   fQKine[3]->Reset(nullptr);

   auto *protoeventAna = dynamic_cast<AtProtoEventAna *>(fProtoEventAnaArray->At(0));

   std::vector<Double_t> *Par0 = protoeventAna->GetPar0();
   std::vector<Double_t> *Par1 = protoeventAna->GetPar1();

   // std::vector<std::pair<Double_t,Double_t>>* ELossHitPattern = protoeventAna->GetELossHitPattern();
   std::vector<std::vector<std::pair<Double_t, Double_t>>> *QELossHitPattern = protoeventAna->GetQELossHitPattern();

   for (Int_t i = 0; i < QELossHitPattern->size(); i++) {
      std::vector<std::pair<Double_t, Double_t>> ELossHitPattern = QELossHitPattern->at(i);
      fFit[i]->SetParameter(0, Par0->at(i));
      fFit[i]->SetParameter(1, Par1->at(i));

      for (auto HPbuffer : ELossHitPattern) {
         Double_t radius = HPbuffer.second;
         Double_t charge = HPbuffer.first;
         fQELossPatternAna[i]->SetPoint(fQELossPatternAna[i]->GetN(), radius, charge);
      }
   }

   std::vector<Double_t> *vertex = protoeventAna->GetVertex();
   std::vector<Double_t> *KineAA = protoeventAna->GetAngleFit();
   std::vector<Double_t> *Chi2 = protoeventAna->GetChi2();
   std::vector<Int_t> *NDF = protoeventAna->GetNDF();
   fQVertex[0]->Fill(vertex->at(0), vertex->at(2));
   fQVertex[1]->Fill(vertex->at(1), vertex->at(3));
   fQVertex[2]->Fill(vertex->at(0), vertex->at(2));
   fQVertex[3]->Fill(vertex->at(1), vertex->at(3));

   fQKine[0]->Fill(KineAA->at(0), KineAA->at(2));
   fQKine[1]->Fill(KineAA->at(1), KineAA->at(3));
   fQKine[2]->Fill(KineAA->at(0), KineAA->at(2));
   fQKine[3]->Fill(KineAA->at(1), KineAA->at(3));

   std::cout << cYELLOW << " ==================================================================== " << std::endl;
   std::cout << "                                  AtEventDrawTask : Fit Results                     " << std::endl;
   std::cout << "       -   Quadrant 0   -   Quadrant 1   -   Quadrant 2  -   Quadrant 3      " << std::endl;
   std::cout << "  Angle  : " << KineAA->at(0) << "              " << KineAA->at(1) << "              " << KineAA->at(2)
             << "                " << KineAA->at(3) << std::endl;
   std::cout << "  Vertex : " << vertex->at(0) << "              " << vertex->at(1) << "              " << vertex->at(2)
             << "                " << vertex->at(3) << std::endl;
   std::cout << "  Chi2   : " << Chi2->at(0) << "              " << Chi2->at(1) << "              " << Chi2->at(2)
             << "                " << Chi2->at(3) << std::endl;
   std::cout << "  NDF    : " << NDF->at(0) << "              " << NDF->at(1) << "              " << NDF->at(2)
             << "                " << NDF->at(3) << std::endl;
   std::cout << " ==================================================================== " << cNORMAL << std::endl;
   std::cout << std::endl;
}

// Draw functions ////

void AtEventDrawTaskProto::DrawPadWave()
{

   fPadWave = new TH1I("fPadWave", "fPadWave", 512, 0, 511);
   gROOT->GetListOfSpecials()->Add(fPadWave);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

void AtEventDrawTaskProto::DrawPadPlane()
{

   /*if(fPadPlane)
   {
     fPadPlane->Reset(0);
     return;
   }*/

   fPadPlane =
      dynamic_cast<AtTpcProtoMap *>(fDetmap)->GetAtTpcPlane("ATTPC_Proto"); // NB: Do not change the pad plane name
   fCvsPadPlane->cd();
   // fPadPlane -> Draw("zcol");
   // fPadPlane -> Draw("COL L0");
   fPadPlane->Draw("COL L");
   fPadPlane->SetMinimum(1.0);
   gStyle->SetOptStat(0);
   gStyle->SetPalette(103);
   gPad->Update();
}

void AtEventDrawTaskProto::DrawPadAll()
{

   fCvsPadAll->cd();

   for (auto &i : fPadAll) {
      // fPadAll[i]->Reset(0);
      // fPadAll[i] = new TH1I("fPadAll","fPadAll",512,0,511);
      i->GetYaxis()->SetRangeUser(0, 2500);
      i->SetLineColor(8);

      /*if (i<64) fPadAll[i]->SetLineColor(6);                         // Q1, pink
      else if(i>=64 && i<127) fPadAll[i]->SetLineColor(8);           // Q2, green
      else if(i>=127 && i<190) fPadAll[i]->SetLineColor(7);           // Q3, blue
      else if(i>=190 && i<253) fPadAll[i]->SetLineColor(kOrange-3);   // Q4, orange
      else fPadAll[i]->SetLineColor(0);                              //white for non physical pads*/

      i->Draw("SAME");
   }
}

void AtEventDrawTaskProto::DrawMesh()
{

   fCvsMesh->cd();
   fMesh = new TH1F("Mesh", "Mesh", 512, 0, 511);
   fMesh->Draw();
}

void AtEventDrawTaskProto::DrawProtoSpace() {}

void AtEventDrawTaskProto::DrawProtoEL()
{

   for (Int_t i = 0; i < 4; i++) {
      fQELossPattern[i] = new TGraph();
      fQELossPattern[i]->SetMarkerStyle(22);
      fQELossPattern[i]->SetMarkerSize(0.7);
      fQELossPattern[i]->SetPoint(1, 0, 0);
      fQELossPatternAna[i] = new TGraph();
      fQELossPatternAna[i]->SetMarkerStyle(20);
      fQELossPatternAna[i]->SetMarkerColor(kRed);
      fQELossPatternAna[i]->SetMarkerSize(0.7);
      fQELossPatternAna[i]->SetPoint(1, 0, 0);
      if (i == 0) {
         fCvsELQuadrant1->cd();
         fQELossPattern[0]->Draw("AP");
         if (fProtoEventAnaArray)
            fQELossPatternAna[0]->Draw("P");
      } else if (i == 1) {
         fCvsELQuadrant2->cd();
         fQELossPattern[1]->Draw("AP");
         if (fProtoEventAnaArray)
            fQELossPatternAna[1]->Draw("P");
      } else if (i == 2) {
         fCvsELQuadrant3->cd();
         fQELossPattern[2]->Draw("AP");
         if (fProtoEventAnaArray)
            fQELossPatternAna[2]->Draw("P");
      } else if (i == 3) {
         fCvsELQuadrant4->cd();
         fQELossPattern[3]->Draw("AP");
         if (fProtoEventAnaArray)
            fQELossPatternAna[3]->Draw("P");
      }
   }
}

void AtEventDrawTaskProto::DrawProtoELAna()
{

   for (Int_t i = 0; i < 4; i++) {
      fQELossPatternAna[i] = new TGraph();
      fQELossPatternAna[i]->SetMarkerStyle(22);
      fQELossPatternAna[i]->SetMarkerSize(0.7);
      fQELossPatternAna[i]->SetMarkerStyle(kRed);
      fQELossPatternAna[i]->SetPoint(1, 0, 0);
      if (i == 0) {
         fCvsELQuadrant1->cd();
         fQELossPatternAna[0]->Draw("A*");
      } else if (i == 1) {
         fCvsELQuadrant2->cd();
         fQELossPatternAna[1]->Draw("A*");
      } else if (i == 2) {
         fCvsELQuadrant3->cd();
         fQELossPatternAna[2]->Draw("A*");
      } else if (i == 3) {
         fCvsELQuadrant4->cd();
         fQELossPatternAna[3]->Draw("A*");
      }
   }
}

void AtEventDrawTaskProto::DrawProtoVertex()
{

   for (Int_t i = 0; i < 4; i++) {
      fQVertex[i] = new TH2F(Form("Vertex_%i", i), Form("Vertex%i", i), 1000, 0, 1000, 1000, 0, 1000);
      fQVertex[i]->SetMarkerSize(1.2);
      fQVertex[i]->SetMarkerStyle(22);
      if (i == 0)
         fQVertex[i]->SetMarkerColor(kRed);
      else if (i == 1) {
         fQVertex[i]->SetMarkerColor(kRed);
         fQVertex[i]->SetMarkerStyle(20);
      } else if (i == 2)
         fQVertex[i]->SetMarkerColor(kBlue);
      else if (i == 3) {
         fQVertex[i]->SetMarkerColor(kBlue);
         fQVertex[i]->SetMarkerStyle(20);
      }
   }

   fCvsVertex->cd();
   fQVertex[0]->Draw();
   fQVertex[1]->Draw("SAME");
   fQVertex[2]->Draw("SAME");
   fQVertex[3]->Draw("SAME");
}

void AtEventDrawTaskProto::DrawProtoKine()
{

   for (Int_t i = 0; i < 4; i++) {
      fQKine[i] = new TH2F(Form("Angle_Angle_Kinematics_%i", i), Form("Angle_Angle_Kinematics%i", i), 1000, 0, 180,
                           1000, 0, 180);
      fQKine[i]->SetMarkerSize(1.2);
      fQKine[i]->SetMarkerStyle(22);
      if (i == 0)
         fQKine[i]->SetMarkerColor(kRed);
      else if (i == 1) {
         fQKine[i]->SetMarkerColor(kRed);
         fQKine[i]->SetMarkerStyle(20);
      } else if (i == 2)
         fQKine[i]->SetMarkerColor(kBlue);
      else if (i == 3) {
         fQKine[i]->SetMarkerColor(kBlue);
         fQKine[i]->SetMarkerStyle(20);
      }
   }

   fCvsKineAA->cd();
   fQKine[0]->Draw();
   fQKine[1]->Draw("SAME");
   fQKine[2]->Draw("SAME");
   fQKine[3]->Draw("SAME");
}

void AtEventDrawTaskProto::DrawProtoAux()
{

   fCvsAux->Divide(3, 3);
   for (Int_t i = 0; i < 9; i++) {
      fAuxChannels[i] = new TH1F(Form("Auxiliary_Channel_%i", i), Form("AuxChannel%i", i), 512, 0, 511);
      fCvsAux->cd(1 + i);
      fAuxChannels[i]->Draw();
   }
}

/// Update functions //////

void AtEventDrawTaskProto::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();
}

void AtEventDrawTaskProto::UpdateCvsPadPlane()
{
   fCvsPadPlane->Modified();
   fCvsPadPlane->Update();
}

void AtEventDrawTaskProto::UpdateCvsPadAll()
{
   fCvsPadAll->Modified();
   fCvsPadAll->Update();
}

void AtEventDrawTaskProto::UpdateCvsMesh()
{

   fCvsMesh->Modified();
   fCvsMesh->Update();
}

void AtEventDrawTaskProto::UpdateCvsProtoQ()
{

   fCvsQuadrant1->Modified();
   fCvsQuadrant1->Update();
   fCvsQuadrant2->Modified();
   fCvsQuadrant2->Update();
   fCvsQuadrant3->Modified();
   fCvsQuadrant3->Update();
   fCvsQuadrant4->Modified();
   fCvsQuadrant4->Update();
}

void AtEventDrawTaskProto::UpdateCvsProtoEL()
{

   fCvsELQuadrant1->Modified();
   fCvsELQuadrant1->Update();
   fCvsELQuadrant2->Modified();
   fCvsELQuadrant2->Update();
   fCvsELQuadrant3->Modified();
   fCvsELQuadrant3->Update();
   fCvsELQuadrant4->Modified();
   fCvsELQuadrant4->Update();
}

void AtEventDrawTaskProto::UpdateCvsProtoVertex()
{

   fCvsVertex->Modified();
   fCvsVertex->Update();
}

void AtEventDrawTaskProto::UpdateCvsProtoKine()
{

   fCvsKineAA->Modified();
   fCvsKineAA->Update();
}

void AtEventDrawTaskProto::UpdateCvsProtoAux()
{

   // for(Int_t i = 0;i<4;i++){
   // fCvsAux->cd(1);
   TPad *Pad_1 = dynamic_cast<TPad *>(fCvsAux->GetPad(1));
   Pad_1->Modified();
   Pad_1->Update();
   TPad *Pad_2 = dynamic_cast<TPad *>(fCvsAux->GetPad(2));
   Pad_2->Modified();
   Pad_2->Update();
   TPad *Pad_3 = dynamic_cast<TPad *>(fCvsAux->GetPad(3));
   Pad_3->Modified();
   Pad_3->Update();
   TPad *Pad_4 = dynamic_cast<TPad *>(fCvsAux->GetPad(4));
   Pad_4->Modified();
   Pad_4->Update();
   TPad *Pad_5 = dynamic_cast<TPad *>(fCvsAux->GetPad(5));
   Pad_5->Modified();
   Pad_5->Update();
   TPad *Pad_6 = dynamic_cast<TPad *>(fCvsAux->GetPad(6));
   Pad_6->Modified();
   Pad_6->Update();
   TPad *Pad_7 = dynamic_cast<TPad *>(fCvsAux->GetPad(7));
   Pad_7->Modified();
   Pad_7->Update();
   TPad *Pad_8 = dynamic_cast<TPad *>(fCvsAux->GetPad(8));
   Pad_8->Modified();
   Pad_8->Update();
   TPad *Pad_9 = dynamic_cast<TPad *>(fCvsAux->GetPad(9));
   Pad_9->Modified();
   Pad_9->Update();
   fCvsAux->Modified();
   fCvsAux->Update();
   //}
}

void AtEventDrawTaskProto::SelectPad(const char *rawevt)
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
               << " = AtEventDrawTaskProto::SelectPad NULL pointer for the AtRawEvent! Please select an event first "
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
            gVirtualX->DrawLine(pxmin, pyold, pxmax, pyold);
         gVirtualX->DrawLine(pxmin, py, pxmax, py);
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
         // new AtTpcProtoMap();
         // TString map = "/Users/yassidayyad/fair_install/AtTPCROOT_v2_06042015/scripts/proto.map";
         // tmap->SetProtoMap(map.Data());
         // Int_t tPadNum = tmap->BinToPad(bin);

         Int_t tPadNum = tmap->BinToPad(bin);

         std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
         AtPad *tPad = tRawEvent->GetPad(tPadNum);

         // Check to make sure pad is valid
         if (!tPad)
            return;

         std::cout << " Event ID (Select Pad) : " << tRawEvent->GetEventID() << std::endl;
         std::cout << " Raw Event Pad Num " << tPad->GetPadNum() << std::endl;
         std::cout << std::endl;
         // TH1D* tPadWaveSub = NULL;
         // tPadWaveSub = new TH1D("tPadWaveSub","tPadWaveSub",512.0,0.0,511.0);
         // tPadWaveSub->SetLineColor(kRed);
         TH1I *tPadWave = nullptr;
         tPadWave = dynamic_cast<TH1I *>(gROOT->GetListOfSpecials()->FindObject("fPadWave"));
         auto rawadc = tPad->GetRawADC();
         auto adc = tPad->GetADC();
         if (tPadWave == nullptr) {
            std::cout << " = AtEventDrawTask::SelectPad NULL pointer for the TH1I! Please select an event first "
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
            std::cout << " = AtEventDrawTask::SelectPad NULL pointer for the TCanvas! Please select an event first "
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

void AtEventDrawTaskProto::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

void AtEventDrawTaskProto::Set3DHitStyleBar()
{
   f3DHitStyle = 0;
}

void AtEventDrawTaskProto::Set3DHitStyleBox()
{
   f3DHitStyle = 1;
}

EColor AtEventDrawTaskProto::GetTrackColor(int i)
{
   std::vector<EColor> colors = {kAzure, kOrange, kViolet, kTeal, kMagenta, kBlue, kViolet, kYellow, kCyan, kAzure};
   if (i < 10) {
      return colors.at(i);
   } else
      return kAzure;
}
