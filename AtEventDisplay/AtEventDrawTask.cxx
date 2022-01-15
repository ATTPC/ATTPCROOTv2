/**
 * @brief Event display task
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */

#include "AtEventDrawTask.h"

#include "AtEvent.h"
#include "AtEventManager.h"
#include "AtHit.h"
#include "AtHoughSpace.h"
#include "AtHoughSpaceCircle.h"
#include "AtHoughSpaceLine.h"
#include "AtLmedsMod.h"
#include "AtMlesacMod.h"
#include "AtPatternEvent.h"
#include "AtRansac.h"
#include "AtRansacMod.h"
#include "AtRawEvent.h"
#include "AtMap.h"
#include "AtGadgetIIMap.h"
#include "AtSpecMATMap.h"
#include "AtTpcMap.h"
#include "AtTrackFinderHC.h"
#include "AtTrackingEventAna.h"

#include "FairLogger.h"
#include "FairRootManager.h"

#include "TClonesArray.h"
#include "TColor.h"
#include "TEveBoxSet.h"
#include "TEveGeoShape.h"
#include "TEveLine.h"
#include "TEveManager.h"
#include "TEvePointSet.h"
#include "TEveTrans.h"
#include "TF1.h"
#include "TGeoSphere.h"
#include "TH1.h"
#include "TH2.h"
#include "TH2Poly.h"
#include "TH3.h"
#include "TPaletteAxis.h"
#include "TRandom.h"
#include "TStyle.h"
#include "TVector3.h"
#include "TVirtualX.h"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"
#define cBLUE "\033[1;34m"

using namespace std;

ClassImp(AtEventDrawTask);

AtEventDrawTask::AtEventDrawTask()
   : fIs2DPlotRange(kFALSE), fUnpackHough(kFALSE), fHitArray(0),
     // fHitClusterArray(0),
     // fRiemannTrackArray(0),
     // fKalmanArray(0),
     fEventManager(0), fRawevent(0), fHoughSpaceArray(0), fDetmap(0), fThreshold(0), fHitSet(0),
     // x(0),
     // hitSphereArray(0),
     fhitBoxSet(0), fPadPlanePal(0), fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium),
     // fHitClusterSet(0),
     // fHitClusterColor(kAzure-5),
     // fHitClusterColor(kYellow-5),
     // fHitClusterSize(1),
     // fHitClusterStyle(kOpenCircle),
     // fRiemannSetArray(0),
     // fRiemannColor(kBlue),
     // fRiemannSize(1.5),
     // fRiemannStyle(kOpenCircle),
     fCvsPadPlane(0), fPadPlane(0), fCvsPadWave(0), fPadWave(0), fCvsPadAll(0), fCvsQEvent(0), fQEventHist(0),
     fQEventHist_H(0), fCvsHoughSpace(0), fHoughSpace(0), fCvsRhoVariance(0), fRhoVariance(0), fCvsPhi(0), fCvsMesh(0),
     fMesh(0), fCvs3DHist(0), f3DHist(0), fCvsRad(0), fRadVSTb(0), fCvsTheta(0), fTheta(0), fAtMapPtr(0), fMinZ(0),
     fMaxZ(1344), fMinX(432), fMaxX(-432), f3DHitStyle(0), fMultiHit(0), fSaveTextData(0), f3DThreshold(0),
     fRANSACAlg(0), fDetNumPads(10240), fRawEventBranchName("AtRawEvent"), fEventBranchName("AtEventH")
{

   Char_t padhistname[256];
   fMultiHit = 10;

   for (Int_t i = 0; i < fNumPads; i++) { // TODO: Full-scale must be accomodated
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
   fIsLinearHough = kFALSE;
   fIsRawData = kFALSE;

   fHoughLinearFit =
      new TF1("HoughLinearFit", " (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) + [1]/TMath::Sin([0])", 0, 500);
   fRansacLinearFit = new TF1("RansacLinearFit", "x", 0, 500);
   fIniHit = new AtHit();
   fIniHitRansac = new AtHit();
   fLineNum = 0;
   fTrackNum = 0;

   fDetectorId = kAtTpc;

   fRGBAPalette = new TEveRGBAPalette(0, 4096);
}

AtEventDrawTask::~AtEventDrawTask()
{

   // TODO Destroy pointers
   // for(Int_t i=0;i<hitSphereArray.size();i++) delete hitSphereArray[i];
   // delete x;
   // hitSphereArray.clear();
   delete fHoughLinearFit;
   delete fRansacLinearFit;
}

InitStatus AtEventDrawTask::Init()
{

   std::cout << " =====  AtEventDrawTask::Init =====" << std::endl;
   // Selection of pad plane
   std::cout << cGREEN << " Selected detector/pad plane :" << cNORMAL;
   DetectorId det = fDetectorId;
   switch (det) {
   case kAtTpc:
      std::cout << cGREEN << " ATTPC\n" << cNORMAL;
      fDetNumPads = 10240;
      fDetmap = new AtTpcMap();
      break;

   case kGADGETII:
      std::cout << cGREEN << " GADGETII\n" << cNORMAL;
      fDetNumPads = 1012;
      fDetmap = new AtGadgetIIMap();
      break;

   case kSpecMAT:
      std::cout << cGREEN << " SpecMAT\n" << cNORMAL;
      fDetNumPads = 3162;
      fDetmap = new AtSpecMATMap(fDetNumPads);
      break;

   default:
      std::cout << cRED << " Pad plane not selected! Exiting...\n" << cNORMAL;
      exit(0);
      break;
   }

   gROOT->Reset();
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManager::Instance();

   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap);

   fHitArray = (TClonesArray *)ioMan->GetObject(fEventBranchName);
   if (fHitArray)
      LOG(INFO) << cGREEN << "Hit Array Found in branch " << fEventBranchName << "." << cNORMAL << std::endl;

   fRawEventArray = (TClonesArray *)ioMan->GetObject(fRawEventBranchName);
   if (fRawEventArray) {
      LOG(INFO) << cGREEN << "Raw Event Array Found in branch " << fRawEventBranchName << "." << cNORMAL << std::endl;
      fIsRawData = kTRUE;
   }

   fHoughSpaceArray = (TClonesArray *)ioMan->GetObject("AtHough");
   if (fHoughSpaceArray)
      LOG(INFO) << cGREEN << "Hough Array Found." << cNORMAL << std::endl;

   fRansacArray = (TClonesArray *)ioMan->GetObject("AtRansac");
   if (fRansacArray)
      LOG(INFO) << cGREEN << "RANSAC Array Found." << cNORMAL << std::endl;

   // fTrackFinderHCArray = (TClonesArray*) ioMan->GetObject("AtTrackFinderHC");
   // if(fTrackFinderHCArray)  LOG(INFO)<<cGREEN<<"Track Finder Hierarchical Clustering Array
   // Found."<<cNORMAL<<std::endl;

   fPatternEventArray = (TClonesArray *)ioMan->GetObject("AtPatternEvent");
   if (fPatternEventArray)
      LOG(INFO) << cGREEN << "Pattern Event Array Found." << cNORMAL << std::endl;

   fTrackingEventAnaArray = (TClonesArray *)ioMan->GetObject("AtTrackingEventAna");
   if (fTrackingEventAnaArray)
      LOG(INFO) << cGREEN << "Tracking Event Analysis Array Found." << cNORMAL << std::endl;

   // gROOT->GetListOfSpecials()->Add(fRawEventArray);
   // fRawEventArray->SetName("AtRawEvent");

   gStyle->SetPalette(55);
   // fPhiDistr=NULL;

   // Get all of the pads from the display manager

   fCvsPadWave = fEventManager->GetCvsPadWave();
   fCvsPadWave->SetName("fCvsPadWave");
   gROOT->GetListOfSpecials()->Add(fCvsPadWave);
   DrawPadWave();
   fCvsPadPlane = fEventManager->GetCvsPadPlane(); // There is a problem if the pad plane is drawn first
   fCvsPadPlane->ToggleEventStatus();
   fCvsPadPlane->AddExec("ex", "AtEventDrawTask::SelectPad(\"fRawEvent\")");
   DrawPadPlane();
   fCvsPadAll = fEventManager->GetCvsPadAll();
   DrawPadAll();
   // fCvs3DHist = new TCanvas("glcvs3dhist");
   fCvs3DHist = fEventManager->GetCvs3DHist();
   Draw3DHist();
   fCvsQEvent = new TCanvas("fCvsQEvent", "fCvsQEvent");
   DrawQEvent();
   fCvsRhoVariance = new TCanvas("fCvsRhoVariance", "fCvsRhoVariance");
   DrawRhoVariance();
   fCvsPhi = fEventManager->GetCvsPhi();
   DrawPhiReco();
   fCvsMesh = fEventManager->GetCvsMesh();
   DrawMesh();
   fCvsRad = fEventManager->GetCvsRad();
   DrawRad();
   fCvsTheta = fEventManager->GetCvsTheta();
   DrawTheta();
   fCvsThetaxPhi = fEventManager->GetCvsThetaxPhi();
   DrawThetaxPhi();
   fCvsMC_XY = fEventManager->GetCvsMC_XY();
   fCvsMC_Z = fEventManager->GetCvsMC_Z();
   DrawMC();
   /*fCvsQuadrant1 = fEventManager->GetCvsQuadrant1();
    fCvsQuadrant2 = fEventManager->GetCvsQuadrant2();
    fCvsQuadrant3 = fEventManager->GetCvsQuadrant3();
    fCvsQuadrant4 = fEventManager->GetCvsQuadrant4();
    DrawHoughSpaceProto();*/
   fCvsAux = fEventManager->GetCvsAux();
   DrawAux();

   //******* NO CALLS TO TCANVAS BELOW THIS ONE
   fCvsHoughSpace = fEventManager->GetCvsHoughSpace();
   DrawHoughSpace();

   std::cout << " AtEventDrawTask::Init : Initialization complete! "
             << "\n";
   return kSUCCESS;
}

void AtEventDrawTask::Exec(Option_t *option)
{
   Reset();
   ResetPadAll();
   ResetPhiDistr();

   if (fHoughSpaceArray) {
      if (fHoughSpaceLine_buff = dynamic_cast<AtHoughSpaceLine *>(fHoughSpaceArray->At(0))) {
         std::cout << " Linear Hough Space Found!" << std::endl;
         fIsLinearHough = kTRUE;
      } else if (fHoughSpaceCircle_buff = dynamic_cast<AtHoughSpaceCircle *>(fHoughSpaceArray->At(0))) {
         std::cout << "Circular Hough Space Found!" << std::endl;
         fIsCircularHough = kTRUE;
      } else
         std::cout << "Hough Space Type NOT Found!" << std::endl;

      if (fIsCircularHough) {
         Double_t XCenter = fHoughSpaceCircle_buff->GetXCenter();
         Double_t YCenter = fHoughSpaceCircle_buff->GetYCenter();
         std::pair<Double_t, Double_t> LinearHoughPar = fHoughSpaceCircle_buff->GetHoughPar();
         std::cout << cYELLOW << " ---- Spiral Hough Space Calculation ----" << std::endl;
         std::cout << " X Center : " << XCenter << " Y Center : " << YCenter << std::endl;
         std::cout << " Radius x Phi Linear Hough parameters - Angle : " << LinearHoughPar.first << "  - Distance "
                   << LinearHoughPar.second << cNORMAL << std::endl;
         fHoughLinearFit->SetParameter(0, TMath::Pi() / 2.0 - LinearHoughPar.first);
         fHoughLinearFit->SetParameter(1, LinearHoughPar.second);
      }
   }

   // if (fRawEventArray)
   //  DrawPadAll();

   if (fHitArray) {
      DrawHitPoints();
      DrawMeshSpace();
   }
   if (fHoughSpaceArray && fUnpackHough)
      DrawHSpace();

   gEve->Redraw3D(kFALSE);

   UpdateCvsPadPlane();
   UpdateCvsPadWave();
   UpdateCvsPadAll();
   UpdateCvsQEvent();
   UpdateCvsRhoVariance();
   UpdateCvsPhi();
   UpdateCvsMesh();
   UpdateCvs3DHist();
   if (fUnpackHough && fEventManager->GetDrawHoughSpace()) {
      UpdateCvsHoughSpace();
      UpdateCvsRad();
      UpdateCvsTheta();
      UpdateCvsThetaxPhi();
      UpdateCvsMC();
      // UpdateCvsQuadrants();
   }
}

void AtEventDrawTask::DrawHitPoints()
{

   Float_t *MeshArray;
   fMesh->Reset(0);

   for (int i = 0; i < 9; ++i)
      fAuxChannels[i]->Reset(0);

   f3DHist->Reset(0);
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   std::vector<Double_t> fPosXMin;
   std::vector<Double_t> fPosYMin;
   std::vector<Double_t> fPosZMin;

   fQEventHist_H->Reset(0);
   AtEvent *event = (AtEvent *)fHitArray->At(0); // TODO: Why this confusing name? It should be fEventArray
   // event->SortHitArray(); // Works surprisingly well
   Double_t Qevent = event->GetEventCharge();
   Double_t RhoVariance = event->GetRhoVariance();
   MeshArray = event->GetMesh();
   Int_t eventID = event->GetEventID();
   //  std::ofstream dumpEvent;
   //  dumpEvent.open ("event.dat");
   TString TSevt = " Event ID : ";
   TString TSpad = " Pad ID : ";
   dumpEvent << TSevt << eventID << std::endl;

   if (fEventManager->GetEraseQEvent()) {
      fQEventHist->Reset();
      fRhoVariance->Reset();
   }

   fQEventHist->Fill(Qevent);
   fQEventHist_H->Fill(Qevent);
   fRhoVariance->Fill(RhoVariance);

   for (Int_t i = 0; i < 512; i++) {

      fMesh->SetBinContent(i, MeshArray[i]);
   }

   if (fIsRawData) {
      fRawevent = (AtRawEvent *)fRawEventArray->At(0);
      fRawevent->SetName("fRawEvent");
      gROOT->GetListOfSpecials()->Add(fRawevent);

      // Draw aux channels
      int numAux = 0;
      auto padArray = fRawevent->GetPads();

      for (auto &padIt : fRawevent->GetAuxPads()) {
         AtPad &pad = padIt.second;
         if (numAux < 9) {
            std::cout << cYELLOW << " Auxiliary Channel " << numAux << " - Name " << pad.GetAuxName() << cNORMAL
                      << std::endl;
            auto adc = pad.GetADC();
            for (int i = 0; i < 512; ++i)
               fAuxChannels[numAux]->SetBinContent(i, adc[i]);
            numAux++;
         }
      }
      if (numAux + 1 == 9)
         std::cout << cYELLOW << "Warning: More auxiliary channels than expected (max 9)" << cNORMAL << std::endl;
   }

   // std::cout<<std::endl;
   // std::cout<<" AtHit Event ID : "<<event->GetEventID()<<std::endl;
   // std::cout<<" AtRawEvent Event ID : "<<rawevent->GetEventID()<<std::endl;
   // if(event->GetEventID()!=rawevent->GetEventID()) std::cout<<" = AtEventDrawTask::DrawHitPoints : Warning, EventID
   // mismatch."<<std::endl;
   Int_t nHits = event->GetNumHits();
   fHitSet = new TEvePointSet("Hit", nHits, TEvePointSelectorConsumer::kTVT_XYZ);
   fHitSet->SetOwnIds(kTRUE);
   fHitSet->SetMarkerColor(fHitColor);
   fHitSet->SetMarkerSize(fHitSize);
   fHitSet->SetMarkerStyle(fHitStyle);
   std::cout << cBLUE << " Number of hits : " << nHits << cNORMAL << std::endl;

   // 3D visualization of the Minimized data

   Int_t nHitsMin = 0; // Initialization of the variable to ensure a non-NULL pointer
   fHitSetMin = new TEvePointSet();

   if (fEventManager->GetDrawHoughSpace()) {

      if (fIsCircularHough) {

         fPosXMin = fHoughSpaceCircle_buff->GetPosXMin();
         fPosYMin = fHoughSpaceCircle_buff->GetPosYMin();
         fPosZMin = fHoughSpaceCircle_buff->GetPosZMin();
         nHitsMin = fPosXMin.size();

         fHitSetMin = new TEvePointSet("HitMin", nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
         fHitSetMin->SetOwnIds(kTRUE);
         fHitSetMin->SetMarkerColor(kGreen);
         fHitSetMin->SetMarkerSize(fHitSize);
         fHitSetMin->SetMarkerStyle(fHitStyle);
      }
   }

   if (fEventManager->GetDrawHoughSpace()) {

      //  if(fIsLinearHough){
      // fLineArray.clear();
      for (Int_t i = 0; i < 5; i++)
         fLineArray[i] = new TEveLine();
      int n = 100;
      double t0 = 0;
      double dt = 2000;
      std::vector<AtTrack> TrackCand;

      if (fIsLinearHough)
         TrackCand = fHoughSpaceLine_buff->GetTrackCand();
      else if (fRansacArray) {
         if (fRANSACAlg == 0) {
            fRansac = dynamic_cast<AtRANSACN::AtRansac *>(fRansacArray->At(0));
            TrackCand = fRansac->GetTrackCand();
            TVector3 Vertex1 = fRansac->GetVertex1();
            TVector3 Vertex2 = fRansac->GetVertex2();
            Double_t VertexTime = fRansac->GetVertexTime();
            std::cout << cGREEN << " Vertex 1 - X : " << Vertex1.X() << " - Y : " << Vertex1.Y()
                      << "  - Z : " << Vertex1.Z() << std::endl;
            std::cout << " Vertex 2 - X : " << Vertex2.X() << " - Y : " << Vertex2.Y() << "  - Z : " << Vertex2.Z()
                      << std::endl;
            std::cout << " Vertex Time : " << VertexTime << std::endl;
            std::cout << " Vertex Mean - X : " << (Vertex1.X() + Vertex2.X()) / 2.0
                      << " - Y : " << (Vertex1.Y() + Vertex2.Y()) / 2.0
                      << "  - Z : " << (Vertex1.Z() + Vertex2.Z()) / 2.0 << cNORMAL << std::endl;

            std::vector<AtRANSACN::AtRansac::PairedLines> trackCorr = fRansac->GetPairedLinesArray();
            // std::ostream_iterator<AtRANSACN::AtRansac::PairedLines> pairLine_it (std::cout,"  ");
            if (trackCorr.size() > 0) {
               for (Int_t i = 0; i < trackCorr.size(); i++) {
                  AtRANSACN::AtRansac::PairedLines pl = trackCorr.at(i);
                  std::cout << pl << std::endl;
               }
            }
         }

         if (fRANSACAlg == 1) {
            fRansacMod = dynamic_cast<AtRansacMod *>(fRansacArray->At(0));
            TrackCand = fRansacMod->GetTrackCand();
            TVector3 Vertex1 = fRansacMod->GetVertex1();
            TVector3 Vertex2 = fRansacMod->GetVertex2();
            Double_t VertexTime = fRansacMod->GetVertexTime();
            std::cout << cGREEN << " Vertex 1 - X : " << Vertex1.X() << " - Y : " << Vertex1.Y()
                      << "  - Z : " << Vertex1.Z() << std::endl;
            std::cout << " Vertex 2 - X : " << Vertex2.X() << " - Y : " << Vertex2.Y() << "  - Z : " << Vertex2.Z()
                      << std::endl;
            std::cout << " Vertex Time : " << VertexTime << std::endl;
            std::cout << " Vertex Mean - X : " << (Vertex1.X() + Vertex2.X()) / 2.0
                      << " - Y : " << (Vertex1.Y() + Vertex2.Y()) / 2.0
                      << "  - Z : " << (Vertex1.Z() + Vertex2.Z()) / 2.0 << cNORMAL << std::endl;
         }

         if (fRANSACAlg == 2) {
            fMlesacMod = dynamic_cast<AtMlesacMod *>(fRansacArray->At(0));
            TrackCand = fMlesacMod->GetTrackCand();
            TVector3 Vertex1 = fMlesacMod->GetVertex1();
            TVector3 Vertex2 = fMlesacMod->GetVertex2();
            Double_t VertexTime = fMlesacMod->GetVertexTime();
            std::cout << cGREEN << " Vertex 1 - X : " << Vertex1.X() << " - Y : " << Vertex1.Y()
                      << "  - Z : " << Vertex1.Z() << std::endl;
            std::cout << " Vertex 2 - X : " << Vertex2.X() << " - Y : " << Vertex2.Y() << "  - Z : " << Vertex2.Z()
                      << std::endl;
            std::cout << " Vertex Time : " << VertexTime << std::endl;
            std::cout << " Vertex Mean - X : " << (Vertex1.X() + Vertex2.X()) / 2.0
                      << " - Y : " << (Vertex1.Y() + Vertex2.Y()) / 2.0
                      << "  - Z : " << (Vertex1.Z() + Vertex2.Z()) / 2.0 << cNORMAL << std::endl;
         }

         if (fRANSACAlg == 3) {
            fLmedsMod = dynamic_cast<AtLmedsMod *>(fRansacArray->At(0));
            TrackCand = fLmedsMod->GetTrackCand();
            TVector3 Vertex1 = fLmedsMod->GetVertex1();
            TVector3 Vertex2 = fLmedsMod->GetVertex2();
            Double_t VertexTime = fLmedsMod->GetVertexTime();
            std::cout << cGREEN << " Vertex 1 - X : " << Vertex1.X() << " - Y : " << Vertex1.Y()
                      << "  - Z : " << Vertex1.Z() << std::endl;
            std::cout << " Vertex 2 - X : " << Vertex2.X() << " - Y : " << Vertex2.Y() << "  - Z : " << Vertex2.Z()
                      << std::endl;
            std::cout << " Vertex Time : " << VertexTime << std::endl;
            std::cout << " Vertex Mean - X : " << (Vertex1.X() + Vertex2.X()) / 2.0
                      << " - Y : " << (Vertex1.Y() + Vertex2.Y()) / 2.0
                      << "  - Z : " << (Vertex1.Z() + Vertex2.Z()) / 2.0 << cNORMAL << std::endl;
         }

      } else if (fPatternEventArray) {
         AtPatternEvent *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
         TrackCand = patternEvent->GetTrackCand();

         for (Int_t i = 0; i < 20; i++) {
            fHitSetTFHC[i] = 0;
            fHitClusterSet[i] = 0;
         }

         if (TrackCand.size() < 20) {
            for (Int_t i = 0; i < TrackCand.size(); i++) {

               AtTrack track = TrackCand.at(i);
               std::vector<AtHit> *trackHits = track.GetHitArray();
               std::vector<AtHitCluster> *hitClusters = track.GetHitClusterArray();

               Color_t trackColor = GetTrackColor(i) + 1;

               fHitSetTFHC[i] = new TEvePointSet(Form("HitMC_%d", i), nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
               if (track.GetIsNoise())
                  fHitSetTFHC[i]->SetMarkerColor(kRed);
               else
                  fHitSetTFHC[i]->SetMarkerColor(trackColor);
               fHitSetTFHC[i]->SetMarkerSize(fHitSize);
               fHitSetTFHC[i]->SetMarkerStyle(fHitStyle);

               for (int j = 0; j < trackHits->size(); ++j) {
                  TVector3 position = trackHits->at(j).GetPosition();
                  fHitSetTFHC[i]->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.);
               }

               fHitClusterSet[i] = new TEveBoxSet(Form("HitCluster_%d", i));
               fHitClusterSet[i]->Reset(TEveBoxSet::kBT_AABox, kFALSE, 64);
               // fHitClusterSet[i]->SetPalette(fRGBAPalette);
               // fHitClusterSet[i]->DigitValue(2000);

               if (hitClusters->size() > 0 && !track.GetIsNoise()) {

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
            }
         }
      }

      if (fTrackingEventAnaArray) {

         for (Int_t i = 0; i < 5; i++)
            fHitSetMC[i] = 0;

         fTrackingEventAna = (AtTrackingEventAna *)fTrackingEventAnaArray->At(0);
         std::vector<AtTrack> anaTracks = fTrackingEventAna->GetTrackArray();
         std::cout << cYELLOW << "  ====   Tracking analysis ==== " << std::endl;
         std::cout << " Number of analyzed tracks : " << anaTracks.size() << std::endl;
         std::cout << " Vertex of reaction : " << fTrackingEventAna->GetVertex() << std::endl;

         fTrackNum = anaTracks.size();

         if (anaTracks.size() < 5) { // Limited to 5 tracks
            for (Int_t i = 0; i < anaTracks.size(); i++) {
               AtTrack track = anaTracks.at(i);
               std::cout << track << std::endl;
               fPosXMin = track.GetPosXMin();
               fPosYMin = track.GetPosYMin();
               fPosZMin = track.GetPosZMin();
               nHitsMin = fPosXMin.size();
               fHitSetMC[i] = new TEvePointSet(Form("HitMC_%d", i), nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
               fHitSetMC[i]->SetOwnIds(kTRUE);
               fHitSetMC[i]->SetMarkerColor(kGreen);
               fHitSetMC[i]->SetMarkerSize(fHitSize);
               fHitSetMC[i]->SetMarkerStyle(fHitStyle);

               for (Int_t iHit = 0; iHit < fPosXMin.size(); iHit++)
                  fHitSetMC[i]->SetNextPoint(fPosXMin.at(iHit) / 10., fPosYMin.at(iHit) / 10., fPosZMin.at(iHit) / 10.);
            }
         }
      } // If trackingEventAnaArray

      fLineNum = TrackCand.size();
      std::cout << cRED << " Found " << TrackCand.size() << " track candidates " << cNORMAL << std::endl;

      if (TrackCand.size() > 0 && fRansacArray) {

         for (Int_t j = 0; j < TrackCand.size(); j++) {
            fLineArray[j] = new TEveLine();
            AtTrack track = TrackCand.at(j);
            std::vector<Double_t> parFit = track.GetFitPar();
            fLineArray[j]->SetMainColor(kRed);
            if (parFit.size() == 4) {
               for (int i = 0; i < n; ++i) {
                  double t = t0 + dt * i / n;
                  double x, y, z;
                  SetLine(t, parFit, x, y, z);
                  fLineArray[j]->SetNextPoint(x, y, z);

                  // fLineArray.push_back(fLine);
                  // l->SetPoint(i,x,y,z);
                  // std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
               }
            } else if (parFit.size() == 6) {
               for (int i = -n; i < n; ++i) {
                  double t = t0 + dt * i / n;
                  double x, y, z;
                  SetLine6(t, parFit, x, y, z);
                  fLineArray[j]->SetNextPoint(x, y, z);

                  // fLineArray.push_back(fLine);
                  // l->SetPoint(i,x,y,z);
                  // std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
               }
            } else
               std::cout
                  << cRED
                  << " AtEventDrawTask::DrawHitPoints - Warning: wrong number of fit parameters for RANSAC lines!"
                  << std::endl;
         }
         fVertex = new TEvePointSet("Vertex", 1, TEvePointSelectorConsumer::kTVT_XYZ);
         fVertex->SetOwnIds(kTRUE);
         fVertex->SetMarkerStyle(34);
         fVertex->SetMarkerSize(2.0);
         fVertex->SetMarkerColor(kViolet);
         if (fRANSACAlg == 0)
            fVertex->SetNextPoint(fRansac->GetVertexMean().x() * 0.1, fRansac->GetVertexMean().y() * 0.1,
                                  fRansac->GetVertexMean().z() * 0.1);
         if (fRANSACAlg == 1)
            fVertex->SetNextPoint(fRansacMod->GetVertexMean().x() * 0.1, fRansacMod->GetVertexMean().y() * 0.1,
                                  fRansacMod->GetVertexMean().z() * 0.1);
         if (fRANSACAlg == 2)
            fVertex->SetNextPoint(fMlesacMod->GetVertexMean().x() * 0.1, fMlesacMod->GetVertexMean().y() * 0.1,
                                  fMlesacMod->GetVertexMean().z() * 0.1);
         if (fRANSACAlg == 3)
            fVertex->SetNextPoint(fLmedsMod->GetVertexMean().x() * 0.1, fLmedsMod->GetVertexMean().y() * 0.1,
                                  fLmedsMod->GetVertexMean().z() * 0.1);
      }
   }

   //////////////////////////////////////////////

   fhitBoxSet = new TEveBoxSet("hitBox");
   fhitBoxSet->Reset(TEveBoxSet::kBT_AABox, kTRUE, 64);

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = event->GetHitArray()->at(iHit);
      Int_t PadNumHit = hit.GetHitPadNum();
      Int_t PadMultHit = event->GetHitPadMult(PadNumHit);
      Double_t BaseCorr = hit.GetBaseCorr();
      Int_t Atbin = -1;

      if (hit.GetCharge() < fThreshold)
         continue;
      if (PadMultHit > fMultiHit)
         continue;
      TVector3 position = hit.GetPosition();
      TVector3 positioncorr = hit.GetPositionCorr();

      if (!fEventManager->GetToggleCorrData()) {
         fHitSet->SetMarkerColor(fHitColor);
         fHitSet->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.); // Convert into cm
         fHitSet->SetPointId(new TNamed(Form("Hit %d", iHit), ""));
         Atbin = fPadPlane->Fill(position.X(), position.Y(), hit.GetCharge());

      } else if (fEventManager->GetToggleCorrData()) {
         fHitSet->SetMarkerColor(kBlue);
         fHitSet->SetNextPoint(positioncorr.X() / 10., positioncorr.Y() / 10.,
                               positioncorr.Z() / 10.); // Convert into ccm
         fHitSet->SetPointId(new TNamed(Form("Corrected Hit %d", iHit), ""));
         Atbin = fPadPlane->Fill(positioncorr.X(), positioncorr.Y(), hit.GetCharge());
      }

      if (fIsRawData) {
         AtPad *RawPad = fRawevent->GetPad(PadNumHit);
         if (RawPad != nullptr) {
            Double_t *adc = RawPad->GetADC();
            for (Int_t i = 0; i < 512; i++) {

               f3DThreshold = fEventManager->Get3DThreshold();
               if (adc[i] > f3DThreshold)
                  f3DHist->Fill(position.X() / 10., position.Y() / 10., i, adc[i]);
            }
         }
      }

      if (fSaveTextData)
         if (!fEventManager->GetToggleCorrData())
            dumpEvent << position.X() << " " << position.Y() << " " << position.Z() << " " << hit.GetTimeStamp() << " "
                      << hit.GetCharge() << std::endl;
         else if (fEventManager->GetToggleCorrData())
            dumpEvent << positioncorr.X() << " " << positioncorr.Y() << " " << positioncorr.Z() << " "
                      << hit.GetTimeStamp() << " " << hit.GetCharge() << std::endl;
   }

   //////////////     Minimization from Hough Space   ///////////////
   std::cout << cGREEN << " Number of simulated points " << fPosXMin.size() << cNORMAL << std::endl;
   for (Int_t iHit = 0; iHit < fPosXMin.size(); iHit++) {
      if (fEventManager->GetDrawHoughSpace()) {
         if (fIsCircularHough) {
            fHitSetMin->SetNextPoint(fPosXMin.at(iHit) / 10., fPosYMin.at(iHit) / 10.,
                                     fPosZMin.at(iHit) / 10.); // Convert into ccm
         }
      }
   }

   //////////////////////// Colored Box Drawing ////////////////

   fPadPlane->Draw("zcol");
   gPad->Update();
   fPadPlanePal = (TPaletteAxis *)fPadPlane->GetListOfFunctions()->FindObject("palette");

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      AtHit hit = event->GetHitArray()->at(iHit);
      TVector3 position = hit.GetPosition();
      TVector3 positioncorr = hit.GetPositionCorr();

      if (f3DHitStyle == 0) {

         Float_t HitBoxYDim = hit.GetCharge() * 0.001;
         Float_t HitBoxZDim = 0.05;
         Float_t HitBoxXDim = 0.05;

         if (!fEventManager->GetToggleCorrData()) {
            fhitBoxSet->AddBox(position.X() / 10. - HitBoxXDim / 2.0, position.Y() / 10.,
                               position.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
                               HitBoxZDim); // This coordinates are x,y,z in our system
         } else if (fEventManager->GetToggleCorrData()) {
            fhitBoxSet->AddBox(positioncorr.X() / 10. - HitBoxXDim / 2.0, positioncorr.Y() / 10.,
                               positioncorr.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
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
         } else if (fEventManager->GetToggleCorrData()) {
            fhitBoxSet->AddBox(positioncorr.X() / 10. - HitBoxXDim / 2.0, positioncorr.Y() / 10. - HitBoxYDim / 2.0,
                               positioncorr.Z() / 10. - HitBoxZDim / 2.0, HitBoxXDim, HitBoxYDim,
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

            Int_t *rawadc = fPad->GetRawADC();
            Double_t *adc = fPad->GetADC();
            // dumpEvent<<TSpad<<fPad->GetPadNum()<<std::endl;

            for (Int_t j = 0; j < 512; j++)
               fPadAll[iPad]->SetBinContent(j, adc[j]);
         }
      }
   }

   // Adding raw data points
   if (!fEventManager->GetDrawHoughSpace()) {

      gEve->AddElement(fHitSet);
      gEve->AddElement(fhitBoxSet);

      // Adding pattern rec. and tracking algorithm results
   } else if (fEventManager->GetDrawHoughSpace()) {

      if (fIsCircularHough)
         gEve->AddElement(fHitSetMin);

      if (fIsLinearHough || fRansacArray) {
         if (fLineNum > 0)
            for (Int_t i = 0; i < fLineNum; i++)
               gEve->AddElement(fLineArray[i]);
         // Lines plto together with data points
         gEve->AddElement(fHitSet);
         gEve->AddElement(fhitBoxSet);
         if (fVertex)
            gEve->AddElement(fVertex);
      }

      if (fPatternEventArray)
         if (fLineNum > 0)
            for (Int_t i = 0; i < fLineNum; i++) {
               gEve->AddElement(fHitSetTFHC[i]);
               gEve->AddElement(fHitClusterSet[i]);
            }

      if (fTrackingEventAnaArray)
         if (fTrackNum > 0 && fTrackNum < 5)
            for (Int_t i = 0; i < fTrackNum; i++)
               gEve->AddElement(fHitSetMC[i]);
   }

   dumpEvent.close();

   // gEve -> AddElement(fLine);
   // if(fLineArray.size()>0) gEve -> AddElement(fLineArray.at(0));
}

void AtEventDrawTask::DrawHSpace()
{

   fRadVSTb->Reset(0);
   fTheta->Reset(0);
   fThetaxPhi->Reset(0);
   fThetaxPhi_Ini->Reset(0);
   fThetaxPhi_Ini_RANSAC->Reset(0);
   fMC_XY->Set(0);
   fMC_ZX->Set(0);
   fMC_ZY->Set(0);
   fMC_XY_exp->Set(0);
   fMC_XY_int->Set(0);
   fMC_ZX_int->Set(0);
   fMC_ZY_int->Set(0);
   fMC_XY_back->Set(0);
   fMC_ZX_back->Set(0);
   fMC_ZY_back->Set(0);

   if (fEventManager->GetDrawHoughSpace()) {
      if (fIsCircularHough) {
         fHoughSpace = fHoughSpaceCircle_buff->GetHoughSpace("XY");
         std::vector<Double_t> const *Radius = fHoughSpaceCircle_buff->GetRadiusDist();
         std::vector<Int_t> const *TimeStamp = fHoughSpaceCircle_buff->GetTimeStamp();
         std::vector<Double_t> const *Theta = fHoughSpaceCircle_buff->GetTheta();
         std::vector<Double_t> const *Dl = fHoughSpaceCircle_buff->GetDl();
         std::vector<Double_t> const *Phi = fHoughSpaceCircle_buff->GetPhi();
         fIniHit = fHoughSpaceCircle_buff->GetIniHit();
         fIniHitRansac = fHoughSpaceCircle_buff->GetIniHitRansac();

         Int_t numRad = Radius->size();
         Int_t numTheta = Theta->size();

         for (Int_t i = 0; i < numRad; i++) {
            fRadVSTb->Fill(TimeStamp->at(i), Radius->at(i));
            // fTheta->SetBinContent(TimeStamp->at(i),Theta->at(i));
            fThetaxPhi->Fill(TimeStamp->at(i), Phi->at(i) * Radius->at(i));
         }

         for (Int_t i = 0; i < numTheta; i++) {
            fTheta->Fill(Dl->at(i), Theta->at(i));
            // std::cout<<" Dl : "<<Dl->at(i)<<" Theta : "<<Theta->at(i)<<std::endl;
         }

         fThetaxPhi_Ini->Fill(fIniHit->GetTimeStamp(),
                              fHoughSpaceCircle_buff->GetIniPhi() * fHoughSpaceCircle_buff->GetIniRadius());
         fThetaxPhi_Ini_RANSAC->Fill(fIniHitRansac->GetTimeStamp(), fHoughSpaceCircle_buff->GetIniPhiRansac() *
                                                                       fHoughSpaceCircle_buff->GetIniRadiusRansac());

         std::vector<Double_t> fPosXMin = fHoughSpaceCircle_buff->GetPosXMin();
         std::vector<Double_t> fPosYMin = fHoughSpaceCircle_buff->GetPosYMin();
         std::vector<Double_t> fPosZMin = fHoughSpaceCircle_buff->GetPosZMin();

         std::vector<Double_t> fPosXExp = fHoughSpaceCircle_buff->GetPosXExp();
         std::vector<Double_t> fPosYExp = fHoughSpaceCircle_buff->GetPosYExp();

         std::vector<Double_t> fPosXInt = fHoughSpaceCircle_buff->GetPosXInt();
         std::vector<Double_t> fPosYInt = fHoughSpaceCircle_buff->GetPosYInt();
         std::vector<Double_t> fPosZInt = fHoughSpaceCircle_buff->GetPosZInt();

         std::vector<Double_t> fPosXBack = fHoughSpaceCircle_buff->GetPosXBack();
         std::vector<Double_t> fPosYBack = fHoughSpaceCircle_buff->GetPosYBack();
         std::vector<Double_t> fPosZBack = fHoughSpaceCircle_buff->GetPosZBack();

         for (Int_t i = 0; i < fPosXMin.size(); i++) {
            fMC_XY->SetPoint(fMC_XY->GetN(), fPosXMin.at(i), fPosYMin.at(i));
            fMC_ZX->SetPoint(fMC_ZX->GetN(), fPosZMin.at(i), fPosXMin.at(i));
            fMC_ZY->SetPoint(fMC_ZY->GetN(), fPosZMin.at(i), fPosYMin.at(i));
         }

         for (Int_t i = 0; i < fPosXExp.size(); i++) {
            fMC_XY_exp->SetPoint(fMC_XY_exp->GetN(), fPosXExp.at(i), fPosYExp.at(i));
         }

         for (Int_t i = 0; i < fPosXInt.size(); i++) {
            fMC_XY_int->SetPoint(fMC_XY_int->GetN(), fPosXInt.at(i), fPosYInt.at(i));
            fMC_ZX_int->SetPoint(fMC_ZX_int->GetN(), fPosZInt.at(i), fPosXInt.at(i));
            fMC_ZY_int->SetPoint(fMC_ZY_int->GetN(), fPosZInt.at(i), fPosYInt.at(i));
         }

         for (Int_t i = 0; i < fPosXBack.size(); i++) {
            fMC_XY_back->SetPoint(fMC_XY_back->GetN(), fPosXBack.at(i), fPosYBack.at(i));
            fMC_ZX_back->SetPoint(fMC_ZX_back->GetN(), fPosZBack.at(i), fPosXBack.at(i));
            fMC_ZY_back->SetPoint(fMC_ZY_back->GetN(), fPosZBack.at(i), fPosYBack.at(i));
         }

         std::cout << cGREEN << "  = Initial conditions for MC : " << std::endl;
         std::cout << "  Theta                 : " << fHoughSpaceCircle_buff->GetIniTheta() * 180 / TMath::Pi()
                   << std::endl;
         std::cout << "  Phi                   : " << fHoughSpaceCircle_buff->GetIniPhi() * 180 / TMath::Pi()
                   << std::endl;
         std::cout << "  Radius                : " << fHoughSpaceCircle_buff->GetIniRadius() << std::endl;
         std::cout << "  Time Bucket           : " << fIniHit->GetTimeStamp() << std::endl;
         std::cout << "  Time Bucket RANSAC    : " << fIniHitRansac->GetTimeStamp() << std::endl;
         std::cout << "  Radius x Phi          : "
                   << fHoughSpaceCircle_buff->GetIniPhi() * fHoughSpaceCircle_buff->GetIniRadius() << cNORMAL
                   << std::endl;

         std::cout << cGREEN << "  = MC results : " << std::endl;
         std::cout << "  Theta          : " << fHoughSpaceCircle_buff->FitParameters.sThetaMin * 180 / TMath::Pi()
                   << std::endl;
         std::cout << "  Phi            : " << fHoughSpaceCircle_buff->FitParameters.sPhiMin * 180 / TMath::Pi()
                   << std::endl;
         std::cout << "  Energy         : " << fHoughSpaceCircle_buff->FitParameters.sEnerMin << std::endl;
         std::cout << "  Brho           : " << fHoughSpaceCircle_buff->FitParameters.sBrhoMin << std::endl;
         std::cout << "  Magnetic field : " << fHoughSpaceCircle_buff->FitParameters.sBMin << std::endl;
         std::cout << "  Norm. Chi-2    : " << fHoughSpaceCircle_buff->FitParameters.sNormChi2 << cNORMAL << std::endl;

      } else if (fIsLinearHough) {
         fHoughSpace = fHoughSpaceLine_buff->GetHoughSpace("XY");
         std::vector<std::pair<Double_t, Double_t>> LinearHoughPar = fHoughSpaceLine_buff->GetHoughPar();
         std::vector<Double_t> LinearHoughMax = fHoughSpaceLine_buff->GetHoughMax();
         TVector3 Vertex_1 = fHoughSpaceLine_buff->GetVertex1();
         TVector3 Vertex_2 = fHoughSpaceLine_buff->GetVertex2();
         // std::vector<AtTrack> TrackCand = fHoughSpaceLine_buff->GetTrackCand();

         std::cout << std::endl;
         std::cout << cGREEN << "  = Number of lines found by Linear Hough Space : " << LinearHoughPar.size()
                   << std::endl;

         // int n = 1000;
         // double t0 = 0;
         // double dt = 1000;

         /*if(TrackCand.size()>0)
         {

           for(Int_t i=0;i<TrackCand.size();i++){
           AtTrack track = TrackCand.at(i);
           std::vector<Double_t> parFit = track.GetFitPar();
           //std::cout<<cRED<<parFit[0]<<" "<<parFit[1]<<" "<<parFit[2]<<" "<<parFit[3]<<cNORMAL<<std::endl;
           TEveLine* line = new TEveLine(n);
           for (int i = 0; i <n;++i) {
           double t = t0+ dt*i/n;
           double x,y,z;
           SetLine(t,parFit,x,y,z);
           line->SetNextPoint(x, y, z);
           //l->SetPoint(i,x,y,z);
           //std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
           }
           line->SetMainColor(kRed);
           //l->Draw("same");
           fLineArray.push_back(fLine);

           }

           }*/

         for (Int_t i = 0; i < LinearHoughPar.size(); i++) {
            std::cout << cYELLOW << "  Hough Maximum " << i << "  : " << std::endl;
            std::cout << cYELLOW << "  Hough Angle : " << LinearHoughPar.at(i).first << std::endl;
            std::cout << cYELLOW << "  Hough Distance : " << LinearHoughPar.at(i).second << std::endl;
            std::cout << cYELLOW << "  Maximum Bin Content : " << LinearHoughMax.at(i) << cNORMAL << std::endl;
            std::cout << cYELLOW << "  Vertex 1 -  X : " << Vertex_1.X() << "   Y : " << Vertex_1.Y()
                      << "  Z : " << Vertex_1.Z() << cNORMAL << std::endl;
            std::cout << cYELLOW << "  Vertex 2 -  X : " << Vertex_2.X() << "   Y : " << Vertex_2.Y()
                      << "  Z : " << Vertex_2.Z() << cNORMAL << std::endl;
            std::cout << std::endl;
         }
      }

      // Test to see if the histograms are there
      /*TCanvas *test = new TCanvas();
       test->Divide(2,3);
       test->cd(1);
       fQuadrant1->Draw("zcol");
       test->cd(2);
       fQuadrant2->Draw("zcol");
       test->cd(3);
       fQuadrant3->Draw("zcol");
       test->cd(4);
       fQuadrant4->Draw("zcol");
       test->cd(5);
       test->Modified();
       test->Update();*/

   } else {
      fHoughSpace = new TH2F();
      // fQuadrant1 = new TH2F();
      // fQuadrant2 = new TH2F();
      // fQuadrant3 = new TH2F();
      // fQuadrant4 = new TH2F();
   }
}

void AtEventDrawTask::DrawMeshSpace() {}

void AtEventDrawTask::Reset()
{
   if (fHitSet) {
      fHitSet->Reset();
      gEve->RemoveElement(fHitSet, fEventManager);
   }

   if (fhitBoxSet) {
      fhitBoxSet->Reset();
      gEve->RemoveElement(fhitBoxSet, fEventManager);
   }

   if (fEventManager->GetDrawHoughSpace()) {

      if (fIsCircularHough) {
         if (fHitSetMin) {
            fHitSetMin->Reset();
            gEve->RemoveElement(fHitSetMin, fEventManager);
         }
      }

      else if (fIsLinearHough || fRansacArray) {

         /*if(fLine){
          fLine->Reset();
          gEve -> RemoveElement(fLine,fEventManager);
          }*/
         if (fVertex) {
            // fVertex->Reset();
            gEve->RemoveElement(fVertex, fEventManager);
            fVertex = nullptr;
         }

         if (fLineNum > 0) {
            for (Int_t i = 0; i < fLineNum; i++) {
               if (fLineArray[i]) {
                  gEve->RemoveElement(fLineArray[i], fEventManager);
               }
            }
         }
      }

      if (fPatternEventArray) {

         if (fLineNum > 0) {
            for (Int_t i = 0; i < fLineNum; i++) {
               if (fHitSetTFHC[i]) {
                  fHitSetTFHC[i]->Reset();
                  gEve->RemoveElement(fHitSetTFHC[i], fEventManager);
               }
               if (fHitClusterSet[i]) {
                  fHitClusterSet[i]->Reset();
                  gEve->RemoveElement(fHitClusterSet[i], fEventManager);
               }
            }
         }
      }

      if (fTrackingEventAnaArray) {

         for (Int_t i = 0; i < fTrackNum; i++) {
            if (fHitSetMC[i]) {
               fHitSetMC[i]->Reset();
               gEve->RemoveElement(fHitSetMC[i], fEventManager);
            }
         }
      }

   } // Draw Minimization

   /* TEveGeoShape* hitShape;
    if(hitSphereArray.size()>0)
    for(Int_t i=0;i<hitSphereArray.size();i++){
    hitShape = hitSphereArray[i];
    gEve->RemoveElement(hitShape,fEventManager);
    }*/

   // hitSphereArray.clear();

   /*if(fHitClusterSet) {
    fHitClusterSet->Reset();
    gEve->RemoveElement(fHitClusterSet, fEventManager);
    }*/

   /* Int_t nRiemannTracks = fRiemannSetArray.size();
    TEvePointSet* pointSet;
    if(nRiemannTracks!=0) {
    for(Int_t i=0; i<nRiemannTracks; i++){
    pointSet = fRiemannSetArray[i];
    gEve->RemoveElement(pointSet, fEventManager);
    }
    fRiemannSetArray.clear();
    }*/

   if (fPadPlane != NULL)
      fPadPlane->Reset(0);
}

/*void
 AtEventDrawTask::Set2DPlotRange(Int_t uaIdx)
 {
 if(uaIdx%100<0 || uaIdx%100>11 || uaIdx/100<0 || uaIdx/100>3)
 {
 fLogger->Error(MESSAGE_ORIGIN,
 "2DPlotRange should be ABB ( A = [0, 3], BB = [00, 11] )!");
 return;
 }

 fMinZ = (uaIdx/100)*12*7*4;
 fMaxZ = (uaIdx/100 + 1)*12*7*4;
 fMinX = (uaIdx%100)*8*9 - 432;
 fMaxX = (uaIdx%100 + 1)*8*9 - 432;

 fIs2DPlotRange = kTRUE;
 }*/

void AtEventDrawTask::DrawPadPlane()
{
   // fAtMapPtr = new AtTpcMap();
   fAtMapPtr = fDetmap;
   if (fPadPlane) {
      fPadPlane->Reset(0);
      return;
   }

   fAtMapPtr->GenerateAtTpc();
   // fAtMapPtr->SetGUIMode();// This method does not need to be called since it generates the Canvas we do not want
   fPadPlane = fAtMapPtr->GetAtTpcPlane();
   fCvsPadPlane->cd();
   // fPadPlane -> Draw("COLZ L0"); //0  == bin lines adre not drawn
   fPadPlane->Draw("COL L0");
   fPadPlane->SetMinimum(1.0);
   gStyle->SetOptStat(0);
   gStyle->SetPalette(103);
   gPad->Update();

   /*  fPadPlanePal
    = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");




    if(fPadPlanePal){

    Int_t cHit = fPadPlanePal->GetValueColor(30.0);
    TColor *hitBoxColor = gROOT->GetColor(cHit);
    Float_t xrgb,yrgb,zrgb;
    std::cout<<" xrgb : "<<xrgb<<std::endl;
    hitBoxColor->GetRGB(xrgb,yrgb,zrgb);


    }*/
}

void AtEventDrawTask::DrawPadWave()
{

   /*  if(fPadWave)
    {
    fPadWave->Reset(0);
    return;
    }
    **/
   fPadWave = new TH1I("fPadWave", "fPadWave", 512, 0, 511);
   gROOT->GetListOfSpecials()->Add(fPadWave);
   fCvsPadWave->cd();
   fPadWave->Draw();
}

void AtEventDrawTask::DrawPadAll()
{

   fCvsPadAll->cd();

   std::cout << "Starting to draw pads" << std::endl;
   for (Int_t i = 0; i < fNumPads; i++) {
      fPadAll[i]->GetYaxis()->SetRangeUser(0, 2500);
      // TODO: make it pad number independent / retrieve the quadrant info
      fPadAll[i]->Draw("SAME");
   }
   std::cout << "Finished drawing." << std::endl;
}

void AtEventDrawTask::DrawQEvent()
{

   fQEventHist = new TH1D("fQEventHist", "fQEventHist", 300, 0., 2000000.);
   fQEventHist_H = new TH1D("fQEventHist_H", "fQEventHist_H", 300, 0., 2000000.);
   fQEventHist_H->SetLineColor(kRed);
   fQEventHist_H->SetFillStyle(1);
   fCvsQEvent->cd();
   fQEventHist->Draw();
   fQEventHist_H->Draw("SAMES");
}

void AtEventDrawTask::DrawRhoVariance()
{

   fCvsRhoVariance->cd();
   fRhoVariance = new TH1D("fRhoVariance", "fRhoVariance", 4000, 0., 1000000.);
   fRhoVariance->Draw();
   fRhoVariance->SetLineColor(kRed);
}

void AtEventDrawTask::DrawHoughSpace()
{
   fCvsHoughSpace->cd();
   fHoughSpace = new TH2F("HistHoughXY", "HistHoughXY", 100, 0, 3.15, 500, -1000, 1000);
   fHoughSpace->Draw("colz");
}

void AtEventDrawTask::DrawPhiReco()
{
   fCvsPhi->cd();
   // fPhiDistr = new TH1D("PhiDist","PhiDist",90.0,0.0,90.0);
   for (Int_t i = 0; i < 5; i++) {
      fPhiDistr[i]->Draw("SAME");
   }
}

void AtEventDrawTask::DrawMesh()
{

   fCvsMesh->cd();
   fMesh = new TH1F("Mesh", "Mesh", 512, 0, 511);
   fMesh->Draw();
}

void AtEventDrawTask::Draw3DHist()
{

   fCvs3DHist->cd();
   f3DHist = new TH3F("gl3DHist", "gl3DHist", 50, -25.0, 25.0, 50, -25.0, 25.0, 50, 0, 512);
   gStyle->SetPalette(55);
   // gStyle->SetCanvasPreferGL(kTRUE);

   f3DHist->SetFillColor(2);
   f3DHist->Draw("box");
   // f3DHist -> Draw("glbox3");
   // f3DHist -> Draw("glcol"); //TODO: Not working, strange behavior
}

void AtEventDrawTask::DrawRad()
{

   fCvsRad->cd();
   fRadVSTb = new TH2F("RadVSTb", "RadVSTb", 100, 0, 512, 100, 0, 250);
   fRadVSTb->SetMarkerStyle(22);
   fRadVSTb->SetMarkerColor(kRed);
   fRadVSTb->Draw();
}

void AtEventDrawTask::DrawTheta()
{

   fCvsTheta->cd();
   // fTheta = new TH1F("Theta","Theta",512,0,511);
   fTheta = new TH2F("Theta", "Theta", 512, 0, 511, 500, 0, 2.0);
   fTheta->SetMarkerStyle(22);
   fTheta->SetMarkerColor(kRed);
   // gStyle->SetErrorX(0);
   fTheta->Draw("");
}

void AtEventDrawTask::DrawThetaxPhi()
{

   fCvsThetaxPhi->cd();
   fThetaxPhi = new TH2F("ThetaxPhi", "ThetaxPhi", 512, 0, 511, 100, -1000, 1000);
   fThetaxPhi->SetMarkerStyle(22);
   fThetaxPhi->SetMarkerColor(kRed);

   fThetaxPhi_Ini_RANSAC = new TH2F("ThetaxPhi_Ini_RANSAC", "ThetaxPhi_Ini_RANSAC", 512, 0, 511, 100, -1000, 1000);
   fThetaxPhi_Ini_RANSAC->SetMarkerStyle(20);
   fThetaxPhi_Ini_RANSAC->SetMarkerColor(kGreen);

   fThetaxPhi_Ini = new TH2F("ThetaxPhi_Ini", "ThetaxPhi_Ini", 512, 0, 511, 100, -1000, 1000);
   fThetaxPhi_Ini->SetMarkerStyle(23);
   fThetaxPhi_Ini->SetMarkerColor(kBlue);
   fThetaxPhi_Ini->SetMarkerSize(1.0);

   fThetaxPhi->Draw("");
   fThetaxPhi_Ini->Draw("SAMES");
   fThetaxPhi_Ini_RANSAC->Draw("SAMES");

   fHoughLinearFit->Draw("SAMES");
}

void AtEventDrawTask::DrawMC()
{

   fCvsMC_XY->cd();

   fMC_XY_exp = new TGraph();
   fMC_XY_exp->SetPoint(1, 0, 0);
   fMC_XY_exp->SetMarkerStyle(20);
   fMC_XY_exp->SetMarkerSize(1.0);
   fMC_XY_exp->SetMarkerColor(kBlack);
   fMC_XY_exp->Draw("AP");

   fMC_XY = new TGraph();
   fMC_XY->SetPoint(1, 0, 0);
   fMC_XY->SetMarkerStyle(20);
   fMC_XY->SetMarkerSize(1.0);
   fMC_XY->SetMarkerColor(kRed);
   fMC_XY->Draw("P");

   fMC_XY_back = new TGraph();
   fMC_XY_back->SetPoint(1, 0, 0);
   fMC_XY_back->SetMarkerStyle(22);
   fMC_XY_back->SetMarkerSize(1.0);
   fMC_XY_back->SetMarkerColor(6);
   fMC_XY_back->Draw("P");

   fMC_XY_int = new TGraph();
   fMC_XY_int->SetPoint(1, 0, 0);
   fMC_XY_int->SetMarkerStyle(22);
   fMC_XY_int->SetMarkerSize(1.0);
   fMC_XY_int->SetMarkerColor(8);
   fMC_XY_int->Draw("P");

   fCvsMC_Z->cd();

   fMC_ZX = new TGraph();
   fMC_ZX->SetPoint(1, 0, 0);
   fMC_ZX->SetMarkerStyle(20);
   fMC_ZX->SetMarkerSize(1.0);
   fMC_ZX->SetMarkerColor(kRed);
   fMC_ZX->Draw("AP");

   fMC_ZY = new TGraph();
   fMC_ZY->SetPoint(1, 0, 0);
   fMC_ZY->SetMarkerStyle(20);
   fMC_ZY->SetMarkerSize(1.0);
   fMC_ZY->SetMarkerColor(kBlack);
   fMC_ZY->Draw("P");

   fMC_ZX_back = new TGraph();
   fMC_ZX_back->SetPoint(1, 0, 0);
   fMC_ZX_back->SetMarkerStyle(22);
   fMC_ZX_back->SetMarkerSize(1.0);
   fMC_ZX_back->SetMarkerColor(6);
   fMC_ZX_back->Draw("P");

   fMC_ZY_back = new TGraph();
   fMC_ZY_back->SetPoint(1, 0, 0);
   fMC_ZY_back->SetMarkerStyle(22);
   fMC_ZY_back->SetMarkerSize(1.0);
   fMC_ZY_back->SetMarkerColor(6);
   fMC_ZY_back->Draw("P");

   fMC_ZX_int = new TGraph();
   fMC_ZX_int->SetPoint(1, 0, 0);
   fMC_ZX_int->SetMarkerStyle(22);
   fMC_ZX_int->SetMarkerSize(1.0);
   fMC_ZX_int->SetMarkerColor(kRed);
   fMC_ZX_int->Draw("P");

   fMC_ZY_int = new TGraph();
   fMC_ZY_int->SetPoint(1, 0, 0);
   fMC_ZY_int->SetMarkerStyle(22);
   fMC_ZY_int->SetMarkerSize(1.0);
   fMC_ZY_int->SetMarkerColor(kBlack);
   fMC_ZY_int->Draw("P");
}
void AtEventDrawTask::DrawAux()
{
   fCvsAux->Divide(3, 3);
   for (int i = 0; i < 9; ++i) {
      fAuxChannels[i] = new TH1F(Form("AuxCh%i", i), Form("AuxChannel%i", i), 512, 0, 511);
      fCvsAux->cd(i + 1);
      fAuxChannels[i]->Draw();
   }
}

void AtEventDrawTask::UpdateCvsAux()
{

   TPad *Pad_1 = (TPad *)fCvsAux->GetPad(1);
   Pad_1->Modified();
   Pad_1->Update();
   TPad *Pad_2 = (TPad *)fCvsAux->GetPad(2);
   Pad_2->Modified();
   Pad_2->Update();
   TPad *Pad_3 = (TPad *)fCvsAux->GetPad(3);
   Pad_3->Modified();
   Pad_3->Update();
   TPad *Pad_4 = (TPad *)fCvsAux->GetPad(4);
   Pad_4->Modified();
   Pad_4->Update();
   TPad *Pad_5 = (TPad *)fCvsAux->GetPad(5);
   Pad_5->Modified();
   Pad_5->Update();
   TPad *Pad_6 = (TPad *)fCvsAux->GetPad(6);
   Pad_6->Modified();
   Pad_6->Update();
   TPad *Pad_7 = (TPad *)fCvsAux->GetPad(7);
   Pad_7->Modified();
   Pad_7->Update();
   TPad *Pad_8 = (TPad *)fCvsAux->GetPad(8);
   Pad_8->Modified();
   Pad_8->Update();
   TPad *Pad_9 = (TPad *)fCvsAux->GetPad(9);
   Pad_9->Modified();
   Pad_9->Update();
   fCvsAux->Modified();
   fCvsAux->Update();
}
void AtEventDrawTask::UpdateCvsPadPlane()
{
   fHoughSpace->Draw("colz");
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

void AtEventDrawTask::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();

   //  TPaletteAxis *paxis
   //  = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");
}

void AtEventDrawTask::UpdateCvsPadAll()
{
   fCvsPadAll->Modified();
   fCvsPadAll->Update();

   //  TPaletteAxis *paxis
   // = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");
}

void AtEventDrawTask::UpdateCvsQEvent()
{
   fCvsQEvent->Modified();
   fCvsQEvent->Update();
}

void AtEventDrawTask::UpdateCvsRhoVariance()
{
   fCvsRhoVariance->Modified();
   fCvsRhoVariance->Update();
}

void AtEventDrawTask::UpdateCvsHoughSpace()
{
   fCvsHoughSpace->Modified();
   fCvsHoughSpace->Update();
}

void AtEventDrawTask::UpdateCvsPhi()
{
   // if(fPhiDistr!=NULL)fPhiDistr->Draw();
   fCvsPhi->Modified();
   fCvsPhi->Update();
}

void AtEventDrawTask::UpdateCvsMesh()
{

   fCvsMesh->Modified();
   fCvsMesh->Update();
}

void AtEventDrawTask::UpdateCvs3DHist()
{

   fCvs3DHist->Modified();
   fCvs3DHist->Update();
}

void AtEventDrawTask::UpdateCvsRad()
{

   fCvsRad->Modified();
   fCvsRad->Update();
}

void AtEventDrawTask::UpdateCvsTheta()
{

   fCvsTheta->Modified();
   fCvsTheta->Update();
}

void AtEventDrawTask::UpdateCvsThetaxPhi()
{

   fCvsThetaxPhi->Modified();
   fCvsThetaxPhi->Update();
}

void AtEventDrawTask::UpdateCvsQuadrants()
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

void AtEventDrawTask::UpdateCvsMC()
{
   fMC_XY_exp->GetXaxis()->SetRangeUser(-300.0, 300);
   fMC_XY_exp->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_XY->GetXaxis()->SetRangeUser(-300.0, 300);
   fMC_XY->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_XY_int->GetXaxis()->SetRangeUser(-300.0, 300);
   fMC_XY_int->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_XY_back->GetXaxis()->SetRangeUser(-300.0, 300);
   fMC_XY_back->GetYaxis()->SetRangeUser(-300.0, 300);
   fCvsMC_XY->Modified();
   fCvsMC_XY->Update();

   fMC_ZX_int->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZX_int->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_ZY_int->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZY_int->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_ZX->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZX->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_ZY->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZY->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_ZX_back->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZX_back->GetYaxis()->SetRangeUser(-300.0, 300);
   fMC_ZY_back->GetXaxis()->SetRangeUser(0, 1000);
   fMC_ZY_back->GetYaxis()->SetRangeUser(-300.0, 300);
   fCvsMC_Z->Modified();
   fCvsMC_Z->Update();
}

void AtEventDrawTask::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

/*void
 AtEventDrawTask::SetHitClusterAttributes(Color_t color, Size_t size, Style_t style)
 {
 fHitClusterColor = color;
 fHitClusterSize = size;
 fHitClusterStyle = style;
 }*/

/*void
 AtEventDrawTask::SetRiemannAttributes(Color_t color, Size_t size, Style_t style)
 {
 fRiemannColor = color;
 fRiemannSize = size;
 fRiemannStyle = style;
 }*/

void AtEventDrawTask::SelectPad(const char *rawevt)
{

   try {
      int event = gPad->GetEvent();
      if (event != 11)
         return; // may be comment this line
      TObject *select = gPad->GetSelected();
      if (!select)
         return;
      if (select->InheritsFrom(TH2Poly::Class())) {
         TH2Poly *h = (TH2Poly *)select;
         gPad->GetCanvas()->FeedbackMode(kTRUE);
         AtRawEvent *tRawEvent = NULL;
         tRawEvent = (AtRawEvent *)gROOT->GetListOfSpecials()->FindObject(rawevt);
         if (tRawEvent == NULL) {
            std::cout << " = AtEventDrawTask::SelectPad NULL pointer for the AtRawEvent! Please select an event first "
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

         AtMap *tmap = NULL;
         tmap = (AtMap *)gROOT->GetListOfSpecials()->FindObject("fMap");
         Int_t tPadNum = 0;

         if (dynamic_cast<AtTpcMap *>(tmap)) {
            tPadNum = dynamic_cast<AtTpcMap *>(tmap)->BinToPad(bin);
         } else if (dynamic_cast<AtGadgetIIMap *>(tmap)) {
            tPadNum = dynamic_cast<AtGadgetIIMap *>(tmap)->BinToPad(bin);
         } else if (dynamic_cast<AtSpecMATMap *>(tmap)) {
            tPadNum = dynamic_cast<AtSpecMATMap *>(tmap)->BinToPad(bin);
         }

         std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
         AtPad *tPad = tRawEvent->GetPad(tPadNum);

         if (tPad == nullptr)
            return;

         std::cout << " Event ID (Select Pad) : " << tRawEvent->GetEventID() << std::endl;
         std::cout << " Raw Event Pad Num " << tPad->GetPadNum() << std::endl;
         std::cout << std::endl;

         TH1I *tPadWave = NULL;
         tPadWave = (TH1I *)gROOT->GetListOfSpecials()->FindObject("fPadWave");
         Int_t *rawadc = tPad->GetRawADC();
         Double_t *adc = tPad->GetADC();
         if (tPadWave == NULL) {
            std::cout << " = AtEventDrawTask::SelectPad NULL pointer for the TH1I! Please enable SetPersistance for "
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

         TCanvas *tCvsPadWave = NULL;
         tCvsPadWave = (TCanvas *)gROOT->GetListOfSpecials()->FindObject("fCvsPadWave");
         if (tCvsPadWave == NULL) {
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

void AtEventDrawTask::DrawWave(Int_t PadNum)
{

   // Bool_t IsValid=kFALSE;
   // AtPad *pad = fRawevent->GetPad(0);
   // AtPad *pad= fRawevent->GetPad(PadNum,IsValid);
   // std::cout<<" Raw Event Pad Num "<<pad->GetPadNum()<<" Is Valid? : "<<IsValidPad<<std::endl;
}

void AtEventDrawTask::ResetPadAll()
{
   for (Int_t i = 0; i < fNumPads; i++) {
      fPadAll[i]->Reset(0);
   }
}

void AtEventDrawTask::ResetPhiDistr()
{

   for (Int_t i = 0; i < 5; i++) {
      fPhiDistr[i]->Reset(0);
   }
}

void AtEventDrawTask::Set3DHitStyleBar()
{
   f3DHitStyle = 0;
}

void AtEventDrawTask::Set3DHitStyleBox()
{
   f3DHitStyle = 1;
}

void AtEventDrawTask::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

void AtEventDrawTask::SetSaveTextData()
{
   fSaveTextData = kTRUE;
}

void AtEventDrawTask::SetLine(double t, std::vector<Double_t> p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = (p[0] + p[1] * t) / 10.0;
   y = (p[2] + p[3] * t) / 10.0;
   z = t / 10.0;
}

void AtEventDrawTask::SetLine6(double t, std::vector<Double_t> p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = (p[0] + p[1] * t) / 10.0;
   y = (p[2] + p[3] * t) / 10.0;
   z = (p[4] + p[5] * t) / 10.0;
}

EColor AtEventDrawTask::GetTrackColor(int i)
{
   std::vector<EColor> colors = {kAzure, kOrange, kViolet, kTeal, kMagenta, kBlue, kViolet, kYellow, kCyan, kAzure};
   if (i < 10) {
      return colors.at(i);
   } else
      return kAzure;
}

void AtEventDrawTask::SetRawEventBranch(TString branchName)
{
   fRawEventBranchName = branchName;
}

void AtEventDrawTask::SetEventBranch(TString branchName)
{
   fEventBranchName = branchName;
}
