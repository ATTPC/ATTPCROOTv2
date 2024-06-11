/**
 * @brief Event display task
 * @author JungWoo Lee (Korea Univ.)
 *         Adapted for AtTPCROOT by Yassid Ayyad (NSCL)
 */

#include "AtEventDrawTaskS800.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEvent.h"            // for AtEvent, hitVector
#include "AtEventManagerS800.h" // for AtEventManagerS800
#include "AtHit.h"              // for AtHit
#include "AtMap.h"              // for AtMap
#include "AtPad.h"              // for AtPad
#include "AtPatternEvent.h"     // for AtPatternEvent
#include "AtProtoEvent.h"       // for AtProtoEvent
#include "AtProtoQuadrant.h"    // for AtProtoQuadrant
#include "AtRawEvent.h"         // for AtRawEvent
#include "AtTpcMap.h"           // for AtTpcMap
#include "AtTpcProtoMap.h"      // for AtTpcProtoMap
#include "AtTrack.h"            // for AtTrack, operator<<
#include "AtTrackingEvent.h"    // for AtTrackingEvent

#include <FairLogger.h>      // for Logger, LOG
#include <FairRootManager.h> // for FairRootManager

#include <Math/Point3D.h>   // for PositionVector3D, Cartesian3D, opera...
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
#include <TEveTreeTools.h>  // for TEvePointSelectorConsumer, TEvePoint...
#include <TF1.h>            // for TF1
#include <TGraph.h>         // for TGraph
#include <TH1.h>            // for TH1D, TH1I, TH1F
#include <TH2.h>            // for TH2F
#include <TH2Poly.h>        // for TH2Poly
#include <TH3.h>            // for TH3F
#include <TList.h>          // for TList
#include <TNamed.h>         // for TNamed
#include <TObject.h>        // for TObject
#include <TPaletteAxis.h>   // for TPaletteAxis
#include <TROOT.h>          // for TROOT, gROOT
#include <TRandom.h>        // for TRandom
#include <TSeqCollection.h> // for TSeqCollection
#include <TString.h>        // for TString, Form, operator==, operator<<
#include <TStyle.h>         // for TStyle, gStyle
#include <TVirtualPad.h>    // for TVirtualPad, gPad
#include <TVirtualX.h>      // for TVirtualX

#include "S800Calc.h" // for S800Calc, CRDC, MultiHitTOF, IC

#include <algorithm> // for max
#include <array>     // for array
#include <cmath>     // for isnan, atan
#include <cstdio>    // for sprintf
#include <iostream>  // for cout
#include <memory>    // for allocator_traits<>::value_type
#include <vector>    // for vector, allocator

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

using namespace std;

ClassImp(AtEventDrawTaskS800);

AtEventDrawTaskS800::AtEventDrawTaskS800()
   : fIs2DPlotRange(kFALSE), fUnpackHough(kFALSE), fHitArray(nullptr),
     // fHitClusterArray(0),
     // fRiemannTrackArray(0),
     // fKalmanArray(0),
     fEventManager(nullptr), fRawevent(nullptr), fHoughSpaceArray(nullptr), fProtoEventArray(nullptr), fDetmap(nullptr),
     fThreshold(0), fHitSet(nullptr),
     // x(0),
     // hitSphereArray(0),
     fhitBoxSet(nullptr), fPadPlanePal(nullptr), fHitColor(kPink), fHitSize(1), fHitStyle(kFullDotMedium),
     // fHitClusterSet(0),
     // fHitClusterColor(kAzure-5),
     // fHitClusterColor(kYellow-5),
     // fHitClusterSize(1),
     // fHitClusterStyle(kOpenCircle),
     // fRiemannSetArray(0),
     // fRiemannColor(kBlue),
     // fRiemannSize(1.5),
     // fRiemannStyle(kOpenCircle),
     fCvsPadPlane(nullptr), fPadPlane(nullptr), fCvsPadWave(nullptr), fPadWave(nullptr), fCvsPadAll(nullptr),
     fCvsQEvent(nullptr), fQEventHist(nullptr), fQEventHist_H(nullptr), fCvsHoughSpace(nullptr), fHoughSpace(nullptr),
     fCvsRhoVariance(nullptr), fRhoVariance(nullptr), fCvsPhi(nullptr), fCvsMesh(nullptr), fMesh(nullptr),
     fCvs3DHist(nullptr), f3DHist(nullptr), fCvsRad(nullptr), fRadVSTb(nullptr), fCvsTheta(nullptr), fTheta(nullptr),
     fAtMapPtr(nullptr), fMinZ(0), fMaxZ(1344), fMinX(432), fMaxX(-432), f3DHitStyle(0), fMultiHit(0),
     fSaveTextData(false), f3DThreshold(0), fRANSACAlg(0), fCvsLvsTheta(nullptr), fLvsTheta(nullptr), fCvsPID(nullptr),
     fPID(nullptr), fCvsPID2(nullptr), fPID2(nullptr)

{

   // fAtMapPtr = new AtTpcMap();

   Char_t padhistname[256];

   /*
   for(Int_t i=0;i<300;i++){ // TODO: Full-scale must be accomodated
       sprintf(padhistname,"pad_%d",i);
       fPadAll[i] = new TH1I(padhistname,padhistname,512,0,511);

       // fPadAll[i] = NULL;
   }
   */

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
}

AtEventDrawTaskS800::~AtEventDrawTaskS800()
{

   // TODO Destroy pointers
   // for(Int_t i=0;i<hitSphereArray.size();i++) delete hitSphereArray[i];
   // delete x;
   // hitSphereArray.clear();
   delete fHoughLinearFit;
   delete fRansacLinearFit;
}

InitStatus AtEventDrawTaskS800::Init()
{

   TString option = fGeoOption;
   std::cout << " =====  AtEventDrawTaskS800::Init =====" << std::endl;
   std::cout << " =====  Current detector : " << option.Data() << std::endl;
   gROOT->Reset();
   FairRootManager *ioMan = FairRootManager::Instance();
   fEventManager = AtEventManagerS800::Instance();

   if (option == "Prototype") {

      fDetmap = new AtTpcProtoMap();
      // TString fMap = "/Users/yassidayyad/fair_install/AtTPCROOT_v2_06042015/scripts/proto.map"; //TODO Put it as
      // input of the run macro
      dynamic_cast<AtTpcProtoMap *>(fDetmap)->SetProtoMap(fMap.Data());
   } else {
      fDetmap = new AtTpcMap();
   }

   fDetmap->SetName("fMap");
   gROOT->GetListOfSpecials()->Add(fDetmap);

   fHitArray = dynamic_cast<TClonesArray *>(
      ioMan->GetObject("AtEventH")); // TODO: Why this confusing name? It should be fEventArray
   if (fHitArray)
      LOG(info) << cGREEN << "Hit Array Found." << cNORMAL;

   /*
   fRawEventArray = (TClonesArray*) ioMan->GetObject("AtRawEvent");
   if(fRawEventArray){
       LOG(info)<<cGREEN<<"Raw Event Array  Found."<<cNORMAL<<FairLogger::endl;
       fIsRawData=kTRUE;
   }
   */
   fHoughSpaceArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtHough"));
   if (fHoughSpaceArray)
      LOG(info) << cGREEN << "Hough Array Found." << cNORMAL;

   fProtoEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtProtoEvent"));
   if (fProtoEventArray)
      LOG(info) << cGREEN << "Prototype Event Array Found." << cNORMAL;

   fRansacArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtRansac"));
   if (fRansacArray)
      LOG(info) << cGREEN << "RANSAC Array Found." << cNORMAL;

   // fTrackFinderHCArray = (TClonesArray*) ioMan->GetObject("AtTrackFinderHC");
   // if(fTrackFinderHCArray)  LOG(info)<<cGREEN<<"Track Finder Hierarchical Clustering Array
   // Found."<<cNORMAL<<FairLogger::endl;

   fPatternEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtPatternEvent"));
   if (fPatternEventArray)
      LOG(info) << cGREEN << "Pattern Event Array Found." << cNORMAL;

   fTrackingEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject("AtTrackingEvent"));
   if (fTrackingEventArray)
      LOG(info) << cGREEN << "Tracking Event Analysis Array Found." << cNORMAL;

   fS800Calc = dynamic_cast<S800Calc *>(ioMan->GetObject("s800cal"));
   if (fS800Calc)
      LOG(info) << cGREEN << "S800Calc Found." << cNORMAL;
   // fS800CalcArray = (TClonesArray*) ioMan->GetObject("s800cal");
   // if(fS800CalcArray) LOG(info)<<cGREEN<<"S800Calc Array Found."<<cNORMAL<<FairLogger::endl;

   // gROOT->GetListOfSpecials()->Add(fRawEventArray);
   // fRawEventArray->SetName("AtRawEvent");

   gStyle->SetPalette(55);
   // fPhiDistr=NULL;
   // fCvsPadWave = fEventManager->GetCvsPadWave();
   // fCvsPadWave->SetName("fCvsPadWave");
   // gROOT->GetListOfSpecials()->Add(fCvsPadWave);
   // DrawPadWave();
   fCvsMesh = fEventManager->GetCvsMesh();
   DrawMesh();
   fCvsPadPlane = fEventManager->GetCvsPadPlane(); // There is a problem if the pad plane is drawn first
   DrawPadPlane();
   // fCvsPadPlane -> ToggleEventStatus();
   // fCvsPadPlane->AddExec("ex","AtEventDrawTaskS800::SelectPad(\"fRawEvent\")");
   // DrawPadPlane();
   // fCvsPadAll = fEventManager->GetCvsPadAll();
   // DrawPadAll();
   // fCvs3DHist = new TCanvas("glcvs3dhist");
   // fCvs3DHist = fEventManager->GetCvs3DHist();
   // Draw3DHist();
   // fCvsQEvent = new TCanvas("fCvsQEvent","fCvsQEvent");
   // DrawQEvent();
   // fCvsRhoVariance = new TCanvas("fCvsRhoVariance","fCvsRhoVariance");
   // DrawRhoVariance();
   // fCvsPhi = fEventManager->GetCvsPhi();
   // DrawPhiReco();

   // fCvsRad = fEventManager->GetCvsRad();
   // DrawRad();
   fCvsLvsTheta = fEventManager->GetCvsLvsTheta();
   DrawLvsTheta();
   fCvsPID = fEventManager->GetCvsPID();
   DrawPID();
   // DrawTEST();
   fCvsPID2 = fEventManager->GetCvsPID2();
   DrawPID2();

   // fCvsThetaxPhi = fEventManager->GetCvsThetaxPhi();
   // DrawThetaxPhi();
   // fCvsMC_XY = fEventManager->GetCvsMC_XY();
   // fCvsMC_Z = fEventManager->GetCvsMC_Z();
   // DrawMC();
   /*fCvsQuadrant1 = fEventManager->GetCvsQuadrant1();
    fCvsQuadrant2 = fEventManager->GetCvsQuadrant2();
    fCvsQuadrant3 = fEventManager->GetCvsQuadrant3();
    fCvsQuadrant4 = fEventManager->GetCvsQuadrant4();
    DrawHoughSpaceProto();*/

   //******* NO CALLS TO TCANVAS BELOW THIS ONE
   fCvsHoughSpace = fEventManager->GetCvsHoughSpace();
   DrawHoughSpace();

   return kSUCCESS;
}

void AtEventDrawTaskS800::Exec(Option_t *option)
{
   Reset();
   // ResetPadAll();
   ResetPhiDistr();

   if (fHitArray) {
      DrawHitPoints();
      DrawMeshSpace();
   }
   if (fProtoEventArray)
      DrawProtoSpace();
   if (fHoughSpaceArray && fUnpackHough)
      DrawHSpace();
   if (fS800Calc) {
      DrawS800();
   }

   gEve->Redraw3D(kFALSE);

   UpdateCvsPadPlane();
   // UpdateCvsPadWave();
   // UpdateCvsPadAll();
   // UpdateCvsQEvent();
   // UpdateCvsRhoVariance();
   // UpdateCvsPhi();
   UpdateCvsMesh();
   // UpdateCvs3DHist();
   if (fUnpackHough && fEventManager->GetDrawHoughSpace()) {
      UpdateCvsHoughSpace();
      // UpdateCvsRad();
      // UpdateCvsTheta();
      // UpdateCvsThetaxPhi();
      // UpdateCvsMC();
      // UpdateCvsQuadrants();
   }
}

void AtEventDrawTaskS800::DrawS800()
{

   // std::cout<<"draw func "<<fS800Calc->GetIC()->GetSum()<<std::endl;

   // fS800Calc = dynamic_cast<S800Calc*> (fS800CalcArray->At(0));
   if (fS800Calc->GetIsInCut()) {
      Double_t x0_corr_tof = 0.;
      Double_t afp_corr_tof = 0.;
      Double_t afp_corr_dE = 0.;
      Double_t x0_corr_dE = 0.;
      Double_t rf_offset = 0.0;
      Double_t corrGainE1up = 1;
      Double_t corrGainE1down = 1;
      Double_t ObjCorr1C1 = 100.;  // 70
      Double_t ObjCorr1C2 = 0.021; // 0.0085

      // Double_t S800_timeRf = fS800Calc->GetMultiHitTOF()->GetFirstRfHit();
      // Double_t S800_timeE1up = fS800Calc->GetMultiHitTOF()->GetFirstE1UpHit();
      // Double_t S800_timeE1down = fS800Calc->GetMultiHitTOF()->GetFirstE1DownHit();
      // Double_t S800_timeE1 = sqrt( (corrGainE1up*S800_timeE1up) * (corrGainE1down*S800_timeE1down) );
      // Double_t S800_timeXf = fS800Calc->GetMultiHitTOF()->GetFirstXfHit();
      // Double_t S800_timeObj = fS800Calc->GetMultiHitTOF()->GetFirstObjHit();

      //----------- New 10/01 -------------------------------------
      Int_t CondMTDCXfObj = 0;
      vector<Float_t> S800_timeMTDCObj = fS800Calc->GetMultiHitTOF()->GetMTDCObj();
      vector<Float_t> S800_timeMTDCXf = fS800Calc->GetMultiHitTOF()->GetMTDCXf();
      Float_t S800_timeObjSelect = -999;
      Float_t S800_timeXfSelect = -999;
      Float_t ObjCorr = -999;

      for (float k : S800_timeMTDCXf) {
         if (k > 140 && k < 230)
            S800_timeXfSelect = k;
      }
      for (float k : S800_timeMTDCObj) {
         if (k > -115 && k < -20)
            S800_timeObjSelect = k;
      }

      Double_t XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
      if (S800_timeXfSelect != -999 && S800_timeObjSelect != -999) {
         XfObj_tof = S800_timeXfSelect - S800_timeObjSelect;
         CondMTDCXfObj = 1;
      }
      Double_t S800_ICSum = fS800Calc->GetIC()->GetSum();
      //----------- New 10/01 -------------------------------------

      Double_t S800_x0 = fS800Calc->GetCRDC(0)->GetX();
      Double_t S800_x1 = fS800Calc->GetCRDC(1)->GetX();
      // Double_t S800_y0 = fS800Calc->GetCRDC(0)->GetY();
      // Double_t S800_y1 = fS800Calc->GetCRDC(1)->GetY();

      // Double_t S800_E1up = fS800Calc->GetSCINT(0)->GetDEup();
      // Double_t S800_E1down = fS800Calc->GetSCINT(0)->GetDEdown();

      // Double_t S800_tof = S800_timeObj - S800_timeE1;

      Double_t S800_afp = atan((S800_x1 - S800_x0) / 1073.);
      // Double_t S800_bfp = atan( (S800_y1-S800_y0)/1073. );
      // Double_t S800_tofCorr = S800_tof + x0_corr_tof*S800_x0 + afp_corr_tof*S800_afp;// - rf_offset;
      // Double_t S800_dE = fS800Calc->GetSCINT(0)->GetDE();//check if is this scint (0)
      // Double_t S800_dE = sqrt( (corrGainE1up*S800_E1up) * (corrGainE1down* S800_E1down ) );
      // Double_t S800_dECorr = S800_dE + afp_corr_dE*S800_afp + x0_corr_dE*fabs(S800_x0);

      if (CondMTDCXfObj && std::isnan(S800_ICSum) == 0 && std::isnan(S800_afp) == 0 && std::isnan(S800_x0) == 0)
         ObjCorr = S800_timeObjSelect + ObjCorr1C1 * S800_afp + ObjCorr1C2 * S800_x0;

      // std::cout<<"draw func in cut "<<S800_timeObjSelect<<" "<<XfObj_tof<<" "<<S800_ICSum<<std::endl;

      if (ObjCorr != -999)
         fPID->Fill(ObjCorr, XfObj_tof);
      if (ObjCorr != -999)
         fPID2->Fill(ObjCorr, S800_ICSum);
   }
}

void AtEventDrawTaskS800::DrawHitPoints()
{

   // std::cout<<"draw hit Points "<<fHitArray->At(0)<<std::endl;

   fMesh->Reset(nullptr);
   // f3DHist->Reset(0);
   TRandom r(0);

   std::ofstream dumpEvent;
   dumpEvent.open("event.dat");

   std::vector<Double_t> fPosXMin;
   std::vector<Double_t> fPosYMin;
   std::vector<Double_t> fPosZMin;

   // fQEventHist_H->Reset(0);
   auto *event = dynamic_cast<AtEvent *>(fHitArray->At(0)); // TODO: Why this confusing name? It should be fEventArray
   // event->SortHitArray(); // Works surprisingly well
   // Double_t Qevent=event->GetEventCharge();
   // Double_t RhoVariance=event->GetRhoVariance();
   auto MeshArray = event->GetMesh();
   Int_t eventID = event->GetEventID();
   //  std::ofstream dumpEvent;
   //  dumpEvent.open ("event.dat");
   TString TSevt = " Event ID : ";
   TString TSpad = " Pad ID : ";
   dumpEvent << TSevt << eventID << std::endl;

   if (fEventManager->GetEraseQEvent()) {
      // fQEventHist->Reset();
      // fRhoVariance->Reset();
   }

   // fQEventHist->Fill(Qevent);
   // fQEventHist_H->Fill(Qevent);
   // fRhoVariance->Fill(RhoVariance);

   for (Int_t i = 0; i < 512; i++) {

      fMesh->SetBinContent(i, MeshArray[i]);
   }

   if (fIsRawData) {
      fRawevent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));
      fRawevent->SetName("fRawEvent");
      gROOT->GetListOfSpecials()->Add(fRawevent);
   }

   // std::cout<<std::endl;
   // std::cout<<" AtHit Event ID : "<<event->GetEventID()<<std::endl;
   // std::cout<<" AtRawEvent Event ID : "<<rawevent->GetEventID()<<std::endl;
   // if(event->GetEventID()!=rawevent->GetEventID()) std::cout<<" = AtEventDrawTaskS800::DrawHitPoints : Warning,
   // EventID mismatch."<<std::endl;
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

      //  if(fIsLinearHough){
      // fLineArray.clear();
      for (auto &i : fLineArray)
         i = new TEveLine();
      int n = 100;
      double t0 = 0;
      double dt = 2000;
      std::vector<AtTrack> TrackCand;

      if (fRansacArray) {

      } else if (fPatternEventArray) {
         auto *patternEvent = dynamic_cast<AtPatternEvent *>(fPatternEventArray->At(0));
         TrackCand = patternEvent->GetTrackCand();
         for (auto &i : fHitSetTFHC)
            i = nullptr;

         if (TrackCand.size() < 10) {
            for (Int_t i = 0; i < TrackCand.size(); i++) {

               AtTrack track = TrackCand.at(i);
               std::vector<AtHit> trackHits = track.GetHitArrayObject();

               fHitSetTFHC[i] = new TEvePointSet(Form("HitMC_%d", i), nHitsMin, TEvePointSelectorConsumer::kTVT_XYZ);
               fHitSetTFHC[i]->SetMarkerColor(GetTrackColor(i) + 1);
               fHitSetTFHC[i]->SetMarkerSize(fHitSize);
               fHitSetTFHC[i]->SetMarkerStyle(fHitStyle);

               for (auto &trackHit : trackHits) {
                  auto position = trackHit.GetPosition();
                  fHitSetTFHC[i]->SetNextPoint(position.X() / 10., position.Y() / 10., position.Z() / 10.);
               }
            }
         }
      }

      if (fTrackingEventArray) {

         for (auto &i : fHitSetMC)
            i = nullptr;

         fTrackingEvent = dynamic_cast<AtTrackingEvent *>(fTrackingEventArray->At(0));
         std::vector<AtTrack> anaTracks = fTrackingEvent->GetTrackArray();
         std::cout << cRED << "Calling code for MC Minimization which is depricated!!!" << std::endl;
         std::cout << cYELLOW << "  ====   Tracking analysis ==== " << std::endl;
         std::cout << " Number of analyzed tracks : " << anaTracks.size() << std::endl;
         std::cout << " Vertex of reaction : " << fTrackingEvent->GetVertex() << std::endl;

         fTrackNum = anaTracks.size();

         if (anaTracks.size() < 5) { // Limited to 5 tracks
            for (Int_t i = 0; i < anaTracks.size(); i++) {
               AtTrack track = anaTracks.at(i);
               std::cout << track << std::endl;
               /*
       fPosXMin = track.GetPosXMin();
               fPosYMin = track.GetPosYMin();
               fPosZMin = track.GetPosZMin();
               nHitsMin = fPosXMin.size();
          */
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
   }

   //////////////////////////////////////////////

   fhitBoxSet = new TEveBoxSet("hitBox");
   fhitBoxSet->Reset(TEveBoxSet::kBT_AABox, kTRUE, 64);

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

      /*std::cout<<"  --------------------- "<<std::endl;
       std::cout<<" Hit : "<<iHit<<" AtHit Pad Number :  "<<PadNumHit<<" Pad Hit Mult : "<<PadMultHit<<" Pad Time Bucket
       : "<<hit.GetTimeStamp()<<" Hit X Position : "<<position.X()<<" Hit Y Position : "<<position.Y()<<" Hit Z Position
       : "<<position.Z()<<std::endl; std::cout<<"  Hit number : "<<iHit<<" - AtHit Pad Number :  "<<PadNumHit<<" - Hit
       Charge : "<<hit.GetCharge()<<" - Hit Base Correction : "<<BaseCorr<<std::endl;*/

      Bool_t fValidPad;

      /*
      if(fIsRawData){
          AtPad *RawPad = fRawevent->GetPad(PadNumHit,fValidPad);
          Double_t *adc = RawPad->GetADC();
          for(Int_t i=0;i<512;i++){

              f3DThreshold = fEventManager->Get3DThreshold();
              if(adc[i]>f3DThreshold) f3DHist->Fill(position.X()/10.,position.Y()/10.,i,adc[i]);

          }
      }
      */

      if (fSaveTextData) {
         if (!fEventManager->GetToggleCorrData())
            dumpEvent << position.X() << " " << position.Y() << " " << position.Z() << " " << hit.GetTimeStamp() << " "
                      << hit.GetCharge() << std::endl;
      }

      // std::cout<<"  Hit number : "<<iHit<<" - Position X : "<<position.X()<<" - Position Y : "<<position.Y()<<" -
      // Position Z : "<<position.Z()<<" - AtHit Pad Number :  "<<PadNumHit<<" - Pad bin :"<<Atbin<<" - Hit Charge :
      // "<<hit.GetCharge()<<std::endl;

      /*  x = new TEveGeoShape(Form("hitShape_%d",iHit));
       x->SetShape(new TGeoSphere(0, 0.1*hit.GetCharge()/300.));
       x->RefMainTrans().SetPos(position.X()/10.,
       position.Y()/10.,
       position.Z()/10.);
       hitSphereArray.push_back(x);*/

      // Float_t HitBoxYDim = TMath::Log(hit.GetCharge())*0.05;
      //   Float_t HitBoxYDim = hit.GetCharge()*0.001;
      //   Float_t HitBoxZDim = 0.05;
      //  Float_t HitBoxXDim = 0.05;

      // fhitBoxSet->AddBox(position.X()/10. - HitBoxXDim/2.0, position.Y()/10., position.Z()/10. - HitBoxZDim/2.0,
      //          HitBoxXDim,HitBoxYDim,HitBoxZDim); //This coordinates are x,y,z in our system

      //   Float_t xrgb=255,yrgb=0,zrgb=0;
      /*  if(fPadPlanePal){

       //  Int_t cHit = fPadPlanePal->GetValueColor();
       // Int_t cHit = 100;
       //TColor *hitBoxColor = gROOT->GetColor(cHit);
       //hitBoxColor->GetRGB(xrgb,yrgb,zrgb);

       std::cout<<" xrgb : "<<xrgb<<std::endl;
       std::cout<<" yrgb : "<<yrgb<<std::endl;
       std::cout<<" zrgb : "<<zrgb<<std::endl;

       }*/

      //  fhitBoxSet->DigitColor(xrgb,yrgb,zrgb, 0);
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

         for (Int_t iPad = 0; iPad < nPads; iPad++) {

            AtPad *fPad = fRawevent->GetPad(iPad);
            // std::cout<<"Pad num : "<<iPad<<" Is Valid? : "<<fPad->GetValidPad()<<" Pad num in pad object
            // :"<<fPad->GetPadNum()<<std::endl;
            auto rawadc = fPad->GetRawADC();
            auto adc = fPad->GetADC();
            // dumpEvent<<TSpad<<fPad->GetPadNum()<<std::endl;

            for (Int_t j = 0; j < 512; j++) { // TODO: This is limited to 256 pads only. Increment the size of the array
                                              // and put another option for AtTPC

               if (fPad->GetValidPad() && iPad < 256) {

                  fPadAll[iPad]->SetBinContent(j, adc[j]);
                  // if(fSaveTextData) dumpEvent<<adc[j]<<"     "<<j<<"     "<<fPad->GetPadNum()<<std::endl;
               }
            }

            // delete fPad;
            // fPad= NULL;
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
         // Lines plot together with data points
         gEve->AddElement(fHitSet);
         gEve->AddElement(fhitBoxSet);
         for (auto &w : fVVertex)
            gEve->AddElement(w);
         // if(fVertex) gEve -> AddElement(fVertex);
      }

      if (fPatternEventArray)
         if (fLineNum > 0)
            for (Int_t i = 0; i < fLineNum; i++)
               gEve->AddElement(fHitSetTFHC[i]);

      if (fTrackingEventArray)
         if (fTrackNum > 0 && fTrackNum < 5)
            for (Int_t i = 0; i < fTrackNum; i++)
               gEve->AddElement(fHitSetMC[i]);
   }

   dumpEvent.close();

   // gEve -> AddElement(fLine);
   // if(fLineArray.size()>0) gEve -> AddElement(fLineArray.at(0));
}

void AtEventDrawTaskS800::DrawHSpace() {}

void AtEventDrawTaskS800::DrawProtoSpace()
{
   auto *protoevent = dynamic_cast<AtProtoEvent *>(fProtoEventArray->At(0));
   Int_t nQuads = protoevent->GetNumQuadrants();
   std::vector<AtProtoQuadrant> quadrant;

   if (nQuads < 5) {
      for (Int_t iQ = 0; iQ < nQuads; iQ++) {

         // AtProtoQuadrant quadrant = protoevent->GetQuadrantArray()->at(iQ);
         quadrant.push_back(protoevent->GetQuadrantArray()->at(iQ));
         std::vector<Double_t> *PhiArray = quadrant[iQ].GetPhiArray();
         for (double pval : *PhiArray) {
            fPhiDistr[iQ]->Fill(pval);
         }
         PhiArray->clear();
      }
   }
}

void AtEventDrawTaskS800::DrawMeshSpace() {}

void AtEventDrawTaskS800::Reset()
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
         // one vertex
         /*if(fVertex)
         {
           //fVertex->Reset();
           gEve -> RemoveElement(fVertex, fEventManager);
      fVertex = nullptr;
   }*/
         // multiple vertex
         if (fVVertex.size() > 0) {
            // fVertex->Reset();
            for (auto &w : fVVertex)
               gEve->RemoveElement(w, fEventManager);
            fVertex = nullptr;
            fVVertex.clear();
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
               if (fHitSetMC[i]) {
                  gEve->RemoveElement(fHitSetTFHC[i], fEventManager);
               }
            }
         }
      }

      if (fTrackingEventArray) {

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

   if (fPadPlane != nullptr)
      fPadPlane->Reset(nullptr);
}

/*void
 AtEventDrawTaskS800::Set2DPlotRange(Int_t uaIdx)
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

void AtEventDrawTaskS800::DrawPadPlane()
{
   fAtMapPtr = new AtTpcMap();
   if (fPadPlane) {
      fPadPlane->Reset(nullptr);
      return;
   }

   dynamic_cast<AtTpcMap *>(fAtMapPtr)->GeneratePadPlane();
   // fAtMapPtr->SetGUIMode();// This method does not need to be called since it generates the Canvas we do not want
   fPadPlane = fAtMapPtr->GetPadPlane();
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

void AtEventDrawTaskS800::DrawPadWave()
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

void AtEventDrawTaskS800::DrawPadAll()
{

   fCvsPadAll->cd();
   TString option = fGeoOption;

   for (Int_t i = 0; i < 300; i++) {
      // fPadAll[i]->Reset(0);
      // fPadAll[i] = new TH1I("fPadAll","fPadAll",512,0,511);
      fPadAll[i]->GetYaxis()->SetRangeUser(0, 2500);
      // TODO: make it pad number independent / retrieve the quadrant info
      if (option == "Prototype") {
         if (i < 64)
            fPadAll[i]->SetLineColor(kPink - 3); // Q1, pink
         else if (i >= 64 && i < 127)
            fPadAll[i]->SetLineColor(kGreen + 2); // Q2, green
         else if (i >= 127 && i < 190)
            fPadAll[i]->SetLineColor(kBlue + 1); // Q3, blue
         else if (i >= 190 && i < 253)
            fPadAll[i]->SetLineColor(kOrange - 3); // Q4, orange
         else
            fPadAll[i]->SetLineColor(0); // white for non physical pads
      }
      fPadAll[i]->Draw("SAME");
   }
}

void AtEventDrawTaskS800::DrawQEvent()
{

   fQEventHist = new TH1D("fQEventHist", "fQEventHist", 300, 0., 2000000.);
   fQEventHist_H = new TH1D("fQEventHist_H", "fQEventHist_H", 300, 0., 2000000.);
   fQEventHist_H->SetLineColor(kRed);
   fQEventHist_H->SetFillStyle(1);
   fCvsQEvent->cd();
   fQEventHist->Draw();
   fQEventHist_H->Draw("SAMES");
}

void AtEventDrawTaskS800::DrawRhoVariance()
{

   fCvsRhoVariance->cd();
   fRhoVariance = new TH1D("fRhoVariance", "fRhoVariance", 4000, 0., 1000000.);
   fRhoVariance->Draw();
   fRhoVariance->SetLineColor(kRed);
}

void AtEventDrawTaskS800::DrawHoughSpace()
{
   fCvsHoughSpace->cd();
   fHoughSpace = new TH2F("HistHoughXY", "HistHoughXY", 100, 0, 3.15, 500, -1000, 1000);
   fHoughSpace->Draw("colz");
}

void AtEventDrawTaskS800::DrawHoughSpaceProto()
{
   // if(fIsLinearHough){ //
   fCvsQuadrant1->cd();
   fQuadrant1 = new TH2F("fQuadrant1", "fQuadrant1", 100, 0, 3.15, 500, -1000, 1000);
   fQuadrant1->Draw("zcol");
   fCvsQuadrant2->cd();
   fQuadrant2 = new TH2F("fQuadrant2", "fQuadrant2", 100, 0, 3.15, 500, -1000, 1000);
   fQuadrant2->Draw("zcol");
   fCvsQuadrant3->cd();
   fQuadrant3 = new TH2F("fQuadrant3", "fQuadrant3", 100, 0, 3.15, 500, -1000, 1000);
   fQuadrant3->Draw("zcol");
   fCvsQuadrant4->cd();
   fQuadrant4 = new TH2F("fQuadrant4", "fQuadrant4", 100, 0, 3.15, 500, -1000, 1000);
   fQuadrant4->Draw("zcol");
   //}
}

void AtEventDrawTaskS800::DrawPhiReco()
{
   fCvsPhi->cd();
   // fPhiDistr = new TH1D("PhiDist","PhiDist",90.0,0.0,90.0);
   for (auto &i : fPhiDistr) {
      i->Draw("SAME");
   }
}

void AtEventDrawTaskS800::DrawMesh()
{

   fCvsMesh->cd();
   fMesh = new TH1F("Mesh", "Mesh", 512, 0, 511);
   fMesh->Draw();
}

void AtEventDrawTaskS800::Draw3DHist()
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

void AtEventDrawTaskS800::DrawRad()
{

   fCvsRad->cd();
   fRadVSTb = new TH2F("RadVSTb", "RadVSTb", 100, 0, 512, 100, 0, 250);
   fRadVSTb->SetMarkerStyle(22);
   fRadVSTb->SetMarkerColor(kRed);
   fRadVSTb->Draw();
}

void AtEventDrawTaskS800::DrawTheta()
{

   fCvsTheta->cd();
   // fTheta = new TH1F("Theta","Theta",512,0,511);
   fTheta = new TH2F("Theta", "Theta", 512, 0, 511, 500, 0, 2.0);
   fTheta->SetMarkerStyle(22);
   fTheta->SetMarkerColor(kRed);
   // gStyle->SetErrorX(0);
   fTheta->Draw("");
}

void AtEventDrawTaskS800::DrawLvsTheta()
{

   fCvsLvsTheta->cd();
   fLvsTheta = new TH2F("LvsTheta", "LvsTheta", 180, 0, 180, 500, 0, 1030);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);
   fLvsTheta->Draw("colz");
}

void AtEventDrawTaskS800::DrawPID()
{

   fCvsPID->cd();
   // fPID = new TH2F("PID","PID",3000,-250,500,2000,0,500);
   fPID = new TH2F("PID", "PID", 500, -150, 50, 300, 230, 260);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);
   fPID->Draw("colz");
}

void AtEventDrawTaskS800::DrawPID2()
{

   fCvsPID2->cd();
   // fPID2 = new TH2F("PID2","PID2",3000,-250,500,2000,0,500);
   fPID2 = new TH2F("PID2", "PID2", 500, -150, 50, 1000, 1400, 2200);
   // fLvsTheta->SetMarkerStyle(22);
   // fLvsTheta->SetMarkerColor(kRed);
   fPID2->Draw("colz");
}

void AtEventDrawTaskS800::DrawThetaxPhi()
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

void AtEventDrawTaskS800::DrawMC()
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

void AtEventDrawTaskS800::UpdateCvsPadPlane()
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

void AtEventDrawTaskS800::UpdateCvsPadWave()
{
   fCvsPadWave->Modified();
   fCvsPadWave->Update();

   //  TPaletteAxis *paxis
   //  = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");
}

void AtEventDrawTaskS800::UpdateCvsPadAll()
{
   fCvsPadAll->Modified();
   fCvsPadAll->Update();

   //  TPaletteAxis *paxis
   // = (TPaletteAxis *) fPadPlane->GetListOfFunctions()->FindObject("palette");
}

void AtEventDrawTaskS800::UpdateCvsQEvent()
{
   fCvsQEvent->Modified();
   fCvsQEvent->Update();
}

void AtEventDrawTaskS800::UpdateCvsRhoVariance()
{
   fCvsRhoVariance->Modified();
   fCvsRhoVariance->Update();
}

void AtEventDrawTaskS800::UpdateCvsHoughSpace()
{
   fCvsHoughSpace->Modified();
   fCvsHoughSpace->Update();
}

void AtEventDrawTaskS800::UpdateCvsPhi()
{
   // if(fPhiDistr!=NULL)fPhiDistr->Draw();
   fCvsPhi->Modified();
   fCvsPhi->Update();
}

void AtEventDrawTaskS800::UpdateCvsMesh()
{

   fCvsMesh->Modified();
   fCvsMesh->Update();
}

void AtEventDrawTaskS800::UpdateCvs3DHist()
{

   fCvs3DHist->Modified();
   fCvs3DHist->Update();
}

void AtEventDrawTaskS800::UpdateCvsRad()
{

   fCvsRad->Modified();
   fCvsRad->Update();
}

void AtEventDrawTaskS800::UpdateCvsTheta()
{

   fCvsTheta->Modified();
   fCvsTheta->Update();
}

void AtEventDrawTaskS800::UpdateCvsLvsTheta()
{

   fCvsLvsTheta->Modified();
   fCvsLvsTheta->Update();
}

void AtEventDrawTaskS800::UpdateCvsPID()
{

   fCvsPID->Modified();
   fCvsPID->Update();
}

void AtEventDrawTaskS800::UpdateCvsPID2()
{

   fCvsPID2->Modified();
   fCvsPID2->Update();
}

void AtEventDrawTaskS800::UpdateCvsThetaxPhi()
{

   fCvsThetaxPhi->Modified();
   fCvsThetaxPhi->Update();
}

void AtEventDrawTaskS800::UpdateCvsQuadrants()
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

void AtEventDrawTaskS800::UpdateCvsMC()
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

void AtEventDrawTaskS800::SetHitAttributes(Color_t color, Size_t size, Style_t style)
{
   fHitColor = color;
   fHitSize = size;
   fHitStyle = style;
}

/*void
 AtEventDrawTaskS800::SetHitClusterAttributes(Color_t color, Size_t size, Style_t style)
 {
 fHitClusterColor = color;
 fHitClusterSize = size;
 fHitClusterStyle = style;
 }*/

/*void
 AtEventDrawTaskS800::SetRiemannAttributes(Color_t color, Size_t size, Style_t style)
 {
 fRiemannColor = color;
 fRiemannSize = size;
 fRiemannStyle = style;
 }*/

void AtEventDrawTaskS800::SelectPad(const char *rawevt)
{
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
         std::cout << " = AtEventDrawTaskS800::SelectPad NULL pointer for the AtRawEvent! Please select an event first "
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
      // new AtTpcProtoMap();
      // TString map = "/Users/yassidayyad/fair_install/AtTPCROOT_v2_06042015/scripts/proto.map";
      // tmap->SetProtoMap(map.Data());
      Int_t tPadNum = tmap->BinToPad(bin);
      std::cout << " Bin : " << bin << " to Pad : " << tPadNum << std::endl;
      AtPad *tPad = tRawEvent->GetPad(tPadNum);
      if (tPad == nullptr)
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
         std::cout << " = AtEventDrawTaskS800::SelectPad NULL pointer for the TH1I! Please enable SetPersistance for "
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
         std::cout << " = AtEventDrawTaskS800::SelectPad NULL pointer for the TCanvas! Please select an event first "
                   << std::endl;
         return;
      }
      tCvsPadWave->cd();
      tPadWave->Draw();
      // tPadWaveSub->Draw("SAME");
      tCvsPadWave->Update();
   }
}

void AtEventDrawTaskS800::DrawWave(Int_t PadNum)
{

   // Bool_t IsValid=kFALSE;
   // AtPad *pad = fRawevent->GetPad(0);
   // AtPad *pad= fRawevent->GetPad(PadNum,IsValid);
   // std::cout<<" Raw Event Pad Num "<<pad->GetPadNum()<<" Is Valid? : "<<IsValidPad<<std::endl;
}

void AtEventDrawTaskS800::ResetPadAll()
{

   for (auto &i : fPadAll) {
      i->Reset(nullptr);
   }
}

void AtEventDrawTaskS800::ResetPhiDistr()
{

   for (auto &i : fPhiDistr) {
      i->Reset(nullptr);
   }
}

void AtEventDrawTaskS800::Set3DHitStyleBar()
{
   f3DHitStyle = 0;
}

void AtEventDrawTaskS800::Set3DHitStyleBox()
{
   f3DHitStyle = 1;
}

void AtEventDrawTaskS800::SetMultiHit(Int_t hitMax)
{
   fMultiHit = hitMax;
}

void AtEventDrawTaskS800::SetSaveTextData()
{
   fSaveTextData = kTRUE;
}

void AtEventDrawTaskS800::SetLine(double t, std::vector<Double_t> p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = (p[0] + p[1] * t) / 10.0;
   y = (p[2] + p[3] * t) / 10.0;
   z = t / 10.0;
}

void AtEventDrawTaskS800::SetLine6(double t, std::vector<Double_t> p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = (p[0] + p[1] * t) / 10.0;
   y = (p[2] + p[3] * t) / 10.0;
   z = (p[4] + p[5] * t) / 10.0;
}

EColor AtEventDrawTaskS800::GetTrackColor(int i)
{
   std::vector<EColor> colors = {kAzure, kOrange, kViolet, kTeal, kMagenta, kBlue, kViolet, kYellow, kCyan, kAzure};
   if (i < 10) {
      return colors.at(i);
   } else
      return kAzure;
}
