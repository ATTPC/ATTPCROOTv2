#include "AtHoughSpaceLine.h"
//#include "AtHoughSpaceLine3D.h"
#include "TCanvas.h"
#include "Fit/Fitter.h"
#ifdef _OPENMP
#include <omp.h>
#endif

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtHoughSpaceLine)

   using namespace ROOT::Math;

AtHoughSpaceLine::AtHoughSpaceLine()
{

   fXbinRZ = 500.0;
   fYbinRZ = 500.0;
   // fHoughDist=5.0;
   HistHoughXZ = new TH2F("HistHoughXZ", "HistHoughXZ", 500, 0, 3.15, 500, 0, 300);
   HistHoughRZ = new TH2F("HistHoughRZ", "HistHoughRZ", fXbinRZ, 0, 3.15, fYbinRZ, -300, 300);
   fRadThreshold = 0.0;

   /*Char_t HoughQuadHistName[256];
   for(Int_t i=0;i<4;i++){
    sprintf(HoughQuadHistName,"HoughQuad_%d",i);
    HistHoughRZ[i] = new TH2F(HoughQuadHistName,HoughQuadHistName,500,0,3.15,2500,0,1000);
   }*/

   FairLogger *fLogger = FairLogger::GetLogger();
   AtDigiPar *fPar;

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";

   fZk = 500.0; // Prototype
   fDriftVelocity = fPar->GetDriftVelocity();
   fEntTB = (Int_t)fPar->GetTBEntrance();
   fTBTime = fPar->GetTBTime();
   // std::cout<<cRED<< fDriftVelocity<<"          "<<fTBTime<<cNORMAL<<std::endl;

   HoughPar.clear();
   HoughParSTD.clear();
   HoughMax.clear();

   fVertex_1.SetXYZ(-10000, -10000, -10000);
   fVertex_2.SetXYZ(-10000, -10000, -10000);
   fMinimum = -1.0;
   fLineDistThreshold = 3.0;
}

AtHoughSpaceLine::~AtHoughSpaceLine()
{
   delete HistHoughXZ;
   delete HistHoughRZ;
   // for(Int_t i=0;i<4;i++) delete HistHoughRZ[i];
}

std::vector<Double_t> AtHoughSpaceLine::GetHoughMax()
{
   return HoughMax;
}
TVector3 AtHoughSpaceLine::GetVertex1()
{
   return fVertex_1;
}
TVector3 AtHoughSpaceLine::GetVertex2()
{
   return fVertex_2;
}
TH2F *AtHoughSpaceLine::GetHoughSpace(TString ProjPlane)
{
   return HistHoughRZ;
}
Double_t AtHoughSpaceLine::GetMinimum()
{
   return fMinimum;
}
std::vector<AtTrack> AtHoughSpaceLine::GetTrackCand()
{
   return fTrackCand;
}
// TH2F* AtHoughSpaceLine::GetHoughQuadrant(Int_t index) {return HistHoughRZ[index];}

void AtHoughSpaceLine::SetRadiusThreshold(Float_t value)
{
   fRadThreshold = value;
}
void AtHoughSpaceLine::SetTrackCand(AtTrack *track)
{
   fTrackCand.push_back(*track);
}
// void AtHoughSpaceLine::SetTrackCandArray(std::vector<AtTrack> *TrackArray)          {fTrackCand = *TrackArray; }

std::vector<std::pair<Double_t, Double_t>> AtHoughSpaceLine::GetHoughPar(TString opt)
{
   if (opt.EqualTo("Hist"))
      return HoughPar;
   else if (opt.EqualTo("STD"))
      return HoughParSTD;

   return std::vector<std::pair<Double_t, Double_t>>();
}

void AtHoughSpaceLine::CalcHoughSpace(AtEvent *event, TH2Poly *hPadPlane) {}

void AtHoughSpaceLine::CalcHoughSpace(AtEvent *event) // Main function of the Linear Hough Space class
{

   /// Set Options here n(default is Generic hough Space calculation)
   // omp_set_num_threads(2);

   std::vector<AtTrack *> YZ_tracks;
   std::vector<AtTrack *> XZ_tracks;
   int threadId;

   // This is a working example of a parallelized block (class needs refactoring)
   //#pragma omp parallel private(threadId) shared(event)
   //{

   // threadId = omp_get_thread_num();
   // if(threadId==1) {YZ_tracks = CalcGenHoughSpace<AtEvent*,TString>(event,"YZ"); std::cout<<" Thread number :
   // "<<threadId<<std::endl;} if(threadId==0) {XZ_tracks = CalcGenHoughSpace<AtEvent*,TString>(event,"XZ");std::cout<<"
   // Thread number : "<<threadId<<std::endl;} std::cout<<" ============= "<<std::endl;

   //}

   YZ_tracks = CalcGenHoughSpace<AtEvent *, TString>(event, "YZ");
   HistHoughRZ->Reset();
   HoughMax.clear();
   HoughPar.clear();
   // XZ_tracks = CalcGenHoughSpace<AtEvent*,TString>(event,"XZ");

   (XZ_tracks.size() > YZ_tracks.size() ? fHoughTracks = XZ_tracks : fHoughTracks = YZ_tracks);

   if (fHoughTracks.size() > 1) { // Defined in CalcGenHoughSpace
      for (Int_t ntrack = 0; ntrack < fHoughTracks.size(); ntrack++)
         MinimizeTrack(fHoughTracks.at(ntrack));
      FindVertex(fHoughTracks);
   }

   //  Block for Hough Space in 3D
   /*Int_t nHits = event->GetNumHits();
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   cloud->reserve(nHits);
   std::vector<std::pair<vector3D, vector3D> > lines;
   AtHoughSpaceLine3D *hough3D = new AtHoughSpaceLine3D();
   for(Int_t i=0;i<nHits;i++){
     AtHit* hit = event->GetHit(i);
     TVector3 position = hit->GetPosition();
     pcl::PointXYZRGB p;
     p.x = position.X();
     p.y = position.Y();
     p.z = position.Z() - 500.0;
     p.r = 0;
     p.g = 0;
     p.b = 0;
     cloud->push_back(p);
     //std::cout<<"  p.x : "<<p.x<<" p.y : "<<p.y<<" p.z : "<<p.z<<std::endl;
   }

   hough3D->lineTransform3D_Tesselation(cloud,lines,10);

   std::cout<<" Lines found : "<<lines.size()<<std::endl;

   if(lines.size()>0){
     vector3D p, b;
   std::pair<vector3D, vector3D> _singleline_ = lines.at(0);
   p = _singleline_.first;
   b = _singleline_.second;

    TGraph2D * gr = new TGraph2D();

   int n = 1000;
   double t0 = 0;
   double dt = 400000;
   TPolyLine3D *l = new TPolyLine3D(n);
   for (int i = 0; i <n;++i) {
      double t = t0+ dt*i/n;
      double x,y,z;
      x = p.x + t*b.x;
      y = p.y + t*b.y;
      z = (p.z+500.0) + t*b.z;
      //std::cout<<p.x<<std::endl;

      //if( (x<1000 && x>-1000) && (y<1000 && y>-1000) &&  (z<1000 && z>-1000))
      gr->SetPoint(i,x,y,z);
      //std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
   }
   gr->SetLineColor(kRed);
   //l->Draw("same");
   gr->Draw("p0");
   //gr->GetXaxis()->SetLimits(-250,250);
   //gr->GetYaxis()->SetLimits(-250,250);
   //gr->GetZaxis()->SetLimits(-1000,1000);
   //gr->Draw("p0");
 }*/
}

void AtHoughSpaceLine::FindVertex(std::vector<AtTrack *> HoughTracks)
{

   // Assumes the minimum distance between two lines, with respect a given threshold, the first vertex candidate. Then
   // evaluates the distance of each remaining line with respect to the others (vertex) to decide the particles of the
   // reaction.

   Double_t mad = 999999; // Minimum approach distance
   XYZVector c_1(-1000, -1000, -1000);
   XYZVector c_2(-1000, -1000, -1000);
   // std::vector<AtTrack*> *TrackCand;

   // Current  parametrization
   // x = p[0] + p[1]*t;
   // y = p[2] + p[3]*t;
   // z = t;
   //  (x,y,z) = (p[0],p[2],0) + (p[1],p[3],1)*t

   // Test each line against the others to find a vertex candidate
   for (Int_t i = 0; i < fHoughTracks.size() - 1; i++) {

      AtTrack *track = fHoughTracks.at(i);
      track->SetTrackID(i);
      std::vector<Double_t> p = track->GetFitPar();

      if (p.size() > 0) {
         XYZVector L_0(p[0], p[2], 0.); // p1
         XYZVector L_1(p[1], p[3], 1.); // d1

         // std::cout<<" L_1 p[0] : "<<p[0]<<" L_1 p[2] : "<<p[2]<<std::endl;
         // std::cout<<" L_1 p[1] : "<<p[1]<<" L_1 p[3] : "<<p[3]<<std::endl;

         for (Int_t j = i + 1; j < fHoughTracks.size(); j++) {
            AtTrack *track_f = fHoughTracks.at(j);
            track_f->SetTrackID(j);
            std::vector<Double_t> p_f = track_f->GetFitPar();

            if (p_f.size() > 0) {
               XYZVector L_f0(p_f[0], p_f[2], 0.); // p2
               XYZVector L_f1(p_f[1], p_f[3], 1.); // d2

               // std::cout<<" L_f1 p_f[1] : "<<p_f[1]<<" L_f1 p_f[3] : "<<p_f[3]<<std::endl;

               XYZVector L = L_1.Cross(L_f1);
               Double_t L_mag = L.Rho();
               XYZVector n_1 = L_1.Cross(L);
               XYZVector n_2 = L_f1.Cross(L);
               c_1 = L_0 + ((L_f0 - L_0).Dot(n_2) * L_1) / (L_1.Dot(n_2));
               c_2 = L_f0 + ((L_0 - L_f0).Dot(n_1) * L_f1) / (L_f1.Dot(n_1));

               // std::cout<<i<<" "<<j<<" "<<L_mag<<std::endl;

               if (L_mag > 0) {
                  XYZVector n = L / (Double_t)L_mag;
                  Double_t d = TMath::Abs(n.Dot(L_0 - L_f0));
                  // std::cout<<" Distance of minimum approach : "<<d<<std::endl;

                  if (d < mad) {

                     mad = d;
                     // std::cout<<" New distance of minimum approach : "<<mad<<std::endl;
                     fVertex_1.SetXYZ(c_1.X(), c_1.Y(), c_1.Z());
                     fVertex_2.SetXYZ(c_2.X(), c_2.Y(), c_2.Z());
                     fMinimum = mad;
                     if (!CheckTrackID(track->GetTrackID(), fTrackCand))
                        fTrackCand.push_back(*track);
                     if (!CheckTrackID(track_f->GetTrackID(), fTrackCand))
                        fTrackCand.push_back(*track_f);
                  }

                  if (d < fLineDistThreshold) {

                     if (!CheckTrackID(track->GetTrackID(), fTrackCand)) {
                        // std::cout<<" Add track"<<track->GetTrackID()<<std::endl;
                        fTrackCand.push_back(*track);
                     }

                     if (!CheckTrackID(track_f->GetTrackID(), fTrackCand)) {
                        // std::cout<<" Add track f"<<track_f->GetTrackID()<<std::endl;
                        fTrackCand.push_back(*track_f);
                     }
                  }
               }

               // for(Int_t i=0;i<fTrackCand.size();i++)
               // std::cout<<fTrackCand.at(i).GetTrackID()<<std::endl;

            } // p_f size
         }    // End of track
      }       // p size
   }
}

void AtHoughSpaceLine::CalcHoughSpace(AtEvent *event, Bool_t YZplane, Bool_t XYplane, Bool_t XZplane)
{

   Int_t nHits = event->GetNumHits();

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit hit = event->GetHitArray()->at(iHit);
      Int_t PadNumHit = hit.GetHitPadNum();
      if (hit.GetCharge() < fThreshold)
         continue;
      TVector3 position = hit.GetPosition();

      for (Int_t itheta = 0; itheta < 1023; itheta++) {
         Float_t angle = TMath::Pi() * (static_cast<Float_t>(itheta) / 1023);
         if (XZplane) {
            Float_t d0_XZ = (TMath::Cos(angle) * position.X()) + (TMath::Sin(angle) * position.Z());
            HistHoughXZ->Fill(angle, d0_XZ);
         }
      }

   } // Hits loop
}

void AtHoughSpaceLine::CalcHoughSpace(AtProtoEvent *protoevent, Bool_t q1, Bool_t q2, Bool_t q3, Bool_t q4)
{
   Int_t nQuads = protoevent->GetNumQuadrants();
   std::vector<AtProtoQuadrant> quadrantArray;
   AtProtoQuadrant *quadrant;
   Double_t drift_cal = fDriftVelocity * fTBTime / 100.0; // mm

   TH2F *HistHoughRZInner[4]; // One per quadrant
   Char_t HoughQuadHistName[256];

   for (Int_t i = 0; i < 4; ++i) {
      sprintf(HoughQuadHistName, "HoughQuad_%d", i);
      // HistHoughRZInner[i] = new TH2F(HoughQuadHistName, HoughQuadHistName, 200, 0, 3.15, 1000, 0, 600);// Was (500,
      // 0, 3.15,....
      HistHoughRZInner[i] =
         new TH2F(HoughQuadHistName, HoughQuadHistName, 500, 0, 3.15, 1000, 0, 600); // Was (500, 0, 3.15,....
   }

   if (nQuads < 5) {
      for (Int_t iQ = 0; iQ < nQuads; ++iQ) {
         HoughMap.clear();
         // std::cout << " =========  Quadrant : " << iQ << std::endl;
         quadrant = &protoevent->GetQuadrantArray()->at(iQ);
         Int_t qNumHits = quadrant->GetNumHits();

         for (Int_t iHit = 0; iHit < qNumHits; ++iHit) {
            AtHit *hit = quadrant->GetHit(iHit);
            Int_t PadNumHit = hit->GetHitPadNum();
            TVector3 position = hit->GetPosition();
            Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));
            Double_t posZCal =
               fZk - (fEntTB - hit->GetTimeStamp()) * drift_cal; // Recalibrating Z position from Time Bucket
            Double_t posZCalCorr =
               fZk - (fEntTB - hit->GetTimeStampCorr()) * drift_cal; // Recalibrating Z position from Time Bucket

            // std::cout << " Pos Z Vector : " << position.Z() << " posZCal : " << posZCal << " fEntTB : " << fEntTB <<
            // " Time Stamp : " << hit->GetTimeStamp() << " Drift cal : " << drift_cal << std::endl;

            if (radius > fRadThreshold) {
               // #pragma omp parallel for ordered schedule(dynamic) // TODO: Check performance
               for (Int_t itheta = 0; itheta < 1023; ++itheta) {
                  Float_t angle = TMath::Pi() * (static_cast<Float_t>(itheta) / 1023);
                  Float_t d0_RZ = (TMath::Cos(angle) * radius) +
                                  (TMath::Sin(angle) * posZCalCorr); // posZCal can be replaced anytime by position.Z()
                  // #pragma omp ordered
                  HistHoughRZInner[iQ]->Fill(angle, d0_RZ);
                  // #pragma omp ordered
                  // FillHoughMap(angle,d0_RZ);
               } // Hough Angle loop
            }
         } // Hit loop

         // HoughMapKey.push_back(GetHoughParameters());
         HoughPar.push_back(GetHoughParameters(HistHoughRZInner[iQ]));
         // HoughParSTD.push_back(GetHoughParameters());
      } // Quadrant loop
   }

   for (Int_t i = 0; i < 4; ++i) {
      delete HistHoughRZInner[i];
   }

   /*
   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->cd(1);
   HistHoughRZInner[0]->Draw("zcol");
   c1->cd(2);
   HistHoughRZInner[1]->Draw("zcol");
   c1->cd(3);
   HistHoughRZInner[2]->Draw("zcol");
   c1->cd(4);
   HistHoughRZInner[3]->Draw("zcol");
   */
}

void AtHoughSpaceLine::CalcHoughSpace(AtEvent *event, TH2Poly *hPadPlane, const multiarray &PadCoord)
{
   // TODO
}

void AtHoughSpaceLine::CalcMultiHoughSpace(AtEvent *event)
{

   Int_t nHits = event->GetNumHits();
   Double_t drift_cal = fDriftVelocity * fTBTime / 100.0; // mm

   std::vector<AtHit *> HitBuffer;
   std::vector<AtHit *> HitBuffer2;

   // HistHoughRZ = new TH2F("HistHoughRZ","HistHoughRZ",500,0,3.15,500,-300,300);

   // TH2F *rad_z = new TH2F("rad_z","rad_z",1000,-500,500,1000,0,1000);

   // TF1* fHoughLinearFit = new TF1("HoughLinearFit"," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) +
   // [1]/TMath::Sin([0])",0,500);

   // TF1 *fHoughLinearFit;
   // fHoughLinearFit = new TF1("HoughLinearFit"," (  (-TMath::Cos([0])/TMath::Sin([0]))*x ) +
   // [1]/TMath::Sin([0])",0,500);

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit *hit = event->GetHit(iHit);
      Int_t PadNumHit = hit->GetHitPadNum();
      TVector3 position = hit->GetPosition();
      Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));
      // Double_t posZCal     = fZk - (fEntTB - hit->GetTimeStamp())*drift_cal; // Recalibrating Z position from Time
      // Bucket Double_t posZCalCorr = fZk - (fEntTB - hit->GetTimeStampCorr())*drift_cal; // Recalibrating Z position
      // from Time Bucket

      Int_t itheta = 0;
      // if(hit->GetTimeStamp()>130){
      //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
      for (itheta = 0; itheta < 1023; itheta++) {
         Float_t angle = TMath::Pi() * (static_cast<Float_t>(itheta) / 1023);
         Float_t d0_RZ = (TMath::Cos(angle) * radius) +
                         (TMath::Sin(angle) * hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
         //#pragma omp ordered
         HistHoughRZ->Fill(angle, d0_RZ);
         // rad_z->Fill(hit->GetTimeStamp(),radius);

      } // Hough Angle loop
      //}
   }

   // delete HistHoughRZ;
   // HistHoughRZ->Draw("zcol");
   // rad_z->Draw();

   std::pair<Double_t, Double_t> HoughParBuff = GetHoughParameters(HistHoughRZ);
   HoughPar.push_back(HoughParBuff);

   /*std::cout<<"  ++++++++++++++++++++++++++++++++++++++ "<<std::endl;
   std::cout<<"  First Hough Space :  "<<std::endl;
   std::cout<<"  Hough Parameter Distance : "<<HoughPar.first<<std::endl;
   std::cout<<"  Hough Parameter Angle : "<<HoughPar.second<<std::endl;*/

   // fHoughLinearFit->SetParameter(0,TMath::Pi()/2.0-HoughPar.first);
   // fHoughLinearFit->SetParameter(1,HoughPar.second);

   // rad_z->clear();

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit *hit = event->GetHit(iHit);
      Int_t PadNumHit = hit->GetHitPadNum();
      TVector3 position = hit->GetPosition();
      Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));
      Double_t geo_dist = TMath::Abs(TMath::Cos(HoughParBuff.first) * radius +
                                     TMath::Sin(HoughParBuff.first) * hit->GetTimeStamp() - HoughParBuff.second);
      if (geo_dist > fHoughDist) {
         HitBuffer.push_back(hit);
         // rad_z->Fill(hit->GetTimeStamp(),radius);
      }
   }

   //******************************************** Recursive Hough Space calculation ***********************************

   // std::cout<<HitBuffer.size()<<std::endl;

   for (Int_t i = 0; i < 3; i++) {

      HistHoughRZ->Reset();
      // rad_z->Reset();

      for (Int_t iHit = 0; iHit < HitBuffer.size(); iHit++) {
         AtHit *hit = HitBuffer.at(iHit);
         Int_t PadNumHit = hit->GetHitPadNum();
         TVector3 position = hit->GetPosition();
         Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));

         Int_t itheta = 0;
         //#pragma omp parallel for ordered schedule(dynamic) private(itheta) //TOOD: Check performance
         for (itheta = 0; itheta < 1023; itheta++) {
            Float_t angle = TMath::Pi() * (static_cast<Float_t>(itheta) / 1023);
            Float_t d0_RZ =
               (TMath::Cos(angle) * radius) +
               (TMath::Sin(angle) * hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
            //#pragma omp ordered
            HistHoughRZ->Fill(angle, d0_RZ);
            // rad_z->Fill(hit->GetTimeStamp(),radius);

         } // Hough Angle loop
      }

      HoughParBuff = GetHoughParameters(HistHoughRZ);
      HoughPar.push_back(HoughParBuff);

      /*  std::cout<<"  Iterative Hough Space  "<<i<<std::endl;
        std::cout<<"  Hough Parameter Angle : "<<HoughPar.first<<std::endl;
         std::cout<<"  Hough Parameter Distance : "<<HoughPar.second<<std::endl;*/

      // fHoughLinearFit->SetParameter(0,TMath::Pi()/2.0-HoughPar.first);
      // fHoughLinearFit->SetParameter(1,HoughPar.second);

      for (Int_t iHit = 0; iHit < HitBuffer.size(); iHit++) {
         AtHit *hit = HitBuffer.at(iHit);
         Int_t PadNumHit = hit->GetHitPadNum();
         TVector3 position = hit->GetPosition();
         Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));
         Double_t geo_dist = TMath::Abs(TMath::Cos(HoughParBuff.first) * radius +
                                        TMath::Sin(HoughParBuff.first) * hit->GetTimeStamp() - HoughParBuff.second);
         if (geo_dist > fHoughDist) {
            // std::cout<<geo_dist<<" Time Bucket : "<<hit->GetTimeStamp()<<std::endl;
            HitBuffer2.push_back(hit);
            // rad_z->Fill(hit->GetTimeStamp(),radius);
         }
      }

      HitBuffer.clear();
      HitBuffer = HitBuffer2;
      HitBuffer2.clear();
   }

   // rad_z->Draw();
   // fHoughLinearFit->Draw("SAME");

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      AtHit *hit = event->GetHit(iHit);
      Int_t PadNumHit = hit->GetHitPadNum();
      TVector3 position = hit->GetPosition();
      Float_t radius = TMath::Sqrt(TMath::Power(position.X(), 2) + TMath::Power(position.Y(), 2));

      Int_t itheta = 0;

      for (itheta = 0; itheta < 1023; itheta++) {
         Float_t angle = TMath::Pi() * (static_cast<Float_t>(itheta) / 1023);
         Float_t d0_RZ = (TMath::Cos(angle) * radius) +
                         (TMath::Sin(angle) * hit->GetTimeStamp()); // posZCal can be replaced anytime by position.Z()
         //#pragma omp ordered
         HistHoughRZ->Fill(angle, d0_RZ);

      } // Hough Angle loop
      //}
   }
}

std::pair<Double_t, Double_t> AtHoughSpaceLine::GetHoughParameters(TH2F *hist)
{

   std::pair<Double_t, Double_t> HoughParBuff;
   Double_t xpos = 0.0;
   Double_t ypos = 0.0;
   Int_t locmaxx, locmaxy, locmaxz;
   hist->GetMaximumBin(locmaxx, locmaxy, locmaxz);
   xpos = hist->GetXaxis()->GetBinCenter(locmaxx);
   ypos = hist->GetYaxis()->GetBinCenter(locmaxy);
   HoughMax.push_back(hist->GetBinContent(locmaxx, locmaxy));
   // std::cout<<" X Hough Position : "<<180-xpos*180/TMath::Pi()<<std::endl;
   // std::cout<<" X Hough Position : "<<xpos<<std::endl;
   // std::cout<<" Y Hough Position : "<<ypos<<std::endl;
   HoughParBuff.first = xpos;
   HoughParBuff.second = ypos;
   return HoughParBuff;
}

std::vector<std::pair<Double_t, Double_t>> AtHoughSpaceLine::GetMultiHoughParameters(TH2F *hist)
{
   std::vector<std::pair<Double_t, Double_t>> HoughParInner;

   Int_t maxBin = 0;
   Double_t xpos1 = 0.0;
   Double_t ypos1 = 0.0;

   for (Int_t binx = 1; binx < fXbinRZ; ++binx) {
      for (Int_t biny = 1; biny < fYbinRZ; ++biny) {
         Int_t buff = hist->GetBinContent(binx, biny);

         if (buff > maxBin) {
            maxBin = buff;
            xpos1 = hist->GetXaxis()->GetBinCenter(binx);
            ypos1 = hist->GetYaxis()->GetBinCenter(biny);
         }
      }
   }

   // std::cout << maxBin << std::endl;
   // std::cout << " X Hough Position 1: " << xpos1 << std::endl;
   // std::cout << " Y Hough Position 1: " << ypos1 << std::endl;

   // FIXME: do anything

   return HoughParInner;
}

std::pair<Double_t, Double_t> AtHoughSpaceLine::GetHoughParameters()
{

   std::pair<Double_t, Double_t> HoughParBuff;
   // Method 1:
   /*  using pair_type = decltype(HoughMap)::value_type;

      auto pr = std::max_element
       (
           std::begin(HoughMap), std::end(HoughMap),
           [] (const pair_type & p1, const pair_type & p2) {
             return p1.second < p2.second;
           }
        );

         std::cout << "pr->first : " << pr->first<< '\n';
         std::cout << "pr->second : " << pr->second<< '\n';
         std::cout << " pr->first&0xFFFFFFFF " << (pr->first&0xFFFFFFFF) << '\n';
         std::cout << " ((pr->first>>32)&0xFFFFFFFF) " << ((pr->first>>32)&0xFFFFFFFF) << '\n';*/

   // Method 2:
   std::map<ULong64_t, Int_t>::iterator maximum = std::max_element(HoughMap.begin(), HoughMap.end(), maxpersecond());
   HoughParBuff.first = (maximum->first & 0xFFFFFFFF) / 159.0; // Binning of the associated histogram
   HoughParBuff.second = ((maximum->first & 0xFFFFFFFF00000000) >> 32) / 17;
   // std::cout<<" Size of First Element in Hough Map (byte) :"<<sizeof(maximum->first)<<std::endl;
   // std::cout<<" Size of Second Element of Houg Map (byte) :"<<sizeof(maximum->second)<<std::endl;
   // std::cout<<" Angle : "<<(maximum->first&0xFFFFFFFF)/159.0<<std::endl;
   // std::cout<<" Distance : "<<((maximum->first&0xFFFFFFFF00000000)>>32)/17<<std::endl;
   // std::cout<<" Number of Maximum Votes : "<<maximum->second<<std::endl;
   return HoughParBuff;
}

void AtHoughSpaceLine::FillHoughMap(Double_t ang, Double_t dist)
{

   ULong64_t MapKey;

   // if(ang<0 || dist<0)  std::cout<<" -I- AtHougSpaceLine : Negative values for Hough Space parameters "<<std::endl;

   if (ang > 0 || dist > 0) {
      Int_t dist_enc = static_cast<Int_t>(round(dist * 17));
      Int_t ang_enc = static_cast<Int_t>(round(ang * 159.0));
      /*std::cout<<" Filling std::map for the Hough Space "<<std::endl;
      std::cout<<" Hough Angle : "<<ang<<" Hough Dist : "<<dist<<std::endl;
      std::cout<<" Encoded Hough Angle : "<<ang_enc<<" Encoded Hough Dist : "<<dist_enc<<std::endl;*/

      MapKey = dist_enc & 0xFFFFFFFF;
      MapKey = (MapKey << 32) + (ang_enc & 0xFFFFFFFF);
      HoughMap[MapKey]++;

      /*  std::cout<<" Map Key : "<<MapKey<<std::endl;
        std::cout<<" Map Key Dist "<<(MapKey&0xFFFFFFFF)<<std::endl;
        std::cout<<" Map Key Ang "<<((MapKey>>32)&(0xFFFFFFFF))<<std::endl;*/
   }
}

Int_t AtHoughSpaceLine::MinimizeTrack(AtTrack *track)
{

   gErrorIgnoreLevel = kFatal;
   Int_t nd = 10000;
   TGraph2D *gr = new TGraph2D(); /////NB: This should be created on the heap only once so it should move outside of
                                  /// this function!!!!!!!!!!!!!!!
   std::vector<AtHit> *HitArray = track->GetHitArray();

   double p0[4] = {10, 20, 1, 2}; // For the moment those are dummy parameters

   for (Int_t N = 0; N < HitArray->size(); N++) {
      AtHit hit = HitArray->at(N);
      TVector3 pos = hit.GetPosition();
      gr->SetPoint(N, pos.X(), pos.Y(), pos.Z());
   }

   ROOT::Fit::Fitter fitter;
   SumDistance2 sdist(gr);
#ifdef __CINT__
   ROOT::Math::Functor fcn(&sdist, 4, "SumDistance2");
#else
   ROOT::Math::Functor fcn(sdist, 4);
#endif
   // set the function and the initial parameter values
   double pStart[4] = {1, 1, 1, 1};
   fitter.SetFCN(fcn, pStart);
   // set step sizes different than default ones (0.3 times parameter values)
   for (int i = 0; i < 4; ++i)
      fitter.Config().ParSettings(i).SetStepSize(0.01);

   bool ok = fitter.FitFCN();
   if (!ok) {
      Error("line3Dfit", "Line3D Fit failed");
      return 1;
   }

   const ROOT::Fit::FitResult &result = fitter.Result();
   const ROOT::Math::Minimizer *min = fitter.GetMinimizer();
   double sigma2 = 25.0; // Size of the pad
   double Chi2_min = min->MinValue();
   int NDF = min->NFree();
   int npoints = gr->GetN();
   const double *parFitBuff = result.GetParams();
   std::vector<Double_t> parFit;
   for (Int_t i = 0; i < 4; i++)
      parFit.push_back(parFitBuff[i]); // 4 parameters per fit
   track->SetFitPar(parFit);
   track->SetMinimum(Chi2_min);
   track->SetNFree(NDF);

   // std::cout<<parFit[0]<<" "<<parFit[1]<<"  "<<parFit[2]<<" "<<parFit[3]<<std::endl;
   // std::cout<<" Chi2 (Minuit) : "<<Chi2_min<<" NDF : "<<NDF<<std::endl;
   // std::cout<<" Chi2 reduced  : "<<(Chi2_min/sigma2/(double) npoints);

   // std::cout << "Total final distance square " << result.MinFcnValue() << std::endl;
   // result.Print(std::cout);

   // Draw the fit
   /*gr->Draw("p0");
   const double * parFit = result.GetParams();
   int n = 1000;
   double t0 = 0;
   double dt = 1000;
   TPolyLine3D *l = new TPolyLine3D(n);
   for (int i = 0; i <n;++i) {
      double t = t0+ dt*i/n;
      double x,y,z;
      SetLine(t,parFit,x,y,z);
      l->SetPoint(i,x,y,z);
      //std::cout<<" x : "<<x<<" y : "<<y<<"  z : "<<z<<std::endl;
   }
   l->SetLineColor(kRed);
   l->Draw("same");*/

   return 0;
}

Double_t AtHoughSpaceLine::distance2(double x, double y, double z, const double *p)
{

   // distance line point is D= | (xp-x0) cross  ux |
   // where ux is direction of line and x0 is a point in the line (like t = 0) and x1 is in t=1
   XYZVector xp(x, y, z);
   XYZVector x0(p[0], p[2], 0.);
   XYZVector x1(p[0] + p[1], p[2] + p[3], 1.);
   XYZVector u = (x1 - x0).Unit();
   double d2 = ((xp - x0).Cross(u)).Mag2();
   return d2;
}

void AtHoughSpaceLine::SetLine(double t, const double *p, double &x, double &y, double &z)
{
   // a parameteric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = p[0] + p[1] * t;
   y = p[2] + p[3] * t;
   z = t;
}

Bool_t AtHoughSpaceLine::CheckTrackID(Int_t trackID, std::vector<AtTrack> trackArray)
{
   auto it = find_if(trackArray.begin(), trackArray.end(),
                     [&trackID](AtTrack &track) { return track.GetTrackID() == trackID; });
   if (it != trackArray.end()) {
      auto hitInd = std::distance<std::vector<AtTrack>::const_iterator>(trackArray.begin(), it);
      return kTRUE;
   } else
      return kFALSE;
}
