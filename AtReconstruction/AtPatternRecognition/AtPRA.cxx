#include "AtPRA.h"

#include "AtHit.h"        // for AtHit, XYZPoint
#include "AtHitCluster.h" // for AtHitCluster
#include "AtPatternCircle2D.h"
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtSampleConsensus.h"
#include "AtTrack.h" // for XYZPoint, AtTrack

#include <Math/Point3D.h>   // for PositionVector3D, Cart...
#include <Math/Vector3D.h>  // for DisplacementVector3D
#include <TGraph.h>         // for TGraph
#include <TMath.h>          // for Power, Sqrt, ATan2, Pi
#include <TMatrixDSymfwd.h> // for TMatrixDSym
#include <TMatrixTSym.h>    // for TMatrixTSym
#include <TVector3.h>       // for TVector3

#include <algorithm> // for max, for_each, copy_if
#include <cmath>     // for fabs, acos
#include <cstddef>   // for size_t
#include <exception> // for exception
#include <iostream>  // for operator<<, basic_ostream
#include <iterator>  // for back_insert_iterator
#include <memory>    // for shared_ptr, __shared_p...
#include <utility>   // for make_pair

ClassImp(AtPATTERN::AtPRA);

void AtPATTERN::AtPRA::SetTrackCurvature(AtTrack &track)
{
   std::cout << " new track  "
             << "\n";
   std::vector<double> radius_vec;
   std::vector<AtHit> hitArray = track.GetHitArray();
   int nstep = 0.60 * hitArray.size(); // 20% of the hits to calculate the radius of curvature with less fluctuations

   for (Int_t iHit = 0; iHit < (hitArray.size() - nstep); iHit++) {

      AtHit hitA = hitArray.at(iHit);
      AtHit hitB = hitArray.at((int)(iHit + (nstep / 2.0)));
      AtHit hitC = hitArray.at((int)(iHit + nstep));

      // std::cout<<nstep<<" "<<iHit<<"  "<<(int)(iHit+(nstep/2.0))<<"  "<<(int)(iHit+nstep)<<"\n";

      auto posA = hitA.GetPosition();
      auto posB = hitB.GetPosition();
      auto posC = hitC.GetPosition();

      double slopeAB = (posB.Y() - posA.Y()) / (posB.X() - posA.X()); // m1
      double slopeBC = (posC.Y() - posB.Y()) / (posC.X() - posB.X()); // m2

      double centerX = (slopeAB * slopeBC * (posA.Y() - posC.Y()) + slopeBC * (posB.X() + posA.X()) -
                        slopeAB * (posB.X() + posC.X())) /
                       (2.0 * (slopeBC - slopeAB));

      double centerY = (-1 / slopeAB) * (centerX - (posB.X() + posA.X()) / 2.0) + (posB.Y() + posA.Y()) / 2.0;

      // std::cout<<" Center "<<centerX<<" - "<<centerY<<"\n";

      double radiusA = TMath::Sqrt(TMath::Power(posA.X() - centerX, 2) + TMath::Power(posA.Y() - centerY, 2));
      radius_vec.push_back(radiusA);
      double radiusB = TMath::Sqrt(TMath::Power(posB.X() - centerX, 2) + TMath::Power(posB.Y() - centerY, 2));
      radius_vec.push_back(radiusB);
      double radiusC = TMath::Sqrt(TMath::Power(posC.X() - centerX, 2) + TMath::Power(posC.Y() - centerY, 2));
      radius_vec.push_back(radiusC);
   }
}

void AtPATTERN::AtPRA::SetTrackInitialParameters(AtTrack &track)
{

   // std::cout<<" Processing track with "<<track.GetHitArray()->size()<<" points."<<"\n";
   //  Get the radius of curvature from RANSAC
   SampleConsensus::AtSampleConsensus RansacSmoothRadius;
   RansacSmoothRadius.SetPatternType(AtPatterns::PatternType::kCircle2D);
   RansacSmoothRadius.SetMinHitsPattern(0.1 * track.GetHitArray().size());
   RansacSmoothRadius.SetDistanceThreshold(6.0);
   auto circularTracks =
      RansacSmoothRadius.Solve(track.GetHitArray()).GetTrackCand(); // Only part of the spiral is used
                                                                    // This function also sets the coefficients
                                                                    // i.e. radius of curvature and center

   if (!circularTracks.empty()) {

      std::vector<AtHit> hits = circularTracks.at(0).GetHitArray();

      auto circle = dynamic_cast<const AtPatterns::AtPatternCircle2D *>(circularTracks.at(0).GetPattern());
      auto center = circle->GetCenter();
      auto radius = circle->GetRadius();

      track.SetGeoCenter({center.X(), center.Y()});
      track.SetGeoRadius(radius);

      std::vector<double> wpca;
      std::vector<double> whit;
      std::vector<double> arclength;

      auto arclengthGraph = std::make_unique<TGraph>();

      auto posPCA = hits.at(0).GetPosition();

      std::vector<AtHit> thetaHits;

      for (size_t i = 0; i < hits.size(); ++i) {

         auto pos = hits.at(i).GetPosition();

         auto temp = posPCA - center;
         wpca.push_back(TMath::ATan2(temp.Y(), temp.X()));
         temp = pos - center;
         whit.push_back(TMath::ATan2(temp.Y(), temp.X()));

         arclength.push_back(fabs(radius * (wpca.at(i) - whit.at(i))));

         arclengthGraph->SetPoint(arclengthGraph->GetN(), arclength.at(i), pos.Z());

         // std::cout<<pos.X()<<"  "<<pos.Y()<<" "<<pos.Z()<<" "<<hits->at(i).GetTimeStamp()<<"
         // "<<hits->at(i).GetCharge()<<"\n";

         // Add a hit in the Arc legnth - Z plane
         Double_t xPos = arclength.at(i);
         Double_t yPos = pos.Z();
         Double_t zPos = i * 1E-19;

         // cloud->points[i].x = arclength.at(i);
         // cloud->points[i].y = pos.Z();
         // cloud->points[i].z = i*1E-19;

         thetaHits.emplace_back(i, hits.at(i).GetPadNum(), XYZPoint(xPos, yPos, zPos), hits.at(i).GetCharge());
      }

      // TF1 *f1 = new TF1("f1", "pol1", -500, 500);
      // TF1 * f1 = new TF1("f1",[](double *x, double *p) { return (p[0]+p[1]*x[0]); },-500,500,2);
      // TF1 * f1 = new TF1("f1","[0]+[1]*x",-500,500);
      // TF1 * f1 = new TF1("f1",fitf,-500,500,2);
      // arclengthGraph->Fit(f1,"R");
      // Double_t slope = f1->GetParameter(1);
      // std::cout<<" Slope "<<slope<<"\n";

      Double_t slope = 0;
      Double_t angle = 0.0;
      Double_t phi0 = 0.0;

      try {

         if (thetaHits.size() > 0) {
            // std::cout<<" RANSAC Theta "<<"\n";

            SampleConsensus::AtSampleConsensus RansacTheta;
            RansacSmoothRadius.SetPatternType(AtPatterns::PatternType::kLine);
            RansacSmoothRadius.SetMinHitsPattern(0.1 * thetaHits.size());
            RansacSmoothRadius.SetDistanceThreshold(6.0);
            auto thetaTracks =
               RansacSmoothRadius.Solve(track.GetHitArray()).GetTrackCand(); // Only part of the spiral is used

            if (thetaTracks.size() > 0) {

               // NB: Only the most intense line is taken, if any
               auto line = dynamic_cast<const AtPatterns::AtPatternLine *>(thetaTracks.at(0).GetPattern());

               // std::vector<Double_t> coeffTheta = thetaTracks.at(0).GetRANSACCoeff();
               auto coeffTheta = line->GetPatternPar();

               int sign = 0;

               if (coeffTheta.at(3) * coeffTheta.at(4) < 0)
                  sign = -1;
               else
                  sign = 1;

               if (coeffTheta.at(3) != 0)
                  angle = acos(sign * fabs(coeffTheta.at(4))) * TMath::RadToDeg();

               auto temp2 = posPCA - center;
               phi0 = TMath::ATan2(temp2.Y(), temp2.X());

            } // thetaTracks

            track.SetGeoTheta(angle * TMath::Pi() / 180.0);
            track.SetGeoPhi(phi0);

         } // if

      } catch (std::exception &e) {

         std::cout << " AtPRA::SetTrackInitialParameters - Exception caught : " << e.what() << "\n";
      }

   } // end if (!circularTracks->empty())
}

Double_t fitf(Double_t *x, Double_t *par)
{

   return par[0] + par[1] * x[0];
}

void AtPATTERN::AtPRA::Clusterize3D(AtTrack &track, Float_t distance, Float_t radius)
{
   std::vector<AtHit> hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   // std::cout<<" ================================================================= "<<"\n";
   // std::cout<<" Clusterizing track : "<<track.GetTrackID()<<"\n";

   /*for(auto iHits=0;iHits<hitArray->size();++iHits)
     {
       TVector3 pos    = hitArray->at(iHits).GetPosition();
       double Q = hitArray->at(iHits).GetCharge();
       int TB          = hitArray->at(iHits).GetTimeStamp();
       //std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";

       }*/

   // Diffusion coefficients (TODO: Get them from the parameter file)
   Double_t driftVel = 1.0;       // cm/us
   Double_t samplingRate = 0.320; // us
   Double_t d_t = 0.0009;         // cm^2/us
   Double_t d_l = 0.0009;         // cm^2/us
   Double_t D_T = TMath::Sqrt((2.0 * d_t) / driftVel);
   Double_t D_L = TMath::Sqrt((2.0 * d_l) / driftVel);

   if (hitArray.size() > 0) {

      auto refPos = hitArray.at(0).GetPosition(); // First hit
      // TODO: Create a clustered hit from the very first hit (test)

      for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

         auto hit = hitArray.at(iHit);

         // Check distance with respect to reference Hit
         Double_t distRef = TMath::Sqrt((hit.GetPosition() - refPos).Mag2());

         if (distRef < distance) {

            continue;

         } else {

            // std::cout<<" Clustering "<<iHit<<" of "<<hitArray->size()<<"\n";
            // std::cout<<" Distance to reference : "<<distRef<<"\n";
            // std::cout<<" Reference position : "<<refPos.X()<<" - "<<refPos.Y()<<" - "<<refPos.Z()<<" -
            // "<<refPos.Mag()<<"\n";

            Double_t clusterQ = 0.0;
            hitTBArray.clear();
            std::copy_if(
               hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
               [&refPos, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - refPos).Mag2()) < radius; });

            // std::cout<<" Clustered "<<hitTBArray.size()<<" Hits "<<"\n";

            if (hitTBArray.size() > 0) {
               double x = 0, y = 0, z = 0;
               double sigma_x = 0, sigma_y = 0, sigma_z = 0;

               int timeStamp;
               std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
               hitCluster->SetClusterID(clusterID);
               Double_t hitQ = 0.0;
               std::for_each(hitTBArray.begin(), hitTBArray.end(),
                             [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                              &samplingRate](AtHit &hitInQ) {
                                auto pos = hitInQ.GetPosition();
                                x += pos.X() * hitInQ.GetCharge();
                                y += pos.Y() * hitInQ.GetCharge();
                                z += pos.Z();
                                hitQ += hitInQ.GetCharge();
                                timeStamp += hitInQ.GetTimeStamp();

                                // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                // Conferences50epjconf/2010010)
                                sigma_x += hitInQ.GetCharge() *
                                           TMath::Sqrt(TMath::Power(0.2, 2) +
                                                       pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                sigma_y += sigma_x;
                                sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                       pos.Z() * TMath::Power(D_L, 2));
                             });
               x /= hitQ;
               y /= hitQ;
               z /= hitTBArray.size();
               timeStamp /= std::round(timeStamp);

               sigma_x /= hitQ;
               sigma_y /= hitQ;
               sigma_z /= hitTBArray.size();

               XYZPoint clustPos(x, y, z);
               Bool_t checkDistance = kTRUE;

               // Check distance with respect to existing clusters
               for (auto iClusterHit : *track.GetHitClusterArray()) {
                  if (TMath::Sqrt((iClusterHit.GetPosition() - clustPos).Mag2()) < distance) {
                     // std::cout<<" Cluster with less than  : "<<distance<<" found "<<"\n";
                     checkDistance = kFALSE;
                     continue;
                  }
               }

               if (checkDistance) {
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  track.AddClusterHit(hitCluster);
               }
            }
         }

         // Sanity check
         /*std::cout<<" Hits for cluster "<<iHit<<" centered in "<<refPos.X()<<" - "<<refPos.Y()<<"-"<<refPos.Z()<<"\n";
    for(auto iHits=0;iHits<hitTBArray.size();++iHits)
         {
           TVector3 pos    = hitTBArray.at(iHits).GetPosition();
           double Q = hitTBArray.at(iHits).GetCharge();
           int TB          = hitTBArray.at(iHits).GetTimeStamp();
           std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
      std::cout<<" Distance to cluster center "<<TMath::Abs((track.GetHitClusterArray()->back().GetPosition() -
    pos).Mag())<<"\n";

    }
         std::cout<<"=================================================="<<"\n";*/

         refPos = hitArray.at(iHit).GetPosition();

         //} // if distance

      } // for

   } // if array size
}

void AtPATTERN::AtPRA::ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius)
{
   std::vector<AtHit> hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   // std::cout<<" ================================================================= "<<"\n";
   // std::cout<<" Clusterizing track : "<<track.GetTrackID()<<"\n";

   /*for(auto iHits=0;iHits<hitArray->size();++iHits)
     {
       TVector3 pos    = hitArray->at(iHits).GetPosition();
       double Q = hitArray->at(iHits).GetCharge();
       int TB          = hitArray->at(iHits).GetTimeStamp();
       //std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
       }*/

   // Diffusion coefficients (TODO: Get them from the parameter file)
   Double_t driftVel = 1.0;       // cm/us
   Double_t samplingRate = 0.320; // us
   Double_t d_t = 0.0009;         // cm^2/us
   Double_t d_l = 0.0009;         // cm^2/us
   Double_t D_T = TMath::Sqrt((2.0 * d_t) / driftVel);
   Double_t D_L = TMath::Sqrt((2.0 * d_l) / driftVel);

   if (hitArray.size() > 0) {

      auto refPos = hitArray.at(0).GetPosition(); // First hit
      // TODO: Create a clustered hit from the very first hit (test)

      for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

         auto hit = hitArray.at(iHit);

         // Check distance with respect to reference Hit
         Double_t distRef = TMath::Sqrt((hit.GetPosition() - refPos).Mag2());

         if (distRef < distance) {

            continue;

         } else {

            // std::cout<<" Clustering "<<iHit<<" of "<<hitArray->size()<<"\n";
            // std::cout<<" Distance to reference : "<<distRef<<"\n";
            // std::cout<<" Reference position : "<<refPos.X()<<" - "<<refPos.Y()<<" - "<<refPos.Z()<<" -
            // "<<refPos.Mag()<<"\n";

            Double_t clusterQ = 0.0;
            hitTBArray.clear();
            std::copy_if(
               hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
               [&refPos, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - refPos).Mag2()) < radius; });

            // std::cout<<" Clustered "<<hitTBArray.size()<<" Hits "<<"\n";

            if (hitTBArray.size() > 0) {
               double x = 0, y = 0, z = 0;
               double sigma_x = 0, sigma_y = 0, sigma_z = 0;

               int timeStamp;
               std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
               hitCluster->SetClusterID(clusterID);
               Double_t hitQ = 0.0;
               std::for_each(hitTBArray.begin(), hitTBArray.end(),
                             [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                              &samplingRate](AtHit &hitInQ) {
                                XYZPoint pos = hitInQ.GetPosition();
                                x += pos.X() * hitInQ.GetCharge();
                                y += pos.Y() * hitInQ.GetCharge();
                                z += pos.Z();
                                hitQ += hitInQ.GetCharge();
                                timeStamp += hitInQ.GetTimeStamp();

                                // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                // Conferences50epjconf/2010010)
                                sigma_x += hitInQ.GetCharge() *
                                           TMath::Sqrt(TMath::Power(0.2, 2) +
                                                       pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                sigma_y += sigma_x;
                                sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                       pos.Z() * TMath::Power(D_L, 2));
                             });
               x /= hitQ;
               y /= hitQ;
               z /= hitTBArray.size();
               timeStamp /= std::round(timeStamp);

               sigma_x /= hitQ;
               sigma_y /= hitQ;
               sigma_z /= hitTBArray.size();

               XYZPoint clustPos(x, y, z);
               Bool_t checkDistance = kTRUE;

               // Check distance with respect to existing clusters
               for (auto iClusterHit : *track.GetHitClusterArray()) {
                  if (TMath::Sqrt((iClusterHit.GetPosition() - clustPos).Mag2()) < distance) {
                     // std::cout<<" Cluster with less than  : "<<distance<<" found "<<"\n";
                     checkDistance = kFALSE;
                     continue;
                  }
               }

               if (checkDistance) {
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  track.AddClusterHit(hitCluster);
               }
            }
         }

         // Sanity check
         /*std::cout<<" Hits for cluster "<<iHit<<" centered in "<<refPos.X()<<" - "<<refPos.Y()<<"-"<<refPos.Z()<<"\n";
    for(auto iHits=0;iHits<hitTBArray.size();++iHits)
         {
           TVector3 pos    = hitTBArray.at(iHits).GetPosition();
           double Q = hitTBArray.at(iHits).GetCharge();
           int TB          = hitTBArray.at(iHits).GetTimeStamp();
           std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
      std::cout<<" Distance to cluster center "<<TMath::Abs((track.GetHitClusterArray()->back().GetPosition() -
    pos).Mag())<<"\n";
    }
         std::cout<<"=================================================="<<"\n";*/

         refPos = hitArray.at(iHit).GetPosition();

         //} // if distance

      } // for

      // Smoothing track
      std::vector<AtHitCluster> *hitClusterArray = track.GetHitClusterArray();
      radius /= 2.0;
      std::vector<std::shared_ptr<AtHitCluster>> hitClusterBuffer;

      // std::cout<<" Hit cluster array size "<<hitClusterArray->size()<<"\n";

      if (hitClusterArray->size() > 2) {

         for (auto iHitCluster = 0; iHitCluster < hitClusterArray->size() - 1;
              ++iHitCluster) // Calculating distances between pairs of clusters
         {

            XYZPoint clusBack = hitClusterArray->at(iHitCluster).GetPosition();
            XYZPoint clusForw = hitClusterArray->at(iHitCluster + 1).GetPosition();
            XYZPoint clusMidPos = clusBack + (clusForw - clusBack) * 0.5;
            std::vector<XYZPoint> renormClus{clusBack, clusMidPos};

            if (iHitCluster == (hitClusterArray->size() - 2))
               renormClus.push_back(clusForw);

            // Create a new cluster and renormalize the charge of the other with half the radius.
            for (auto iClus : renormClus) {
               hitTBArray.clear();
               std::copy_if(hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
                            [&iClus, radius](AtHit &hitIn) {
                               return TMath::Sqrt((hitIn.GetPosition() - iClus).Mag2()) < radius;
                            });

               if (hitTBArray.size() > 0) {
                  double x = 0, y = 0, z = 0;
                  double sigma_x = 0, sigma_y = 0, sigma_z = 0;

                  int timeStamp = 0;
                  std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
                  hitCluster->SetClusterID(clusterID);
                  Double_t hitQ = 0.0;
                  std::for_each(hitTBArray.begin(), hitTBArray.end(),
                                [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                                 &samplingRate](AtHit &hitInQ) {
                                   auto pos = hitInQ.GetPosition();
                                   x += pos.X() * hitInQ.GetCharge();
                                   y += pos.Y() * hitInQ.GetCharge();
                                   z += pos.Z();
                                   hitQ += hitInQ.GetCharge();
                                   timeStamp += hitInQ.GetTimeStamp();

                                   // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                   // Conferences50epjconf/2010010)
                                   sigma_x +=
                                      hitInQ.GetCharge() *
                                      TMath::Sqrt(TMath::Power(0.2, 2) +
                                                  pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                   sigma_y += sigma_x;
                                   sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                          pos.Z() * TMath::Power(D_L, 2));
                                });
                  x /= hitQ;
                  y /= hitQ;
                  z /= hitTBArray.size();
                  timeStamp /= std::round(timeStamp);

                  sigma_x /= hitQ;
                  sigma_y /= hitQ;
                  sigma_z /= hitTBArray.size();

                  TVector3 clustPos(x, y, z);
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  hitClusterBuffer.push_back(hitCluster);

               } // hitTBArray size

            } // for iClus

         } // for HitArray

         // Remove previous clusters
         track.ResetHitClusterArray();

         // Adding new clusters
         for (auto iHitClusterRe : hitClusterBuffer) {

            track.AddClusterHit(iHitClusterRe);
         }

      } // Cluster array size

   } // if array size
}

void AtPATTERN::AtPRA::Clusterize3D(AtTrack &track)
{
   std::vector<AtHit> hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   Float_t distance = 10.0;
   Float_t radius = 30.0;

   // std::cout<<" ================================================================= "<<"\n";
   // std::cout<<" Clusterizing track : "<<track.GetTrackID()<<"\n";

   for (auto &iHits : hitArray) {
      XYZPoint pos = iHits.GetPosition();
      // double Q = iHits.GetCharge();
      // int TB = iHits.GetTimeStamp();
      //  std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
   }

   if (hitArray.size() > 0) {

      auto refPos = hitArray.at(0).GetPosition(); // First hit

      for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

         auto hit = hitArray.at(iHit);

         // Check distance with respect to reference Hit
         Double_t distRef = TMath::Sqrt((hit.GetPosition() - refPos).Mag2());

         if (distRef < distance) {

            continue;

         } else {

            // std::cout<<" Clustering "<<iHit<<" of "<<hitArray->size()<<"\n";
            // std::cout<<" Distance to reference : "<<distRef<<"\n";
            // std::cout<<" Reference position : "<<refPos.X()<<" - "<<refPos.Y()<<" - "<<refPos.Z()<<" -
            // "<<refPos.Mag()<<"\n";

            Double_t clusterQ = 0.0;
            hitTBArray.clear();
            std::copy_if(
               hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
               [&refPos, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - refPos).Mag2()) < radius; });

            // std::cout<<" Clustered "<<hitTBArray.size()<<" Hits "<<"\n";

            if (hitTBArray.size() > 0) {
               double x = 0, y = 0, z = 0;
               std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
               hitCluster->SetClusterID(clusterID);
               Double_t hitQ = 0.0;
               std::for_each(hitTBArray.begin(), hitTBArray.end(), [&x, &y, &z, &hitQ](AtHit &hitInQ) {
                  auto pos = hitInQ.GetPosition();
                  x += pos.X() * hitInQ.GetCharge();
                  y += pos.Y() * hitInQ.GetCharge();
                  z += pos.Z();
                  hitQ += hitInQ.GetCharge();
               });
               x /= hitQ;
               y /= hitQ;
               z /= hitTBArray.size();

               hitCluster->SetCharge(hitQ);
               hitCluster->SetPosition(x, y, z);
               hitCluster->SetTimeStamp(hitTBArray.at(0).GetTimeStamp());
               TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution. Using
                                   // estimations for the moment.
               cov(0, 1) = 0;
               cov(1, 2) = 0;
               cov(2, 0) = 0;
               cov(0, 0) = 0.04;
               cov(1, 1) = 0.04;
               cov(2, 2) = 0.01;
               hitCluster->SetCovMatrix(cov);
               ++clusterID;
               track.AddClusterHit(hitCluster);
            }
         }

         // Sanity check
         /*std::cout<<" Hits for cluster "<<iHit<<" centered in "<<refPos.X()<<" - "<<refPos.Y()<<"
    -"<<refPos.Z()<<"\n"; for(auto iHits=0;iHits<hitTBArray.size();++iHits)
         {
           TVector3 pos    = hitTBArray.at(iHits).GetPosition();
           double Q = hitTBArray.at(iHits).GetCharge();
           int TB          = hitTBArray.at(iHits).GetTimeStamp();
           std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";

    }
         std::cout<<"=================================================="<<"\n";*/

         refPos = hitArray.at(iHit).GetPosition();

         //} // if distance

      } // for

   } // if array size
}

void AtPATTERN::AtPRA::Clusterize(AtTrack &track)
{
   // std::cout << " ================================================================= "
   //         << "\n";
   // std::cout << " Clusterizing track : " << track.GetTrackID() << "\n";
   std::vector<AtHit> hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   /* for (auto iHits = 0; iHits < hitArray->size(); ++iHits) {
      TVector3 pos = hitArray->at(iHits).GetPosition();
      double Q = hitArray->at(iHits).GetCharge();
      int TB = hitArray->at(iHits).GetTimeStamp();
      std::cout << " Pos : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << " - TB : " << TB
                << " - Charge : " << Q << "\n";
      }*/

   for (auto iTB = 0; iTB < 512; ++iTB) {

      Double_t clusterQ = 0.0;
      hitTBArray.clear();

      std::copy_if(hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
                   [&iTB](AtHit &hit) { return hit.GetTimeStamp() == iTB; });

      if (hitTBArray.size() > 0) {
         double x = 0, y = 0;
         std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
         hitCluster->SetClusterID(clusterID);
         Double_t hitQ = 0.0;
         std::for_each(hitTBArray.begin(), hitTBArray.end(), [&x, &y, &hitQ](AtHit &hit) {
            auto pos = hit.GetPosition();
            x += pos.X() * hit.GetCharge();
            y += pos.Y() * hit.GetCharge();
            hitQ += hit.GetCharge();
         });
         x /= hitQ;
         y /= hitQ;
         hitCluster->SetCharge(hitQ);
         hitCluster->SetPosition(x, y, hitTBArray.at(0).GetPosition().Z());
         hitCluster->SetTimeStamp(hitTBArray.at(0).GetTimeStamp());
         TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution. Using
                             // estimations for the moment.
         cov(0, 1) = 0;
         cov(1, 2) = 0;
         cov(2, 0) = 0;
         cov(0, 0) = 0.04;
         cov(1, 1) = 0.04;
         cov(2, 2) = 0.01;
         hitCluster->SetCovMatrix(cov);
         ++clusterID;
         track.AddClusterHit(hitCluster);
      }
   }

   // Sanity check
   /*std::vector<AtHitCluster> *hitClusterArray = track.GetHitClusterArray();
   std::cout << " Clusterized hits : " << hitClusterArray->size() << "\n";
   for (auto iClusterHits = 0; iClusterHits < hitClusterArray->size(); ++iClusterHits) {
      TVector3 pos = hitClusterArray->at(iClusterHits).GetPosition();
      double clusterQ = hitClusterArray->at(iClusterHits).GetCharge();
      int TB = hitClusterArray->at(iClusterHits).GetTimeStamp();
      std::cout << " Pos : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << " - TB : " << TB
                << " - Charge : " << clusterQ << "\n";
      }*/
}

void AtPATTERN::AtPRA::PruneTrack(AtTrack &track)
{
   std::vector<AtHit> hitArray = track.GetHitArray();

   std::cout << "    === Prunning track : " << track.GetTrackID() << "\n";
   std::cout << "      = Hit Array size : " << hitArray.size() << "\n";

   for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

      try {
         bool isNoise = kNN(hitArray, hitArray.at(iHit), fKNN); // Returns true if hit is an outlier

         if (isNoise) {
            // std::cout<<" Hit "<<iHit<<" flagged as outlier. "<<"\n";
            hitArray.erase(hitArray.begin() + iHit);
         }
      } catch (std::exception &e) {

         std::cout << " AtPRA::PruneTrack - Exception caught : " << e.what() << "\n";
      }
   }

   std::cout << "      = Hit Array size after prunning : " << hitArray.size() << "\n";
}

bool AtPATTERN::AtPRA::kNN(const std::vector<AtHit> &hits, AtHit &hitRef, int k)
{

   std::vector<Double_t> distances;
   distances.reserve(hits.size());

   std::for_each(hits.begin(), hits.end(), [&distances, &hitRef](const AtHit &hit) {
      distances.push_back(TMath::Sqrt((hitRef.GetPosition() - hit.GetPosition()).Mag2()));
   });

   std::sort(distances.begin(), distances.end(), [](Double_t a, Double_t b) { return a < b; });

   Double_t mean = 0.0;
   Double_t stdDev = 0.0;

   if (k > hits.size())
      k = hits.size();

   // Compute mean distance of kNN
   for (auto i = 0; i < k; ++i)
      mean += distances.at(i);

   mean /= k;

   // Compute std dev
   for (auto i = 0; i < k; ++i)
      stdDev += TMath::Power((distances.at(i) - mean), 2);

   stdDev = TMath::Sqrt(stdDev / k);

   // Compute threshold
   Double_t T = mean + stdDev * fStdDevMulkNN;

   return (T < fkNNDist) ? false : true;
}
