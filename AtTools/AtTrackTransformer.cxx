#include "AtTrackTransformer.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtHit.h"        // for AtHit, AtHit::XYZPoint
#include "AtHitCluster.h" // for AtHitCluster
#include "AtTrack.h"      // for XYZPoint, AtTrack

#include <Math/Point3D.h> // for PositionVector3D, Cart...
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>  // for DisplacementVector3D
#include <TMath.h>          // for Power, Sqrt, ATan2, Pi
#include <TMatrixDSymfwd.h> // for TMatrixDSym
#include <TMatrixTSym.h>    // for TMatrixTSym
#include <TVector3.h>       // for TVector3

#include <algorithm> // for max, for_each, copy_if
#include <iterator>  // for back_insert_iterator
#include <memory>    // for shared_ptr, __shared_p...
#include <vector>    // for vector

AtTools::AtTrackTransformer::AtTrackTransformer() = default;
AtTools::AtTrackTransformer::~AtTrackTransformer() = default;
using XYZPoint = ROOT::Math::XYZPoint;

void AtTools::AtTrackTransformer::ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius)
{
   std::vector<AtHit> hitArray = track.GetHitArrayObject();
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
               timeStamp /= hitTBArray.size();

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
                  hitCluster->SetPosition({x, y, z});
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
                  timeStamp /= hitTBArray.size();

                  sigma_x /= hitQ;
                  sigma_y /= hitQ;
                  sigma_z /= hitTBArray.size();

                  TVector3 clustPos(x, y, z);
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition({x, y, z});
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
