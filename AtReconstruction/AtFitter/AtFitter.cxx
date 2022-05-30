#include "AtFitter.h"

#include "AtHit.h"
#include "AtHitCluster.h" // for AtHitCluster
#include "AtTrack.h"

#include <Math/Point3D.h>  // for Cartesian3D, operator-, PositionVector3D
#include <Math/Vector3D.h> // for DisplacementVector3D
#include <TMath.h>
#include <TMatrixDSymfwd.h> // for TMatrixDSym
#include <TMatrixTSym.h>    // for TMatrixTSym

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for operator<<, basic_ostream::operator<<
#include <iterator> // for back_insert_iterator, back_inserter
#include <memory>   // for shared_ptr, __shared_ptr_access, __sha...
#include <utility>  // for pair

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

ClassImp(AtFITTER::AtFitter);

AtFITTER::AtFitter::AtFitter() = default;

AtFITTER::AtFitter::~AtFitter() = default;

std::tuple<Double_t, Double_t> AtFITTER::AtFitter::GetMomFromBrho(Double_t M, Double_t Z, Double_t brho)
{

   const Double_t M_Ener = M * 931.49401 / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   // std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}

Bool_t AtFITTER::AtFitter::FindVertexTrack(AtTrack *trA, AtTrack *trB)
{
   // Determination of first hit distance. NB: Assuming both tracks have the same angle sign
   Double_t vertexA = 0.0;
   Double_t vertexB = 0.0;
   if (trA->GetGeoTheta() * TMath::RadToDeg() < 90) {
      auto iniClusterA = trA->GetHitClusterArray()->back();
      auto iniClusterB = trB->GetHitClusterArray()->back();
      vertexA = 1000.0 - iniClusterA.GetPosition().Z();
      vertexB = 1000.0 - iniClusterB.GetPosition().Z();
   } else if (trA->GetGeoTheta() * TMath::RadToDeg() > 90) {
      auto iniClusterA = trA->GetHitClusterArray()->front();
      auto iniClusterB = trB->GetHitClusterArray()->front();
      vertexA = iniClusterA.GetPosition().Z();
      vertexB = iniClusterB.GetPosition().Z();
   }

   return vertexA < vertexB;
}

Bool_t AtFITTER::AtFitter::MergeTracks(std::vector<AtTrack *> *trackCandSource, std::vector<AtTrack> *trackDest,
                                       Bool_t enableSingleVertexTrack)
{

   Bool_t toMerge = kFALSE;

   Int_t addHitCnt = 0;
   // Find the track closer to vertex
   std::sort(trackCandSource->begin(), trackCandSource->end(),
             [this](AtTrack *trA, AtTrack *trB) { return FindVertexTrack(trA, trB); });
   
   // Track stitching from vertex
   AtTrack *vertexTrack = *trackCandSource->begin();

   if(enableSingleVertexTrack){
     trackDest->push_back(*vertexTrack);
     return true;
   }

   
   // Check if the candidate vertex track was merged
   if (vertexTrack->GetIsMerged())
      return kFALSE;
   else
      vertexTrack->SetIsMerged(kTRUE);

   // If enabled, choose only the track closest to vertex (i.e. first one of the collection of candidates)
   // TODO: Select by number of points

   for (auto it = trackCandSource->begin() + 1; it != trackCandSource->end(); ++it) {
      // NB: These tracks were previously marked to merge. If merging fails they should be discarded.
      AtTrack *trackToMerge = *(it);
      toMerge = kFALSE;

      // Skip trackes flagged as merged
      if (!trackToMerge->GetIsMerged()) {
         trackToMerge->SetIsMerged(kTRUE);
      } else
         continue;

      Double_t endVertexZ = 0.0;
      Double_t iniMergeZ = 0.0;
      std::cout<<" Trying to merge ... "<<"\n";
      std::cout << " Vertex track " << vertexTrack->GetTrackID() << " - Track to Merge " << trackToMerge->GetTrackID()
                << "\n";
      // Check relative position between end and begin of each track using Hit Clusters
      std::cout << " Vertex angle " << vertexTrack->GetGeoTheta() * TMath::RadToDeg() << "\n";
      if (vertexTrack->GetGeoTheta() * TMath::RadToDeg() < 90) {
         auto endClusterVertex = vertexTrack->GetHitClusterArray()->front();
         auto iniClusterMerge = trackToMerge->GetHitClusterArray()->back();
         // Check separation and relative distance
         endVertexZ = 1000.0 - endClusterVertex.GetPosition().Z();
         iniMergeZ = 1000.0 - iniClusterMerge.GetPosition().Z();

         Double_t distance = std::sqrt((iniClusterMerge.GetPosition() - endClusterVertex.GetPosition()).Mag2());
         //std::cout << " Distance between tracks " << distance << "\n";
         //std::cout << " Ini Merge " << iniMergeZ << " - endVertexZ " << endVertexZ << "\n";
         if (((iniMergeZ + 10.0) > endVertexZ) && distance < 200) {
            toMerge = kTRUE;
         }

      } else if (vertexTrack->GetGeoTheta() * TMath::RadToDeg() > 90) {
         auto endClusterVertex = vertexTrack->GetHitClusterArray()->back();
         auto iniClusterMerge = trackToMerge->GetHitClusterArray()->front();
         // Check separation and relative distance
         endVertexZ = endClusterVertex.GetPosition().Z();
         iniMergeZ = iniClusterMerge.GetPosition().Z();
         Double_t distance = std::sqrt((iniClusterMerge.GetPosition() - endClusterVertex.GetPosition()).Mag2());
         // std::cout<<" Distance between tracks "<<distance<<"\n";
         // std::cout<<" Ini Merge "<<iniMergeZ<<" - endVertexZ "<<endVertexZ<<"\n";
         if (((iniMergeZ + 10.0) > endVertexZ) &&
             distance < 100) { // NB: Distance between parts of the backward tracks is more critical
            toMerge = kTRUE;
         }
      }

      if (toMerge) {

	std::cout << " --- Merging Succeeded! Vertex track " << vertexTrack->GetTrackID() << " - Track to Merge "
	         << trackToMerge->GetTrackID() << "\n";
         for (const auto &hit : trackToMerge->GetHitArray()) {

            vertexTrack->AddHit(hit);
            ++addHitCnt;
         }

         // Reclusterize after merging
         vertexTrack->SortHitArrayTime();
         vertexTrack->ResetHitClusterArray();
         ClusterizeSmooth3D(*vertexTrack, 7.5, 15.0); // TODO Pass parameters

         // TODO: Check if phi recalculatio is needed

      } else {
	std::cout << " --- Merging Failed ! Vertex track " << vertexTrack->GetTrackID() << " - Track to Merge "
	         << trackToMerge->GetTrackID() << "\n";
      }
   }

   trackDest->push_back(*vertexTrack);

   return toMerge;
}

void AtFITTER::AtFitter::ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius)
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

                  XYZPoint clustPos(x, y, z);
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

void AtFITTER::AtFitter::MergeTracks(std::vector<AtTrack> *trackCandSource, std::vector<AtTrack> *trackJunkSource,
                                     std::vector<AtTrack> *trackDest, bool fitDirection, bool simulationConv)
{
   // DEPRECATED
   // Track destination are the merged tracks.
   // Track candidate source are the main tracks identified as candidates.
   // Track junk source are the tracks from which clusters will be extracted. Boundary conditions are applied: Proximity
   // in space, angle and center.
   // NB: Only works for backward tracks

   Double_t trackDist = 20.0;    // Distance between clusters in mm
   Double_t angleSpread = 5.0;   // Maximum angular spread between clusters
   Double_t centerSpread = 20.0; // Maximim distance between track centers for merging.
   Int_t minClusters = 3;
   Int_t trackSize = 0;

   for (auto trackCand : *trackCandSource) {
      Double_t thetaCand = trackCand.GetGeoTheta();
      auto hitArrayCand = trackCand.GetHitArray();
      std::pair<Double_t, Double_t> centerCand = trackCand.GetGeoCenter();

      AtTrack track = trackCand;
      Int_t jnkCnt = 0;
      Int_t jnkHitCnt = 0;

      if (simulationConv) {
         thetaCand = 180.0 - thetaCand * TMath::RadToDeg();

      } else {
         thetaCand = thetaCand * TMath::RadToDeg();
      }

      for (auto trackJunk : *trackJunkSource) {
         Double_t thetaJunk = trackJunk.GetGeoTheta();
         auto hitArrayJunk = trackJunk.GetHitArray();
         std::pair<Double_t, Double_t> centerJunk = trackJunk.GetGeoCenter();

         if (simulationConv) {

            thetaJunk = 180.0 - thetaJunk * TMath::RadToDeg();
         } else {

            thetaJunk = thetaJunk * TMath::RadToDeg();
         }

         if (thetaCand > 90) {

            /*if ((thetaCand + angleSpread) > thetaJunk && (thetaCand - angleSpread) < thetaJunk) {
                     for (auto hit : *hitArrayJunk) {

                        track.AddHit(&hit);
                        ++jnkHitCnt;
                     }
                }*/

         } else if (thetaCand < 90 && thetaJunk < 90) {

            if ((thetaCand + angleSpread) > thetaJunk && (thetaCand - angleSpread) < thetaJunk) { // Check angle
               std::cout << " Center cand : " << centerCand.first << " - " << centerCand.second << " " << thetaCand
                         << "\n";
               std::cout << " Center junk : " << centerJunk.first << " - " << centerJunk.second << " " << thetaJunk
                         << "\n";
               Double_t centerDistance = TMath::Sqrt(TMath::Power(centerCand.first - centerJunk.first, 2) +
                                                     TMath::Power(centerCand.second - centerJunk.second, 2));
               std::cout << " Distance " << centerDistance << "\n";
               if (centerDistance < 50) // Check quadrant

                  for (const auto &hit : hitArrayJunk) {

                     track.AddHit(hit);
                     ++jnkHitCnt;
                  }
            }
         }

         ++jnkCnt;

      } // Junk track

      // track.SortClusterHitArrayZ();
      track.SortHitArrayTime();

      // Prune if other tracks were added track

      if (trackJunkSource->size() > 0) {
         Double_t pruneFraction = 10.0;
         if (jnkHitCnt > hitArrayCand.size())
            pruneFraction = 4.0; // 25%
         else
            pruneFraction = 10.0; // 10%

         Int_t numHits = (Int_t)track.GetHitArray().size() / pruneFraction;
         for (auto iHit = 0; iHit < numHits; ++iHit)
            track.GetHitArray().pop_back();
      }

      trackDest->push_back(track);

   } // Source track
}
