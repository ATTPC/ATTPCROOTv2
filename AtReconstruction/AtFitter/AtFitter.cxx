#include "AtFitter.h"

#include "AtHit.h"
#include "AtHitCluster.h" // for AtHitCluster
#include "AtTrack.h"

#include <Math/Point3D.h>  // for Cartesian3D, operator-, PositionVector3D
#include <Math/Vector3D.h> // for DisplacementVector3D
#include <TMath.h>

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for operator<<, basic_ostream::operator<<
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
                                       Bool_t enableSingleVertexTrack, Double_t clusterRadius, Double_t clusterDistance)
{

   Bool_t toMerge = kFALSE;

   Int_t addHitCnt = 0;
   // Find the track closer to vertex
   std::sort(trackCandSource->begin(), trackCandSource->end(),
             [this](AtTrack *trA, AtTrack *trB) { return FindVertexTrack(trA, trB); });

   // Track stitching from vertex
   AtTrack *vertexTrack = *trackCandSource->begin();

   if (enableSingleVertexTrack) {

      // Mark all tracks as merged
      for (auto track : *trackCandSource)
         track->SetIsMerged(kTRUE);

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
      std::cout << " Trying to merge ... "
                << "\n";
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
         // std::cout << " Distance between tracks " << distance << "\n";
         // std::cout << " Ini Merge " << iniMergeZ << " - endVertexZ " << endVertexZ << "\n";
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
         fTrackTransformer->ClusterizeSmooth3D(
            *vertexTrack, clusterRadius,
            clusterDistance); // NB: It can be removed if we force reclusterization for any track in the mina program

         // TODO: Check if phi recalculatio is needed

      } else {
         std::cout << " --- Merging Failed ! Vertex track " << vertexTrack->GetTrackID() << " - Track to Merge "
                   << trackToMerge->GetTrackID() << "\n";
      }
   }

   trackDest->push_back(*vertexTrack);

   return toMerge;
}
[[deprecated]] void
AtFITTER::AtFitter::MergeTracks(std::vector<AtTrack> *trackCandSource, std::vector<AtTrack> *trackJunkSource,
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
