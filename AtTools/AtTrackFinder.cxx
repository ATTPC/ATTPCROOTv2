#include "AtTrackFinder.h"

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
#include <utility> 

bool AtTrackFinder::MergeTracks(std::vector<AtTrack *> *trackCandSource, std::vector<AtTrack> *trackDest,
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

            vertexTrack->AddHit(hit->Clone()); // TODO: Look at code and see if this can be a move instead of a copy
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