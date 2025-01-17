#include "AtTrackFinderTC.h"

#include "AtEvent.h"            // for AtEvent
#include "AtHit.h"              // for AtHit
#include "AtPatternEvent.h"     // for AtPatternEvent
#include "AtTrack.h"            // for AtTrack
#include "AtTrackTransformer.h"
#include "AtFitter.h" // for AtTrackTransformer
#include "AtTrackFinder.h"

#include <Math/Point3D.h> // for PositionVector3D

#include "dnn.h"
#include "graph.h"
#include "option.h"
#include "pointcloud.h"
#include "postprocess.h"
#include "triplet.h" // for triplet, generate_triplets

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for cout, cerr
#include <memory>   // for allocator_traits<>::value_...
#include <utility>  // for move

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
std::vector<AtTrack *> candTrackPool;
std::vector<AtTrack> mergedTrackPool;
Bool_t fEnableSingleVertexTrack = kTRUE;
Double_t fClusterSize = 20.0;


AtPATTERN::AtTrackFinderTC::AtTrackFinderTC() : AtPATTERN::AtPRA() {}

std::unique_ptr<AtPatternEvent> AtPATTERN::AtTrackFinderTC::FindTracks(AtEvent &event)
{
   Opt opt_params;
   int opt_verbose = opt_params.get_verbosity();

   opt_params.set_parameters(inputParams.s, inputParams.k, inputParams.n, inputParams.m, inputParams.r, inputParams.a,
                             inputParams.t);

   PointCloud cloud_xyz;
   eventToClusters(event, cloud_xyz);

   if (cloud_xyz.size() == 0) {
      std::cerr << "[Error] empty cloud " << std::endl;

      return nullptr;
   }

   if (opt_params.needs_dnn()) {
      double dnn = std::sqrt(first_quartile(cloud_xyz));
      if (opt_verbose > 0) {
         std::cout << "AtPATTERN::AtTrackFinderTC - [Info] computed dnn: " << dnn << std::endl;
      }
      opt_params.set_dnn(dnn);
      if (dnn == 0.0) {
         std::cerr << "AtPATTERN::AtTrackFinderTC - [Error] dnn computed as zero. "
                   << "Suggestion: remove doublets, e.g. with 'sort -u'" << std::endl;
         return nullptr;
      }
   }

   // Step 1) smoothing by position averaging of neighboring points
   PointCloud cloud_xyz_smooth;
   smoothen_cloud(cloud_xyz, cloud_xyz_smooth, opt_params.get_r());

   // Step 2) finding triplets of approximately collinear points
   std::vector<triplet> triplets;
   generate_triplets(cloud_xyz_smooth, triplets, opt_params.get_k(), opt_params.get_n(), opt_params.get_a());

   // Step 3) single link hierarchical clustering of the triplets
   cluster_group cl_group;
   compute_hc(cloud_xyz_smooth, cl_group, triplets, opt_params.get_s(), opt_params.get_t(), opt_params.is_tauto(),
              opt_params.get_dmax(), opt_params.is_dmax(), opt_params.get_linkage(), opt_verbose);

   // Step 4) pruning by removal of small clusters ...
   cleanup_cluster_group(cl_group, opt_params.get_m(), opt_verbose);
   cluster_triplets_to_points(triplets, cl_group);
   // .. and (optionally) by splitting up clusters at gaps > dmax
   if (opt_params.is_dmax()) {
      cluster_group cleaned_up_cluster_group;
      for (auto &cl : cl_group) {
         max_step(cleaned_up_cluster_group, cl, cloud_xyz, opt_params.get_dmax(), opt_params.get_m() + 2);
      }
      cl_group = cleaned_up_cluster_group;
   }

   // store cluster labels in points
   add_clusters(cloud_xyz, cl_group, opt_params.is_gnuplot());

   // Post processing
   //process_pointcloud(cloud_xyz, 25, 0);

   // Adapt clusters to AtTrack
   return clustersToTrack(cloud_xyz, cl_group, event);
}

const double tolerance = 3.0;
Bool_t SameTrack(AtTrack *trA, AtTrack *trB)
{
   auto theta0 = trA->GetGeoTheta();
   auto theta = trB->GetGeoTheta();

   if (std::abs((theta0 * TMath::RadToDeg()) - (theta * TMath::RadToDeg())) <= tolerance)
      return true;
   else
      return false;
}

void AtPATTERN::AtTrackFinderTC::eventToClusters(AtEvent &event, PointCloud &cloud)
{
   Int_t nHits = event.GetNumHits();

   for (Int_t iHit = 0; iHit < nHits; iHit++) {
      Point point;
      const AtHit hit = event.GetHit(iHit);
      auto position = hit.GetPosition();
      point.x = position.X();
      point.y = position.Y();
      point.z = position.Z();
      point.SetID(iHit);
      cloud.push_back(point);
   }
}

std::unique_ptr<AtPatternEvent>
AtPATTERN::AtTrackFinderTC::clustersToTrack(PointCloud &cloud, const std::vector<cluster_t> &clusters, AtEvent &event)
{
   std::vector<AtTrack> tracks;
   // std::vector<Point> points = cloud;
   auto points = cloud;

   for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {

      AtTrack track; // One track per cluster

      const std::vector<size_t> &point_indices = clusters[cluster_index];
      if (point_indices.size() == 0)
         continue;

      // add points
      for (auto ind : point_indices) {

         const Point &point = cloud[ind];

         track.AddHit(event.GetHit(point.GetID()));

         // remove current point from vector points
         for (auto p = points.begin(); p != points.end(); p++) {
            if (*p == point) {
               points.erase(p);
               break;
            }
         }

      } // Point indices

      track.SetTrackID(cluster_index);

      fTrackTransformer->ClusterizeSmooth3D(track, fClusterRadius, fClusterDistance);

      if (kSetPrunning)
         PruneTrack(track);

      tracks.push_back(track);

   } // Clusters loop

   std::cout << cRED << " Tracks found " << tracks.size() << cNORMAL << std::endl;

   // Dump noise into pattern event
   auto retEvent = std::make_unique<AtPatternEvent>();
   for (const auto &point : points)
      retEvent->AddNoise(event.GetHit(point.GetID()));

   for (auto &track : tracks) {
      if (track.GetHitArray().size() > 0)
         SetTrackInitialParameters(track);
      //retEvent->AddTrack(std::move(track));
   }

   bool kMergeTracks = false;

   std::vector<AtTrack> mergedTracks;
   if(tracks.size() > 2){

      //const double tolerance = 2.0;
      std::vector<bool> processed(tracks.size(), false);

      for(Int_t numtr = 0; numtr < tracks.size(); numtr++){
         std::cout << "Theta " << tracks.at(numtr).GetGeoTheta() << std::endl;
         if (processed[numtr]) continue;
       
         auto track0 = tracks.at(numtr);  
         auto hitArray0 = track0.GetHitArrayObject();
         auto theta0 = track0.GetGeoTheta();

         AtTrack newTrack;

         for(auto &hit : hitArray0){
            newTrack.AddHit(hit);
                           
         }
                     
         if (numtr + 1 < tracks.size()) {
                           
            for (size_t tr = numtr + 1; tr < tracks.size(); ++tr) {
               if (processed[tr]) continue;

               auto track = tracks.at(tr);
               auto hitArray = track.GetHitArrayObject();
               auto theta = track.GetGeoTheta();

               if (std::abs((theta0 * TMath::RadToDeg()) - (theta * TMath::RadToDeg())) <= tolerance) {
                  for(auto &hit : hitArray){
                     newTrack.AddHit(hit);
                                 
                  }
                  processed[tr] = true;
               }

            }  
         }
                        
         processed[numtr] = true;
         fTrackTransformer->ClusterizeSmooth3D(newTrack, fClusterRadius, fClusterDistance);             
         mergedTracks.push_back(newTrack);

      }

      kMergeTracks = true;  
   }

   if(kMergeTracks){
    std::swap(tracks, mergedTracks);
   }

   for (auto &track : tracks) {
      if (track.GetHitArray().size() > 0)
         SetTrackInitialParameters(track);
      retEvent->AddTrack(std::move(track));
   }



   return retEvent;
}





//First try of merging tracks
 /*  bool kMergeTracks = false;

   if(tracks.size() > 2){

      const double tolerance = 2.0;
      std::vector<bool> processed(tracks.size(), false);

      for(Int_t numtr = 0; numtr < tracks.size(); numtr++){
         std::cout << "Theta " << tracks.at(numtr).GetGeoTheta() << std::endl;
         if (processed[numtr]) continue;
       
         auto track0 = tracks.at(numtr);  
         auto hitArray0 = track0.GetHitArrayObject();
         auto theta0 = track0.GetGeoTheta();

         AtTrack newTrack;

         for(auto &hit : hitArray0){
            newTrack.AddHit(hit);
                           
         }
                     
         if (numtr + 1 < tracks.size()) {
                           
            for (size_t tr = numtr + 1; tr < tracks.size(); ++tr) {
               if (processed[tr]) continue;

               auto track = tracks.at(tr);
               auto hitArray = track.GetHitArrayObject();
               auto theta = track.GetGeoTheta();

               if (std::abs((theta0 * TMath::RadToDeg()) - (theta * TMath::RadToDeg())) <= tolerance) {
                  for(auto &hit : hitArray){
                     newTrack.AddHit(hit);
                                 
                  }
                  processed[tr] = true;
               }

            }  
         }
                        
         processed[numtr] = true;
         //fTrackTransformer->ClusterizeSmooth3D(newTrack, fClusterRadius, fClusterDistance);             
         mergedTracks.push_back(newTrack);

      }

      kMergeTracks = true;  
   }

   if(kMergeTracks){
    std::swap(tracks, mergedTracks);
   }
   */

  /*
   auto sp = std::unique_ptr<AtTrack[]>(new AtTrack[tracks.size()]);


   for (auto iTrack = 0; iTrack < tracks.size(); ++iTrack) {
      sp[iTrack] = tracks.at(iTrack);
      candTrackPool.push_back(std::move(&sp[iTrack]));
      
   }
   
   std::vector<AtTrack *> candToMergePool;
   AtTrackFinder merger;
   for (auto itA = candTrackPool.begin(); itA != candTrackPool.end(); ++itA) {
      if(candTrackPool.size() == 0) break;
      AtTrack *trA = *(itA);
      if(candTrackPool.size() == 1){
         mergedTrackPool.push_back(*trA);
         break;   
      } 
      candToMergePool.clear();
      
      auto itB = std::copy_if(itA + 1, candTrackPool.end(), std::back_inserter(candToMergePool),
                                 [&trA, this](AtTrack *track) { return SameTrack(trA, track); });
      
                                std::cout << "candToMergePool size after copy_if: " << candToMergePool.size() << std::endl;  
      
      if (candToMergePool.size() > 0) { // Merge if matches are found
         candToMergePool.push_back(trA);
         Bool_t merged = merger.MergeTracks(&candToMergePool, &mergedTrackPool, fEnableSingleVertexTrack, fClusterRadius,
                                        fClusterSize);

            std::cout << "Merged: " << merged << std::endl;
            std::cout << "candTrackPool size before erase: " << candTrackPool.size() << std::endl;

         itA = candTrackPool.erase(std::remove_if(itA, candTrackPool.end(),
                  [&trA, this](AtTrack *track) { return SameTrack(trA, track); }),
         candTrackPool.end());

         } else {
            mergedTrackPool.push_back(*trA);
                    itA = candTrackPool.erase(itA); // Erase the track from the pool
         }

   }
   
   std::swap(tracks, mergedTrackPool);

   candToMergePool.clear();
   candTrackPool.clear();
   //mergedTrackPool.clear();

 /*  for (auto &track : tracks) {
      if (track.GetHitArray().size() > 0)
         SetTrackInitialParameters(track);
      retEvent->AddTrack(std::move(track));
   }*/
   //std::cout << "Number of tracks: " << tracks.size() << std::endl;

  
  