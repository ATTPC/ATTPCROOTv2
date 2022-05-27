#include "AtTrackFinderHC.h"

#include "AtEvent.h"        // for AtEvent
#include "AtHit.h"          // for AtHit
#include "AtPatternEvent.h" // for AtPatternEvent
#include "AtTrack.h"        // for AtTrack

#include <Math/Point3D.h> // for PositionVector3D

#include <Eigen/Core>                     // for aligned_allocator
#include <boost/core/checked_delete.hpp>  // for checked_delete
#include <boost/smart_ptr/shared_ptr.hpp> // for shared_ptr
#include <pcl/PointIndices.h>             // for PointIndicesPtr, PointIndices
#include <pcl/common/io.h>                // for copyPointCloud

#include "hc.h"            // for triplet, cleanupClusterGroup
#include "msd.h"           // for first_quartile
#include "smoothenCloud.h" // for smoothenCloud

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for cout, cerr
#include <memory>   // for allocator_traits<>::value_...
#include <utility>  // for move

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

AtPATTERN::AtTrackFinderHC::AtTrackFinderHC() : AtPATTERN::AtPRA() {}

std::unique_ptr<AtPatternEvent> AtPATTERN::AtTrackFinderHC::FindTracks(AtEvent &event)
{

   int opt_verbose = 0;

   hc_params opt_params = inputParams;

   // Parse AtTPCROOT date into PCL format
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(new pcl::PointCloud<pcl::PointXYZI>());

   /// TODO: Convert hit patern
   eventToClusters(event, cloud_xyzti);

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
   pcl::copyPointCloud(*cloud_xyzti, *cloud_xyz);
   if (cloud_xyz->size() == 0) {
      std::cerr << "Error: empty cloud <<"
                   "\n";
      return nullptr;
   }

   // compute default r if it is not given
   if (opt_params.r < 0.0) {
      float dnn = 2.0f * std::sqrt(msd::first_quartile(cloud_xyz));
      opt_params.r = dnn;
      if (opt_verbose > 0) {
         std::cout << "Computed smoothed radius: " << dnn << std::endl;
      }
   }

   // compute default s if it is not given
   if (opt_params.s < 0.0) {
      float dnn = std::sqrt(msd::first_quartile(cloud_xyz)) / 3.0f;
      opt_params.s = dnn;
      if (opt_verbose > 0) {
         std::cout << "Computed distance scale: " << dnn << std::endl;
      }
   }

   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(new pcl::PointCloud<pcl::PointXYZI>());

   cloud_xyzti_smooth = smoothenCloud(cloud_xyzti, opt_params.r, false);

   // calculate cluster
   Cluster cluster;
   std::vector<hc::triplet> triplets;

   triplets = hc::generateTriplets(cloud_xyzti_smooth, opt_params.k, opt_params.n, opt_params.a);

   cluster = use_hc(cloud_xyzti_smooth, triplets, opt_params.s, opt_params.t, opt_params.m, opt_verbose);

   // Adapt clusters to AtTrack
   return clustersToTrack(cloud_xyzti, cluster, event);
}

Cluster AtPATTERN::AtTrackFinderHC::use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                           std::vector<hc::triplet> triplets, float scale, float cdist,
                                           size_t cleanup_min_triplets, int opt_verbose = 0)
{

   hc::ScaleTripletMetric scale_triplet_metric(scale);

   Cluster empty_cluster;

   if (triplets.size() > 5) {
      hc::cluster_group result = hc::compute_hc(cloud, triplets, scale_triplet_metric, cdist, opt_verbose);
      hc::cluster_group const &cleaned_up_cluster_group = hc::cleanupClusterGroup(result, cleanup_min_triplets);
      return hc::toCluster(triplets, cleaned_up_cluster_group, cloud->size());
   } else
      return empty_cluster;
}

void AtPATTERN::AtTrackFinderHC::eventToClusters(AtEvent &event, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
   Int_t nHits = event.GetNumHits();
   cloud->points.resize(nHits);

   for (Int_t iHit = 0; iHit < nHits; iHit++) {

      const AtHit hit = event.GetHit(iHit);
      auto position = hit.GetPosition();
      cloud->points[iHit].x = position.X();
      cloud->points[iHit].y = position.Y();
      cloud->points[iHit].z = position.Z();
      cloud->points[iHit].intensity = iHit; // Storing the position of the hit in the event container
                                            // std::cout<<position.Y()<<"\n";
   }
}

std::unique_ptr<AtPatternEvent> AtPATTERN::AtTrackFinderHC::clustersToTrack(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                                                            Cluster const cluster, AtEvent &event)
{
   std::vector<AtTrack> tracks;

   std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI>> points = cloud->points;

   std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();

   for (size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {

      AtTrack track; // One track per cluster

      pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
      // get color colour

      for (int index : pointIndices->indices) {
         pcl::PointXYZI point = cloud->points[index];

         track.AddHit(event.GetHit(point.intensity));

         // remove clustered points from point-vector
         for (auto it = points.end(); it != points.begin(); --it) {
            if (it->x == point.x && it->y == point.y && it->z == point.z) {

               if (it != points.end()) {
                  points.erase(it);
                  break;
               }
            }
         }

      } // Indices loop

      track.SetTrackID(clusterIndex);
      ClusterizeSmooth3D(track, 15.0, 30.5); // 10.5,20.0
      // Clusterize3D(track, 15.0, 30.5); //10.5,20.0

      if (kSetPrunning)
         PruneTrack(track);

      tracks.push_back(track);

   } // Clusters loop

   std::cout << cRED << " Tracks found " << tracks.size() << cNORMAL << "\n";

   // Dump noise into pattern event
   auto retEvent = std::make_unique<AtPatternEvent>();
   for (const auto &point : points)
      retEvent->AddNoise(event.GetHit(point.intensity));

   for (auto &track : tracks) {
      if (track.GetHitArray().size() > 0)
         SetTrackInitialParameters(track);
      retEvent->AddTrack(std::move(track));
   }
   return retEvent;
   /*ROOT::EnableThreadSafety();

   //Estimaton of track parameters
   std::vector<std::future<void>> futures;
   futures.reserve(10);

   for(auto& track : tracks)
   {

      futures.push_back( std::async(std::launch::async,&AtPRA::SetTrackInitialParameters,this,std::ref(track)) );
      //std::cout<<" Processing track "<<"\n";
      //SetTrackInitialParameters(track);

   }

   for(auto& future : futures) future.wait();

   for(auto& track : tracks)
   {
     std::vector<Double_t>& coeffs = track.GetRANSACCoeff();

     std::cout<<" RANSAC coeff for Track "<<track.GetTrackID()<<"\n";

       for(auto& coeff : coeffs){

    std::cout<<coeff<<"\n";
       }

       }*/
}

ClassImp(AtPATTERN::AtTrackFinderHC)
