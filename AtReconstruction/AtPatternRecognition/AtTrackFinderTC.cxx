#include "AtTrackFinderTC.h"

#include "AtEvent.h"        // for AtEvent
#include "AtHit.h"          // for AtHit
#include "AtPatternEvent.h" // for AtPatternEvent
#include "AtTrack.h"        // for AtTrack

#include <Math/Point3D.h> // for PositionVector3D

#include <boost/core/checked_delete.hpp>  // for checked_delete
#include <boost/smart_ptr/shared_ptr.hpp> // for shared_ptr

#include "dnn.h"
#include "graph.h"
#include "option.h"
#include "output.h"
#include "pointcloud.h"

#include <algorithm>
#include <cmath>    // for sqrt
#include <iostream> // for cout, cerr
#include <memory>   // for allocator_traits<>::value_...
#include <utility>  // for move

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

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

      return NULL;
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
         return NULL;
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
      for (cluster_group::iterator cl = cl_group.begin(); cl != cl_group.end(); ++cl) {
         max_step(cleaned_up_cluster_group, *cl, cloud_xyz, opt_params.get_dmax(), opt_params.get_m() + 2);
      }
      cl_group = cleaned_up_cluster_group;
   }

   // store cluster labels in points
   add_clusters(cloud_xyz, cl_group, opt_params.is_gnuplot());

   // Adapt clusters to AtTrack
   return clustersToTrack(cloud_xyz, cl_group, event);
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
   std::vector<Point> points = cloud;

   for (size_t cluster_index = 0; cluster_index < clusters.size(); ++cluster_index) {

      AtTrack track; // One track per cluster

      const std::vector<size_t> &point_indices = clusters[cluster_index];
      if (point_indices.size() == 0)
         continue;

      // add points
      for (std::vector<size_t>::const_iterator it = point_indices.begin(); it != point_indices.end(); ++it) {

         const Point &point = cloud[*it];

         track.AddHit(event.GetHit(point.GetID()));

         // remove current point from vector points
         for (std::vector<Point>::iterator p = points.begin(); p != points.end(); p++) {
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

   std::cout << cRED << " Tracks found " << tracks.size() << cNORMAL << "\n";

   // Dump noise into pattern event
   auto retEvent = std::make_unique<AtPatternEvent>();
   for (const auto &point : points)
      retEvent->AddNoise(event.GetHit(point.GetID()));

   for (auto &track : tracks) {
      if (track.GetHitArray().size() > 0)
         SetTrackInitialParameters(track);
      retEvent->AddTrack(std::move(track));
   }

   return retEvent;
}
