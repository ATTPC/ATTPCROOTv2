/*******************************************************************
 * Base class for AtTrackFinderHC                                   *
 * Log: Class started 04-05-2018                                    *
 * Author: Y. Ayyad (ayyad@lbl.gov)                                 *
 ********************************************************************/

#ifndef AtTRACKFINDERHC_H
#define AtTRACKFINDERHC_H

#ifdef _OPENMP
#include <omp.h>
#endif
#include "AtPRA.h" // for AtPRA

#include <Rtypes.h> // for THashConsistencyHolder, ClassDef

#include <pcl/point_cloud.h> // for PointCloud, PointCloud<>::Ptr
#include <pcl/point_types.h> // for PointXYZI

#include "cluster.h" // for Cluster
#include <stdio.h>   // for size_t

#include <memory> // for unique_ptr
#include <vector> // for vector

class AtEvent;
class AtPatternEvent;
class TBuffer;
class TClass;
class TMemberInspector;

namespace hc {
struct triplet;
}

namespace AtPATTERN {
struct hc_params {
   float s;
   size_t k;
   size_t n;
   size_t m;
   float r;
   float a;
   float t;
   float _padding;
};

class AtTrackFinderHC : public AtPRA {
private:
   hc_params inputParams{.s = -1, .k = 19, .n = 3, .m = 8, .r = -1, .a = 0.03, .t = 3.5};

public:
   AtTrackFinderHC();
   ~AtTrackFinderHC() = default;

   std::unique_ptr<AtPatternEvent> FindTracks(AtEvent &event) override;

   void SetScluster(float s) { inputParams.s = s; }
   void SetKtriplet(size_t k) { inputParams.k = k; }
   void SetNtriplet(size_t n) { inputParams.n = n; }
   void SetMcluster(size_t m) { inputParams.m = m; }
   void SetRsmooth(float r) { inputParams.r = r; }
   void SetAtriplet(float a) { inputParams.a = a; }
   void SetTcluster(float t) { inputParams.t = t; }
   void SetPadding(size_t padding) { inputParams._padding = padding; }

private:
   Cluster use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<hc::triplet> triplets, float scale,
                  float cdist, size_t cleanup_min_triplets, int opt_verbose);

   std::unique_ptr<AtPatternEvent>
   clustersToTrack(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Cluster const cluster, AtEvent &event);

   void eventToClusters(AtEvent &event, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

   ClassDefOverride(AtTrackFinderHC, 1);
};

} // namespace AtPATTERN

#endif
