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

// System
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <future>

// AtTPCROOT
#include "AtPRA.h"
#include "AtHit.h"
#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtDigiPar.h"
#include "AtTpcMap.h"
#include "AtTrack.h"
#include <TObject.h>

// FairRoot classes
#include <FairRootManager.h>
#include <FairLogger.h>

// PCL
#include <pcl/common/common.h>

// trackfinder
#include "hc.h"
#include "msd.h"
#include "smoothenCloud.h"

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

struct Point {
   float x;
   float y;
   float z;
   std::vector<int> clIds;

   template <typename T>
   Point(T point)
   {
      x = point.x;
      y = point.y;
      z = point.z;
   }

   bool operator==(const Point &p) const { return (x == p.x && y == p.y && z == p.z); }
};

namespace AtPATTERN {

class AtTrackFinderHC : public AtPRA {

public:
   AtTrackFinderHC();
   ~AtTrackFinderHC() = default;

   bool FindTracks(AtEvent &event, AtPatternEvent *patternEvent);
   std::vector<AtTrack> GetTrackCand();
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

   std::vector<AtTrack>
   clustersToTrack(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Cluster const cluster, AtEvent &event);

   void eventToClusters(AtEvent &event, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

   std::vector<AtTrack> fTrackCand; // Candidate tracks

   hc_params inputParams{.s = -1, .k = 19, .n = 3, .m = 8, .r = -1, .a = 0.03, .t = 3.5};

   ClassDef(AtTrackFinderHC, 1);
};

} // namespace AtPATTERN

#endif
