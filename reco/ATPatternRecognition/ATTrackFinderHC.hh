/*******************************************************************
* Base class for ATTrackFinderHC                                   *
* Log: Class started 04-05-2018                                    *
* Author: Y. Ayyad (ayyad@lbl.gov)                                 *
********************************************************************/

#ifndef ATTRACKFINDERHC_H
#define ATTRACKFINDERHC_H

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

//System
#include <fstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <future>

//ATTPCROOT
#include "ATPRA.hh"
#include "ATHit.hh"
#include "ATEvent.hh"
#include "ATPatternEvent.hh"
#include "ATDigiPar.hh"
#include "AtTpcMap.h"
#include "ATTrack.hh"
#include "TObject.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//PCL
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

//trackfinder
#include "hc.hh"
#include "msd.hh"
#include "smoothenCloud.hh"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

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

  Point(pcl::PointXYZ point) {
    x = point.x;
    y = point.y;
    z = point.z;
  }

  bool operator==(const Point &p) const {
    return (x == p.x && y == p.y && z == p.z);
  }
};

namespace ATPATTERN{


class ATTrackFinderHC : public ATPRA
{

  public:
      ATTrackFinderHC();
      ~ATTrackFinderHC();

      bool FindTracks(ATEvent &event, ATPatternEvent *patternEvent);
      std::vector<ATTrack> GetTrackCand();

  private:

      Cluster use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                   std::vector<hc::triplet> triplets, float scale, float cdist,
                   size_t cleanup_min_triplets, int opt_verbose);

      std::vector<ATTrack> clustersToTrack(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                          Cluster const cluster, ATEvent& event);

      void eventToClusters(ATEvent& event,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

      std::vector<ATTrack> fTrackCand; //Candidate tracks


      ClassDef(ATTrackFinderHC, 1);

};

}//namespace

#endif
