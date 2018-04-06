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

//ATTPCROOT
#include "ATHit.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoQuadrant.hh"
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
  float cloudScaleModifier;
  size_t genTripletsNnKandidates;
  size_t genTripletsNBest;
  size_t cleanupMinTriplets;
  float smoothRadius;
  float genTripletsMaxError;
  float bestClusterDistanceDelta;
  float _padding;
};

class ATTrackFinderHC : public TObject
{

  public:
      ATTrackFinderHC();
      ~ATTrackFinderHC();

      bool FindTracks(ATEvent &event);

  private:

      Cluster use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                   std::vector<hc::triplet> triplets, float scale, float cdist,
                   size_t cleanup_min_triplets, int opt_verbose);

      void clustersToTrack(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Cluster const cluster);

      void eventToClusters(ATEvent& event,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);


      ClassDef(ATTrackFinderHC, 1);

};

#endif
