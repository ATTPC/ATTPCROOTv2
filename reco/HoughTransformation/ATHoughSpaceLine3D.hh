#ifndef ATHOUGHSPACELINE3D_H
#define ATHOUGHSPACELINE3D_H

#include "ATHoughSpace.hh"
#include "TH2F.h"
#include "TF1.h"
#include "TGraph2D.h"
#include <Math/Vector3D.h>
#include <Math/Functor.h>
#include <Math/Minimizer.h>
#include <TPolyLine3D.h>

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//ATTPCROOT
#include "ATTrack.hh"
#include "mmprivate.h"
#undef BLOCKSIZE
// Needed to avoid clash with FLANN library

//PCL
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/organized.h>
#include <pcl/search/impl/organized.hpp>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/impl/filter_indices.hpp>


class ATHoughSpaceLine3D : public ATHoughSpace{

  public:
	 ATHoughSpaceLine3D();
  ~ATHoughSpaceLine3D();

  TH2F* GetHoughSpace(TString ProjPlane);
  void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane);
  void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane);
  void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
  void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane,multiarray PadCoord);
  void CalcMultiHoughSpace(ATEvent* event);
  void CalcHoughSpace(ATEvent* event);


  ClassDef(ATHoughSpaceLine3D, 1);

};

#endif
