/*******************************************************************
* Base class for Hough Space transformation for the ATTPCROOT      *
* Log: Class started 28-04-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/


#ifndef ATHOUGHSPACE_H
#define ATHOUGHSPACE_H


#include "ATHit.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoQuadrant.hh"
#include "ATDigiPar.hh"
#include "AtTpcMap.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//ROOT classes
#include "TClonesArray.h"
#include "TH2F.h"
#include "TH2Poly.h"
#include "TMath.h"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__

#include "TObject.h"


/*#include "mmprivate.h"
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
#include <pcl/filters/impl/filter_indices.hpp>*/

class ATHoughSpace : public TObject
{

	public:

		ATHoughSpace();
		virtual ~ATHoughSpace();

		typedef boost::multi_array<double,3> multiarray;
		typedef multiarray::index index;

         virtual TH2F* GetHoughSpace(TString ProjPlane)=0;
	       virtual void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane)=0;
				 virtual void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane)=0;
				 virtual void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4)=0;
				 virtual void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane,multiarray PadCoord)=0;
				 virtual void CalcMultiHoughSpace(ATEvent* event)=0;
				 virtual void CalcHoughSpace(ATEvent* event)=0; //General Main Hough Space function

				 void SetThreshold(Double_t value);
				 void SetHoughDistance(Double_t value);

   protected:


		  Double_t fThreshold;
			Double_t fHoughDist;
			Double_t fHoughMaxThreshold;


		ClassDef(ATHoughSpace, 2);

};

#endif
