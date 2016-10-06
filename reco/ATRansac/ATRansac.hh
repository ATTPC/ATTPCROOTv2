/*******************************************************************
* Base class for RANSAC                                            *
* Log: Class started 02-10-2016                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/

#ifndef ATRANSAC_H
#define ATRANSAC_H

#ifdef _OPENMP
#include <omp.h>
#endif

#include <fstream>
#include <iostream>
#include <string>

#include <stdlib.h>
#include <stdio.h>

#include "TH1.h"
#include "TCanvas.h"
#include "TGraph2D.h"
#include "TH2.h"
#include "TMath.h"
#include "TApplication.h"
#include "TROOT.h"
#include "Math/Minimizer.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Fit/Fitter.h"
#include <Math/Vector3D.h>

#include "ATHit.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoQuadrant.hh"
#include "ATDigiPar.hh"
#include "AtTpcMap.h"
#include "ATTrack.hh"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//#ifndef __CINT__ // Boost
//#include <boost/multi_array.hpp>
//#endif //__CINT__

#include "TObject.h"

//#include "mmprivate.h"
//#undef BLOCKSIZE
// Needed to avoid clash with FLANN library
//PCL
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <boost/shared_ptr.hpp>




namespace ATRANSACN
{


class ATRansac : public TObject
{

  public:
      ATRansac();
      ~ATRansac();

      void CalcRANSAC(ATEvent *event);
      std::vector<ATTrack*> RansacPCL(ATEvent *event);
      TVector3 GetVertex1();
      TVector3 GetVertex2();
      Double_t GetMinimum();//Distance of minumum approach between candidate lines (for the moment only 2)
      std::vector<ATTrack> GetTrackCand();
      void SetModelType(int model);
      void SetDistanceThreshold(Float_t threshold);

      struct PairedLines
      {
        std::pair<Int_t,Int_t> LinesID;
        Double_t minDist;
        TVector3 meanVertex;
        Double_t angle;
      };

      std::vector<PairedLines> PLines;

  protected:
      Int_t MinimizeTrack(ATTrack* track);
      static double distance2(double x,double y,double z, const double *p);
      void SetLine(double t, const double *p, double &x, double &y, double &z);
      void FindVertex(std::vector<ATTrack*> tracks);
      Bool_t CheckTrackID(Int_t trackID, std::vector<ATTrack> trackArray); //Check if Track ID is in the list


      TVector3 fVertex_1;
      TVector3 fVertex_2;
      Double_t fMinimum;
      std::vector<ATTrack> fTrackCand; //Candidate tracks (debugging purposes)
      Int_t fLineDistThreshold;
      int fRANSACModel;
      Float_t fRANSACThreshold;

      struct SumDistance2
      {
          TGraph2D * fGraph;

              SumDistance2(TGraph2D * g) : fGraph(g) {}
                  double operator() (const double * par) {
                  assert(fGraph    != 0);
                  double * x = fGraph->GetX();
                  double * y = fGraph->GetY();
                  double * z = fGraph->GetZ();
                  int npoints = fGraph->GetN();
                  double sum = 0;
                  for (int i  = 0; i < npoints; ++i) {
                    double d = distance2(x[i],y[i],z[i],par);
                    sum += d;
                  }
            #ifdef DEBUG
             if (first) std::cout << "point " << i << "\t"
                << x[i] << "\t"
                << y[i] << "\t"
                << z[i] << "\t"
                << std::sqrt(d) << std::endl;
            #endif

            //if (first)
              //std::cout << "Total Initial distance square = " << sum << std::endl;
              //first = false;
              return sum;


              }


      };






  ClassDef(ATRansac, 1);


};

}//namespace

#endif
