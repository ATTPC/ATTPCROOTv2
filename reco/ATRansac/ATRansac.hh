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
#include "TGraph.h"
#include "TH2.h"
#include "TMath.h"
#include "TApplication.h"
#include "TROOT.h"
#include "TF1.h"
#include "Math/Minimizer.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Fit/Fitter.h"
#include <Math/Vector3D.h>
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"
#include "TVectorD.h"

#include "Math/GenVector/Rotation3D.h"
#include "Math/GenVector/EulerAngles.h"
#include "Math/GenVector/AxisAngle.h"
#include "Math/GenVector/Quaternion.h"
#include "Math/GenVector/RotationX.h"
#include "Math/GenVector/RotationY.h"
#include "Math/GenVector/RotationZ.h"
#include "Math/GenVector/RotationZYX.h"

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

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"


namespace ATRANSACN
{


class ATRansac : public TObject
{

  public:
      ATRansac();
      ~ATRansac();

      void CalcRANSAC(ATEvent *event);
      void CalcRANSACFull(ATEvent *event);
      std::vector<ATTrack*> RansacPCL(ATEvent *event);
      std::vector<ATTrack*> Ransac(std::vector<ATHit>* hits);
      TVector3 GetVertex1();
      TVector3 GetVertex2();
      Double_t GetVertexTime();
      Double_t GetAngleTracks(const ROOT::Math::XYZVector& vec1,const ROOT::Math::XYZVector& vec2);
      std::pair<Int_t,Int_t> GetPairTracksIndex();
      Int_t FindIndexTrack(Int_t index);
      Double_t GetMinimum();//Distance of minumum approach between candidate lines (for the moment only 2)
      TVector3 GetVertexMean();
      Int_t MinimizeTrack(ATTrack* track);
      Int_t MinimizeTrackRPhi(ATTrack* track);
      std::vector<ATTrack> GetTrackCand();
      void SetModelType(int model); //RANSAC Model: Line, plane...
      void SetDistanceThreshold(Float_t threshold); //Distance to RANSAC line
      void SetMinHitsLine(Int_t nhits);
      void SetRPhiSpace(); // For RxPhi Ransac calculation (eventually will be moved into a template, when I have the time...)
      void SetXYCenter(Double_t xc, Double_t yc);
      void SetRANSACPointThreshold(Float_t val);
      void SetVertexTime(Double_t val);

      struct PairedLines
      {
        std::pair<Int_t,Int_t> LinesID;
        std::pair<Double_t,Double_t> AngleZAxis;
        std::pair<Double_t,Double_t> AngleZDet;
        std::pair<Double_t,Double_t> AngleYDet;
        Double_t minDist;
        TVector3 meanVertex;
        Double_t angle;
      };

      std::vector<PairedLines> PLines;
      std::vector<PairedLines> GetPairedLinesArray();

      //ATDigiPar *fPar;
      //FairLogger *fLogger;

  protected:
      static double distance2(double x,double y,double z, const double *p);
      void SetLine(double t, const double *p, double &x, double &y, double &z);
      void FindVertex(std::vector<ATTrack*> tracks);
      Bool_t CheckTrackID(Int_t trackID, std::vector<ATTrack> *trackArray); //Check if Track ID is in the list


      TVector3 fVertex_1;
      TVector3 fVertex_2;
      TVector3 fVertex_mean;
      Double_t fMinimum;
      std::vector<ATTrack> fTrackCand; //Candidate tracks
      Int_t fLineDistThreshold;
      int fRANSACModel;
      Float_t fRANSACThreshold;
      Bool_t fRPhiSpace;
      Double_t fXCenter;
      Double_t fYCenter;
      Float_t fRANSACPointThreshold; //Number of points in percentage
      std::pair<Int_t,Int_t> fVertex_tracks; // ID of the tracks that form the best vertex
      Double_t fVertexTime;
      Double_t fTiltAng; // From parameter file
      Int_t fMinHitsLine; //Minimum number of hits per line

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

      //template<typename T>
      friend inline std::ostream& operator <<(std::ostream &o, const ATRANSACN::ATRansac::PairedLines &pl)
      {

          o <<cGREEN<< " =============================================== "<<std::endl;
          o << " Lines ID : " << pl.LinesID.first << " - " << pl.LinesID.second << std::endl;
          o << " Minimum distance between line : "<<pl.minDist<<std::endl;
          o << " Mean vertex between line - X : "<<pl.meanVertex.X()<<"  - Y : "<<pl.meanVertex.Y()<<"  - Z : "<<pl.meanVertex.Z()<<std::endl;
          o << " Angle with Z solenoid axis - Line 1 : "<<pl.AngleZAxis.first<<"  - Line 2 : "<<pl.AngleZAxis.second<<std::endl;
          o << " Angle with Z detector axis - Line 1 : "<<pl.AngleZDet.first<<"  - Line 2 : "<<pl.AngleZDet.second<<std::endl;
          o << " Angle with Y detector axis - Line 1 : "<<pl.AngleYDet.first<<"  - Line 2 : "<<pl.AngleYDet.second<<std::endl;
          o << " Angle between lines : "<<pl.angle<<cNORMAL<<std::endl;
          return o;
      }



  ClassDef(ATRansac, 1);


};

}//namespace

#endif
