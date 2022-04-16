#ifndef SAMAIN_H
#define SAMAIN_H

// STD
#include <ios>
#include <iostream>
#include <istream>
#include <limits>
#include <map>
#include <vector>

// FairROOT
#include <FairLogger.h>
#include <FairParAsciiFileIo.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

// ATTPCROOT
#include "ATEvent.hh"
#include "ATHDFParserTask.hh"
#include "ATHit.hh"
#include "ATHoughSpace.hh"
#include "ATHoughSpaceCircle.hh"
#include "ATHoughSpaceLine.hh"
#include "ATPSATask.hh"
#include "ATPad.hh"
#include "ATTrack.hh"
#include "AtTpcPoint.h"

// PCL
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/sac_segmentation.h>

// ROOT
#include <TApplication.h>
#include <TArrayD.h>
#include <TAxis.h>
#include <TCanvas.h>
#include <TClonesArray.h>
#include <TF1.h>
#include <TFile.h>
#include <TGraph.h>
#include <TH1F.h>
#include <TMath.h>
#include <TMatrixD.h>
#include <TRegexp.h>
#include <TRotation.h>
#include <TStopwatch.h>
#include <TString.h>
#include <TSystem.h>
#include <TTree.h>
#include <TTreePlayer.h>
#include <TTreeReader.h>
#include <TTreeReaderValue.h>
#include <TVectorD.h>

#include "Fit/Fitter.h"
#include "Math/Factory.h"
#include "Math/Functor.h"
#include "Math/GenVector/AxisAngle.h"
#include "Math/GenVector/EulerAngles.h"
#include "Math/GenVector/Quaternion.h"
#include "Math/GenVector/Rotation3D.h"
#include "Math/GenVector/RotationX.h"
#include "Math/GenVector/RotationY.h"
#include "Math/GenVector/RotationZ.h"
#include "Math/GenVector/RotationZYX.h"
#include "Math/Minimizer.h"

int MinimizeTrack(ATTrack *track);
static double distance2(double x, double y, double z, const double *p);
void SetLine(double t, const double *p, double &x, double &y, double &z);

struct SumDistance2 {
   TGraph2D *fGraph;

   SumDistance2(TGraph2D *g) : fGraph(g) {}
   double operator()(const double *par)
   {
      assert(fGraph != 0);
      double *x = fGraph->GetX();
      double *y = fGraph->GetY();
      double *z = fGraph->GetZ();
      int npoints = fGraph->GetN();
      double sum = 0;
      for (int i = 0; i < npoints; ++i) {
         double d = distance2(x[i], y[i], z[i], par);
         sum += d;
      }
#ifdef DEBUG
      if (first)
         std::cout << "point " << i << "\t" << x[i] << "\t" << y[i] << "\t" << z[i] << "\t" << std::sqrt(d)
                   << std::endl;
#endif

      // if (first)
      // std::cout << "Total Initial distance square = " << sum << std::endl;
      // first = false;
      return sum;
   }
};

#endif
