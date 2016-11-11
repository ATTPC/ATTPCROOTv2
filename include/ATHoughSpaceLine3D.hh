//
// Implementation of the Hough transform for 3D line detection
// as described in
//
//   M. Jeltsch, C. Dalitz, R. Pohle-Fröhlich:
//   "Hough Parameter Space Regularisation for Line Detection in 3D."
//   Int. Conf. on Computer Vision Theory and Applications (VISAPP),
//   pp. 345-352, 2016
//
//   Author & Copyright: Manuel Jeltsch, 2016
//
//   Adapted for ATTPCROOT by Yassid Ayyad (NSCL) ayyadlim@nscl.msu.edu
//


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


#include <math.h>
#include <iostream>

#define AZIMUTH_ELEVATION   0
//#define HOUGH_OCTAHEDRON    1
//#define HOUGH_ICOSAHEDRON   2
#define HOUGH_FIBONACCI     3

#define HOUGH_OCTAHEDRON 0
#define HOUGH_ICOSAHEDRON 1

struct vector3D {
    float x;
    float y;
    float z;
    //double theta;
    //double phi;
    bool operator==(const vector3D &rhs) {
        if(x != rhs.x) {
            return false;
        }
        if(y != rhs.y) {
            return false;
        }
        if(z != rhs.z) {
            return false;
        }
        return true;
    }
};


class ATHoughSpaceLine3D : public ATHoughSpace{

  public:
	 ATHoughSpaceLine3D();
  ~ATHoughSpaceLine3D();

  TH2F* GetHoughSpace(TString ProjPlane);
  void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane);
  void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane);
  void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
  void CalcHoughSpace(ATEvent* event, TH2Poly* hPadPlane,const multiarray& PadCoord);
  void CalcMultiHoughSpace(ATEvent* event);
  void CalcHoughSpace(ATEvent* event);

  class Sphere {
  #define GOLDEN_RATIO 1.61803399
  public:
      std::vector<vector3D> vertices;
      std::deque<unsigned int> triangles;

      std::vector<double> sin_phi;
      std::vector<double> cos_phi;
      std::vector<double> sin_theta;
      std::vector<double> cos_theta;

      //void fromAzimuthAndElevation_full(double step);
      void fromAzimuthAndElevation_CosCorrected(double step);
      void fromAzimuthAndElevationUniqueHalfSphere(double step);
      void fromIcosahedron(int subDivisions);
      void fromIcosahedron_full(int subDivisions);
      void fromOctahedron(int subDivisions);
      void fromOctahedron_full(int subDivisions);
      void fromFibonacciSphere(int n);
      void fromFibonacciSphere_full(int n);
      void writeToGnuplotFile(std::string filename);
      void show();

  private:
      void getOctahedron();
      void getIcosahedron();
      void precomputeSinCos(double min_phi, double max_phi, double min_theta, double max_theta, double step);
      void subDivide();
      void makeUnique();


  };

  static void lineTransform3D_fibonacciSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines,
     unsigned int threshold = 1, unsigned int n_directionalVectors = 20400, double step_plane = 1, unsigned int max_n = 0);

  static void lineTransform3D_azimuthElevation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines,
     unsigned int threshold = 1, double step_b = M_PI / 45, double step_plane = 1, unsigned int max_n = 0);

  static void lineTransform3D_Tesselation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines,
    unsigned int threshold = 1, unsigned char platonic_solid = HOUGH_ICOSAHEDRON, unsigned char granularity_b = 3, double step_plane = 20, unsigned int max_n = 10);

  static void lineTransform3D_Tesselation_weighted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> &weights, std::vector<std::pair<vector3D, vector3D> > &lines,
    unsigned int threshold = 1, unsigned char platonic_solid = HOUGH_ICOSAHEDRON, unsigned char granularity_b = 4, double step_plane = 1, unsigned int max_n = 0);

private:

   static void lineTransform3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<std::pair<vector3D, vector3D> > &lines,
     unsigned int threshold, Sphere *sphere, double step_plane, unsigned int max_n);

   static void lineTransform3D_weighted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<double> &weights, std::vector<std::pair<vector3D, vector3D> > &lines,
     unsigned int threshold, Sphere *sphere, double step_plane, unsigned int max_n);

   static void nonmaximumSupression_AE(std::vector<unsigned int> &VotingSpace, Sphere *sphere, unsigned int num_b, unsigned int num_plane, int n, int k);

   static void nonmaximumSupression(std::vector<unsigned int> &VotingSpace, std::vector<vector3D> &vertices, std::deque<unsigned int> &triangles,
     unsigned int num_b, unsigned int num_plane, int n, int r = 1);

   static void nonmaximumSupression(std::vector<double> &VotingSpace, std::vector<vector3D> &vertices, std::deque<unsigned int> &triangles,
     unsigned int num_b, unsigned int num_plane, int n, int r = 1);

   static double roundToNearest(double num) {
       return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
   }

  ClassDef(ATHoughSpaceLine3D, 1);

};

#endif
