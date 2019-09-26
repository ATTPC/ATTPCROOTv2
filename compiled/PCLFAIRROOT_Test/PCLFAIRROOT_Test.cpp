#include "PCLFAIRROOT_Test.hh"
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <fstream>

//ROOT
#include "TGraph.h"
#include "TCanvas.h"
#include "TApplication.h"
#include "TMath.h"
#include "TF1.h"
#include "TAxis.h"


int
main(int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  std::ifstream file;
  file.open("../event_test.dat");
  std::string line_buffer;

  float x,y,z,A;
  int TB;
  int i = 0;
  int nPoints =0;

  cloud->width    = 1000;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  TGraph *hitPatternOrigin = new TGraph();

  while(std::getline(file,line_buffer))
  {
          std::istringstream ss_line(line_buffer);
          ss_line >> x >> y >> z >> TB >> A;
          cloud->points[i].x = x;
          cloud->points[i].y = y;
          cloud->points[i].z = z;
          hitPatternOrigin->SetPoint(hitPatternOrigin->GetN(),cloud->points[i].x,cloud->points[i].y );
          ++i;

  }

  i/=1;

  cloud->points.resize (i * cloud->height);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(6);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " "
                                      << coefficients->values[3] << std::endl;



}
