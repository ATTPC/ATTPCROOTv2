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


void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){

  //Energy per nucleon
  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=pow(X,2);
  X=2.*AM*X;
  X=X+pow(AM,2);
  E=TMath::Sqrt(X)-AM;

}


int
main(int argc, char** argv)
{

  TApplication app("app",&argc,argv);

  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

  std::ifstream file;
  file.open("../event_7.dat");

  //events 1 and 19 - 3.3 MeV 137 deg
  // events 2 and 6 - 9.9 MeV 60 deg

  double angle = 25.57;

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

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (6.0);

  seg.setInputCloud (cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  double bro = 2.0*coefficients->values[2]/TMath::Sin(angle*TMath::Pi()/180.0)/1000.0;                                    
  double ener = 0;

  GetEnergy(1.0,1.0,bro,ener);

  std::cout<<" Energy "<<ener<<"\n";                                      

  TGraph *hitPattern = new TGraph();

  std::vector<double> wpca;
  std::vector<double> whit;


  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i){
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;

     //wpca.push_back( (cloud->points[inliers->indices[i]].y -  coefficients->values[1])/ )                                          


     hitPattern->SetPoint(hitPattern->GetN(),cloud->points[inliers->indices[i]].x,cloud->points[inliers->indices[i]].y );  


  }



  hitPatternOrigin->SetMarkerStyle(20);
  hitPatternOrigin->SetMarkerColor(kBlue);
  hitPatternOrigin->SetMarkerSize(1.2);
  hitPatternOrigin->Draw("ap");
  
  hitPattern->SetMarkerStyle(20);
  hitPattern->SetMarkerColor(kRed);
  hitPattern->SetMarkerSize(1.2);
  hitPattern->Draw("p");



  app.Run();

 
  return 0;
 }