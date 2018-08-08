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

  TGraph *hitPattern = new TGraph();

  TGraph *arclengthGraph = new TGraph();

  std::vector<double> wpca;
  std::vector<double> whit;
  std::vector<double> arclength;
 

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i){
    /*std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;*/

     wpca.push_back( TMath::ATan2 ( (cloud->points[inliers->indices[0]].y -  coefficients->values[1]) ,  
      (cloud->points[inliers->indices[0]].x -  coefficients->values[0]) ) );    

     whit.push_back( TMath::ATan2 ( (cloud->points[inliers->indices[i]].y -  coefficients->values[1]) ,  
      (cloud->points[inliers->indices[i]].x -  coefficients->values[0]) ) );

      arclength.push_back(  fabs(coefficients->values[2])*( wpca.at(i) - whit.at(i))  );                                   


     hitPattern->SetPoint(hitPattern->GetN(),cloud->points[inliers->indices[i]].x,cloud->points[inliers->indices[i]].y );  

     arclengthGraph->SetPoint(arclengthGraph->GetN(),arclength.at(i),cloud->points[inliers->indices[i]].z);

     std::cout<<arclength.at(i)<<"  "<<cloud->points[inliers->indices[i]].z<<"  "<<wpca.at(i)<<"  "<<whit.at(i)<<"\n";

  }

  TF1 *f1 = new TF1("f1","pol1",0,500);
  arclengthGraph->Fit(f1,"R");  
  //TF1 *fitfunc = arclengthGraph->GetFunction("pol1");

  Double_t slope = f1->GetParameter(1);
  double angle = (TMath::ATan2(slope,1)*180.0/TMath::Pi());

  std::cout<<" Angle before "<<angle<<"\n";

  if(angle<0) angle=90.0+angle;
  else if(angle>0) angle=90+angle;

  double bro = 2.0*coefficients->values[2]/TMath::Sin(angle*TMath::Pi()/180.0)/1000.0;                                    
  double ener = 0;

  GetEnergy(1.0,1.0,bro,ener);

  std::cout<<" Angle "<<angle<<"\n";
  std::cout<<" Bro "<<bro<<"\n";

  std::cout<<" Energy "<<ener<<"\n"; 

  TCanvas *c1 = new TCanvas("c1","c1",700,500);
  c1->Divide(2,1);
  c1->cd(1);

  hitPatternOrigin->SetMarkerStyle(20);
  hitPatternOrigin->SetMarkerColor(kBlue);
  hitPatternOrigin->SetMarkerSize(1.2);
  hitPatternOrigin->GetXaxis()->SetTitle("X (mm)");
  hitPatternOrigin->GetYaxis()->SetTitle("Y (mm)");
  hitPatternOrigin->Draw("ap");
  
  hitPattern->SetMarkerStyle(20);
  hitPattern->SetMarkerColor(kRed);
  hitPattern->SetMarkerSize(1.2);
  hitPattern->GetXaxis()->SetTitle("X (mm)");
  hitPattern->GetYaxis()->SetTitle("Y (mm)");
  hitPattern->Draw("p");



  c1->cd(2);
  arclengthGraph->SetMarkerStyle(20);
  arclengthGraph->SetMarkerColor(kRed);
  arclengthGraph->SetMarkerSize(1.2);
  arclengthGraph->GetXaxis()->SetTitle("Arclength (mm)");
  arclengthGraph->GetYaxis()->SetTitle("Z (mm)");
  arclengthGraph->Draw("ap");



  app.Run();

 
  return 0;
 }