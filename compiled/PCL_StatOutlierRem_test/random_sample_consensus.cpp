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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <fstream>

// ROOT
#include "TGraph.h"
#include "TCanvas.h"
#include "TApplication.h"
#include "TMath.h"
#include "TF1.h"
#include "TAxis.h"
//#include "AtHit.h"

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E)
{

   // Energy per nucleon
   Float_t AM = 931.5;
   Float_t X = BRO / 0.1439 * IZ / M;
   X = pow(X, 2);
   X = 2. * AM * X;
   X = X + pow(AM, 2);
   E = TMath::Sqrt(X) - AM;
}

int main(int argc, char **argv)
{

   TApplication app("app", &argc, argv);

   Int_t opt1;
   double opt2;

   if (argc == 4) {

      opt1 = std::atoi(argv[1]);
      opt2 = std::stod(argv[2]);

   } else {

      std::cout << " Wrong number of arguments. Expecting 3."
                << "\n";
      std::cout << " i.e. random _sample_consensus 10 0.1 -s "
                << "\n";
      return 0;
   }

   std::cout << " Opt 1 : " << opt1 << " - Opt 2 : " << opt2 << "\n";

   // initialize PointClouds
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

   std::ifstream file;
   file.open("../event_87.dat");

   std::string line_buffer;

   float x, y, z, A;
   int TB;
   int i = 0;
   int nPoints = 0;

   cloud->width = 10000;
   cloud->height = 1;
   cloud->is_dense = false;
   cloud->points.resize(cloud->width * cloud->height);

   TGraph *hitPatternOrigin = new TGraph();

   while (std::getline(file, line_buffer)) {
      std::istringstream ss_line(line_buffer);
      ss_line >> x >> y >> z >> TB >> A;
      std::cout << x << "   " << y << "  " << z << "   "
                << "\n";
      if ((x < 300 && x > -300) && (y < 300 && y > -300) && (z < 2000 && z > -2000)) {

         cloud->points[i].x = x;
         cloud->points[i].y = y;
         cloud->points[i].z = z;
         hitPatternOrigin->SetPoint(hitPatternOrigin->GetN(), cloud->points[i].x, cloud->points[i].y);
         ++i;
      }
   }

   i /= 1;

   cloud->points.resize(i * cloud->height);

   std::cout << " Cloud size " << cloud->size() << "\n";
   pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
   pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;

   if (strcmp(argv[3], "-s") == 0) {

      std::cout << " Statistical Outlier Removal "
                << "\n";
      sor.setInputCloud(cloud);
      sor.setMeanK(opt1);
      sor.setStddevMulThresh(opt2);
      sor.filter(*cloud_filtered);

   } else if (strcmp(argv[3], "-r") == 0) {

      std::cout << " Radius Outlier Removal "
                << "\n";
      outrem.setInputCloud(cloud);
      outrem.setRadiusSearch(opt2);
      outrem.setMinNeighborsInRadius(opt1);
      outrem.setKeepOrganized(true);
      outrem.filter(*cloud_filtered);
   }

   std::cout << " Filtered Cloud size " << cloud_filtered->size() << "\n";

   /*  std::cerr << "Cloud before filtering: " << std::endl;
   for (const auto& point: *cloud)
     std::cerr << "    " << point.x << " "
                         << point.y << " "
                         << point.z << std::endl;
   // display pointcloud after filtering
   std::cerr << "Cloud after filtering: " << std::endl;
   for (const auto& point: *cloud_filtered)
     std::cerr << "    " << point.x << " "
                         << point.y << " "
                         << point.z << std::endl;*/

   TGraph *hitPatternFiltered = new TGraph();
   TGraph *hitPatternFilteredXZ = new TGraph();

   for (auto iFil = 0; iFil < cloud_filtered->size(); ++iFil) {
      hitPatternFiltered->SetPoint(hitPatternFiltered->GetN(), cloud_filtered->points[iFil].x,
                                   cloud_filtered->points[iFil].y);
      hitPatternFilteredXZ->SetPoint(hitPatternFilteredXZ->GetN(), cloud_filtered->points[iFil].x,
                                     cloud_filtered->points[iFil].z);
   }

   TCanvas *c1 = new TCanvas("c1", "c1", 700, 500);
   c1->Divide(2, 2);
   c1->cd(1);
   hitPatternFiltered->SetMarkerStyle(20);
   hitPatternFiltered->SetMarkerColor(kRed);
   hitPatternFiltered->SetMarkerSize(1.2);
   hitPatternFiltered->GetXaxis()->SetTitle("X (mm)");
   hitPatternFiltered->GetYaxis()->SetTitle("Y (mm)");
   hitPatternFiltered->Draw("ap");
   c1->cd(2);
   hitPatternOrigin->SetMarkerStyle(20);
   hitPatternOrigin->SetMarkerColor(kBlue);
   hitPatternOrigin->SetMarkerSize(1.2);
   hitPatternOrigin->GetXaxis()->SetTitle("X (mm)");
   hitPatternOrigin->GetYaxis()->SetTitle("Y (mm)");
   hitPatternOrigin->Draw("ap");
   c1->cd(3);
   hitPatternFilteredXZ->SetMarkerStyle(20);
   hitPatternFilteredXZ->SetMarkerColor(kRed);
   hitPatternFilteredXZ->SetMarkerSize(1.2);
   hitPatternFilteredXZ->GetXaxis()->SetTitle("X (mm)");
   hitPatternFilteredXZ->GetYaxis()->SetTitle("Y (mm)");
   hitPatternFilteredXZ->Draw("ap");

   app.Run();

   return 0;
}
