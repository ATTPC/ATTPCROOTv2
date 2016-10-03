#include "ATRansac.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATRANSACN::ATRansac)

ATRANSACN::ATRansac::ATRansac()
{

}

ATRANSACN::ATRansac::~ATRansac()
{
}


std::vector<ATTrack*> ATRANSACN::ATRansac::RansacPCL(ATEvent *event)
{

    std::vector<ATTrack*> tracks;


    //Data writer
	  pcl::PCDWriter writer;

	  // initialize PointClouds
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);

	  //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGBA>);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

    Int_t nHits = event->GetNumHits();
    cloud->points.resize(nHits);

    for(Int_t iHit=0; iHit<nHits; iHit++){

          ATHit* hit = event->GetHit(iHit);
          Int_t PadNumHit = hit->GetHitPadNum();
          TVector3 position = hit->GetPosition();
          cloud->points[iHit].x = position.X();
 			    cloud->points[iHit].y = position.Y();
			    cloud->points[iHit].z = position.Z();
          cloud->points[iHit].rgb = iHit; // Storing the position of the hit in the event container

    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

	  // Create the segmentation object
  /*	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  	seg.setOptimizeCoefficients(true);
  	seg.setModelType (pcl::SACMODEL_LINE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (1000);
  	seg.setDistanceThreshold (5.0);

    // Create the filtering object
 pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

int i = 0, nr_points = (int) cloud->points.size ();

// While 30% of the original cloud is still there
while (cloud->points.size () > 0.01 * nr_points)
 {
 // Segment the largest planar component from the remaining cloud
 std::cout<<cloud->points.size()<<std::endl;

     seg.setInputCloud (cloud);
     seg.segment(*inliers, *coefficients);
     if (inliers->indices.size () == 0)
     {
       std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
       break;
     }
  // Extract the inliers
     extract.setInputCloud (cloud);
     extract.setIndices (inliers);
     extract.setNegative (false);
     extract.filter (*cloud_p);
     std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
     ATTrack *track = new ATTrack();

          for(Int_t iHit=0;iHit<(cloud_p->width*cloud_p->height);iHit++)
          {
            if(event->GetHit(cloud_p->points[iHit].rgb)) track->AddHit(event->GetHit(cloud_p->points[iHit].rgb));
          }
      tracks.push_back(track);
     //std::stringstream ss;
     //ss << "../track_" << i << ".pcd";
     //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

     // Create the filtering object
     extract.setNegative(true);
     extract.filter(*cloud_f);
     cloud.swap(cloud_f);
     i++;


}*/

    return tracks;

}
