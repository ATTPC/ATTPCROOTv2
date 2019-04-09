#include "ATTrackFinderHC.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATPATTERN::ATTrackFinderHC)


ATPATTERN::ATTrackFinderHC::ATTrackFinderHC()
{

}

ATPATTERN::ATTrackFinderHC::~ATTrackFinderHC()
{

}

std::vector<ATTrack> ATPATTERN::ATTrackFinderHC::GetTrackCand() {return fTrackCand;}

bool ATPATTERN::ATTrackFinderHC::FindTracks(ATEvent &event, ATPatternEvent *patternEvent)
{

  int opt_verbose = 0;

  hc_params opt_params;

  hc_params bestParams;
  // ATTPC
  // Defaultvalues
  bestParams.s = -1.0;
  bestParams.r = -1.0;
  bestParams.k = 19;
  bestParams.n = 3;
  bestParams.a = 0.03;
  bestParams.t = 3.5;
  bestParams.m = 8;
  opt_params = bestParams;

  //Parse ATTPCROOT date into PCL format
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti(
      new pcl::PointCloud<pcl::PointXYZI>());

  ///TODO: Convert hit patern
  eventToClusters(event,cloud_xyzti);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cloud_xyzti, *cloud_xyz);
  if (cloud_xyz->size() == 0) {
    std::cerr << "Error: empty cloud <<""\n";
    return 0;
  }

  // compute default r if it is not given
  if (opt_params.r < 0.0) {
    float dnn = 2.0f * std::sqrt(msd::first_quartile(cloud_xyz));
    opt_params.r = dnn;
    if (opt_verbose > 0) {
      std::cout << "Computed smoothed radius: " << dnn << std::endl;
    }
  }

  // compute default s if it is not given
  if (opt_params.s < 0.0) {
    float dnn = std::sqrt(msd::first_quartile(cloud_xyz)) / 3.0f;
    opt_params.s = dnn;
    if (opt_verbose > 0) {
      std::cout << "Computed distance scale: " << dnn << std::endl;
    }
  }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(
        new pcl::PointCloud<pcl::PointXYZI>());


    cloud_xyzti_smooth = smoothenCloud(cloud_xyzti, opt_params.r,false);


      // calculate cluster
      Cluster cluster;
      std::vector<hc::triplet> triplets;

      triplets = hc::generateTriplets(
      cloud_xyzti_smooth, opt_params.k,
      opt_params.n, opt_params.a);

      cluster = use_hc(cloud_xyzti_smooth, triplets, opt_params.s,
                   opt_params.t,
                   opt_params.m, opt_verbose);

      //Adapt clusters to ATTrack
      //fTrackCand = clustersToTrack(cloud_xyzti,cluster,event);

      patternEvent->SetTrackCand(clustersToTrack(cloud_xyzti,cluster,event));

}

Cluster ATPATTERN::ATTrackFinderHC::use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               std::vector<hc::triplet> triplets, float scale, float cdist,
               size_t cleanup_min_triplets, int opt_verbose = 0) {

  hc::ScaleTripletMetric scale_triplet_metric(scale);

  Cluster empty_cluster;

  if(triplets.size()>5)
  {
    hc::cluster_group result =
    hc::compute_hc(cloud, triplets, scale_triplet_metric, cdist, opt_verbose);
    hc::cluster_group const &cleaned_up_cluster_group =
    hc::cleanupClusterGroup(result, cleanup_min_triplets);
    return hc::toCluster(triplets, cleaned_up_cluster_group, cloud->size());
  }else return empty_cluster; 
}

void ATPATTERN::ATTrackFinderHC::eventToClusters(ATEvent& event,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  Int_t nHits = event.GetNumHits();
  cloud->points.resize(nHits);

  for(Int_t iHit=0; iHit<nHits; iHit++){

        ATHit* hit = event.GetHit(iHit);
        Int_t PadNumHit = hit->GetHitPadNum();
        TVector3 position = hit->GetPosition();
        cloud->points[iHit].x = position.X();
        cloud->points[iHit].y = position.Y();
        cloud->points[iHit].z = position.Z();
        cloud->points[iHit].intensity = iHit; // Storing the position of the hit in the event container
        //std::cout<<position.Y()<<"\n";


  }


}

std::vector<ATTrack> ATPATTERN::ATTrackFinderHC::clustersToTrack(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,Cluster const cluster,ATEvent& event)
{
    std::vector<ATTrack> tracks;

    std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> > points =
        cloud->points;

        std::vector<pcl::PointIndicesPtr> clusters = cluster.getClusters();

        for (size_t clusterIndex = 0; clusterIndex < clusters.size(); ++clusterIndex) {

          ATTrack track; // One track per cluster

          pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
          // get color colour

          for (size_t i = 0; i < pointIndices->indices.size(); ++i) {
            int index = pointIndices->indices[i];
            pcl::PointXYZI point = cloud->points[index];


            if(event.GetHit(point.intensity)) track.AddHit(event.GetHit(point.intensity));


              // remove clustered points from point-vector
               for (std::vector<pcl::PointXYZI, Eigen::aligned_allocator<pcl::PointXYZI> >::iterator it =points.end();it != points.begin(); --it) {
                        if (it->x == point.x && it->y == point.y && it->z == point.z) {

                            if(it!=points.end()){
                                points.erase(it);                          
                                break;
                             }   
                         }
               }


          }//Indices loop

          tracks.push_back(track);

        }// Clusters loop

        std::cout<<cRED<<" Tracks found "<<tracks.size()<<cNORMAL<<"\n";

        //Dump noise into a track
        ATTrack track;
        for (std::vector<pcl::PointXYZI,
                         Eigen::aligned_allocator<pcl::PointXYZI> >::iterator it =
                 points.begin();
             it != points.end(); ++it){
              if(event.GetHit(it->intensity)) track.AddHit(event.GetHit(it->intensity));
        }
        track.SetIsNoise(kTRUE);
        tracks.push_back(track);

        /*ROOT::EnableThreadSafety();

        //Estimaton of track parameters
        std::vector<std::future<void>> futures;
        futures.reserve(10);

        for(auto& track : tracks)
        {

           futures.push_back( std::async(std::launch::async,&ATPRA::SetTrackInitialParameters,this,std::ref(track)) );
           //std::cout<<" Processing track "<<"\n";
           //SetTrackInitialParameters(track);

        }

        for(auto& future : futures) future.wait();

        for(auto& track : tracks)
        { 
          std::vector<Double_t>& coeffs = track.GetRANSACCoeff();

            for(auto& coeff : coeffs){
              std::cout<<coeff<<"\n";
            }

        }*/

        return tracks;

}
