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
  // Skalierungsfaktor für das clustering
  bestParams.cloudScaleModifier = 1.0;
  // Radius zum glätten der Punkte
  bestParams.smoothRadius = 7.0f;
  // Anzal nächste Nachbarn um Triplets zu erzeugen
  bestParams.genTripletsNnKandidates = 19;
  // Anzahl bester Triplets, die aus der Kandidatenmenge übernommen werden
  bestParams.genTripletsNBest = 3;
  // Vermutung: Maximaler Winkel zwichen den Geraden AB und BC im Triplet.
  bestParams.genTripletsMaxError = 0.015;
  // Schwellwert für den besten Cluster Abstand.
  // Darüber werden die Cluster nicht mehr vereinigt
  bestParams.bestClusterDistanceDelta = 5.0;  // Aymans: 2.0
  // Schwellwert zum Entfernen aller Cluster,
  // welche weniger Tripletten besitzen als angegeben.
  bestParams.cleanupMinTriplets = 5;
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

  if (opt_params.smoothRadius < 0.0) {
    float fq = msd::first_quartile(cloud_xyz);
    opt_params.smoothRadius = fq;
    //if (opt_verbose > 0) {
      //std::cout << "Computed smoothen radius: " << fq << std::endl;
    //}
  }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(
        new pcl::PointCloud<pcl::PointXYZI>());

      // cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
      cloud_xyzti_smooth =
          smoothenCloud::smoothenCloud(cloud_xyzti, opt_params.smoothRadius,false);  // radius


      // calculate cluster
      Cluster cluster;
      std::vector<hc::triplet> triplets;

      triplets = hc::generateTriplets(
          cloud_xyzti_smooth, opt_params.genTripletsNnKandidates,
          opt_params.genTripletsNBest, opt_params.genTripletsMaxError);

      cluster = use_hc(cloud_xyzti_smooth, triplets, opt_params.cloudScaleModifier,
                       opt_params.bestClusterDistanceDelta,
                       opt_params.cleanupMinTriplets, opt_verbose);

      //Adapt clusters to ATTrack
      //fTrackCand = clustersToTrack(cloud_xyzti,cluster,event);

      patternEvent->SetTrackCand(clustersToTrack(cloud_xyzti,cluster,event));

}

Cluster ATPATTERN::ATTrackFinderHC::use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               std::vector<hc::triplet> triplets, float scale, float cdist,
               size_t cleanup_min_triplets, int opt_verbose = 0) {

  hc::ScaleTripletMetric scale_triplet_metric(scale);
  hc::cluster_group result =
  hc::compute_hc(cloud, triplets, scale_triplet_metric, cdist, opt_verbose);
  hc::cluster_group const &cleaned_up_cluster_group =
  hc::cleanupClusterGroup(result, cleanup_min_triplets);

  return hc::toCluster(triplets, cleaned_up_cluster_group, cloud->size());
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

        for (size_t clusterIndex = 0; clusterIndex < clusters.size();
             ++clusterIndex) {

          ATTrack track; // One track per cluster

          pcl::PointIndicesPtr const &pointIndices = clusters[clusterIndex];
          // get color colour

          for (size_t i = 0; i < pointIndices->indices.size(); ++i) {
            int index = pointIndices->indices[i];
            pcl::PointXYZI point = cloud->points[index];


            if(event.GetHit(point.intensity)) track.AddHit(event.GetHit(point.intensity));


                      // remove clustered points from point-vector
                      for (std::vector<pcl::PointXYZI,
                                       Eigen::aligned_allocator<pcl::PointXYZI> >::iterator it =
                               points.end();
                           it != points.begin(); --it) {
                        if (it->x == point.x && it->y == point.y && it->z == point.z) {
                          points.erase(it);
                          break;
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

        return tracks;

}
