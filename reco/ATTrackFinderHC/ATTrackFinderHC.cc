#include "ATTrackFinderHC.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATTrackFinderHC)

ATTrackFinderHC::ATTrackFinderHC()
{

}

ATTrackFinderHC::~ATTrackFinderHC()
{

}


bool ATTrackFinderHC::FindTracks(ATEvent &event)
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



}

Cluster ATTrackFinderHC::use_hc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
               std::vector<hc::triplet> triplets, float scale, float cdist,
               size_t cleanup_min_triplets, int opt_verbose = 0) {
  hc::ScaleTripletMetric scale_triplet_metric(scale);
  hc::cluster_group result =
      hc::compute_hc(cloud, triplets, scale_triplet_metric, cdist, opt_verbose);
  hc::cluster_group const &cleaned_up_cluster_group =
      hc::cleanupClusterGroup(result, cleanup_min_triplets);

  return hc::toCluster(triplets, cleaned_up_cluster_group, cloud->size());
}

void ATTrackFinderHC::eventToClusters(ATEvent& event,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
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

void ATTrackFinderHC::clustersToTrack(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,Cluster const cluster)
{


}
