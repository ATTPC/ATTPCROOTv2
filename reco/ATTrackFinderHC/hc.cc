#include "fastcluster.hh"
#include "hc.hh"

#pragma warning(push, 0)
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <fstream>
#pragma warning(pop)

namespace hc {
std::vector<triplet> generateTriplets(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, size_t nnKandidates,
    size_t nBest, float maxError) {
  std::vector<triplet> triplets;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t pointIndexB = 0; pointIndexB < cloud->size(); ++pointIndexB) {
    pcl::PointXYZI pointB = (*cloud)[pointIndexB];
    Eigen::Vector3f pointBEigen(pointB.x, pointB.y, pointB.z);

    std::vector<triplet> tripletCandidates;

    std::vector<int> nnIndices;
    nnIndices.reserve(nnKandidates);
    std::vector<float> nnbSquaredDistances;
    nnbSquaredDistances.reserve(nnKandidates);
    int const nnFound =
        kdtree.nearestKSearch(*cloud, (int)pointIndexB, (int)nnKandidates,
                              nnIndices, nnbSquaredDistances);

    for (size_t pointIndexIndexA = 0; pointIndexIndexA < nnFound;
         ++pointIndexIndexA) {
      size_t const pointIndexA = nnIndices[pointIndexIndexA];
      pcl::PointXYZI pointA = (*cloud)[pointIndexA];
      Eigen::Vector3f pointAEigen(pointA.x, pointA.y, pointA.z);

      Eigen::Vector3f directionAB = pointBEigen - pointAEigen;
      directionAB /= directionAB.norm();

      for (size_t pointIndexIndexC = pointIndexIndexA + 1;
           pointIndexIndexC < nnFound; ++pointIndexIndexC) {
        size_t const pointIndexC = nnIndices[pointIndexIndexC];
        pcl::PointXYZI pointC = (*cloud)[pointIndexC];
        Eigen::Vector3f pointCEigen(pointC.x, pointC.y, pointC.z);

        Eigen::Vector3f directionBC = pointCEigen - pointBEigen;
        directionBC /= directionBC.norm();

        float const angle = directionAB.dot(directionBC);

        // calculate error
        float const error = -0.5f * (angle - 1.0f);

        if (error <= maxError) {
          // calculate center
          Eigen::Vector3f center =
              (pointAEigen + pointBEigen + pointCEigen) / 3.0f;

          // calculate direction
          Eigen::Vector3f direction = pointCEigen - pointBEigen;
          direction /= direction.norm();

          triplet newTriplet;

          newTriplet.pointIndexA = pointIndexA;
          newTriplet.pointIndexB = pointIndexB;
          newTriplet.pointIndexC = pointIndexC;
          newTriplet.center = center;
          newTriplet.direction = direction;
          newTriplet.error = error;

          tripletCandidates.push_back(newTriplet);
        }
      }
    }

    // order triplet candidates
    // std::sort(tripletCandidates.begin(), tripletCandidates.end(), [](triplet
    // const &lhs, triplet const &rhs) { return lhs.error < rhs.error; });
    std::sort(tripletCandidates.begin(), tripletCandidates.end());

    // use the n best candidates
    for (size_t i = 0; i < std::min(nBest, tripletCandidates.size()); ++i) {
      triplets.push_back(tripletCandidates[i]);
    }
  }

  return triplets;
}

/*
static Eigen::MatrixXf calculateDistanceMatrix(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    std::vector<triplet> const &triplets, ScaleTripletMetric tripletMetric) {
  size_t const tripletSize = triplets.size();
  Eigen::MatrixXf result(tripletSize, tripletSize);

  for (size_t i = 0; i < tripletSize; ++i) {
    result(i, i) = 0.0f;

    for (size_t j = i + 1; j < tripletSize; ++j) {
      result(i, j) = tripletMetric(triplets[i], triplets[j], cloud);
      result(j, i) = result(i, j);
    }
  }

  return result;
}
*/

/*
@brief computation of condensed distance matrix.

This function computes the condensed distance matrix and returns i as an array.

@param  cloud is the pointcloud
@param  triplets is a list of triplets
@param  triplet_metric the metric to compute the distance between triplets
@return returns an arry of double as condensed distance matrix
*/
double *calculate_distance_matrix(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    std::vector<triplet> const &triplets, ScaleTripletMetric triplet_metric) {
  size_t const triplet_size = triplets.size();
  size_t k = 0;
  double *result = new double[(triplet_size * (triplet_size - 1)) / 2];

  for (size_t i = 0; i < triplet_size; ++i) {
    for (size_t j = i + 1; j < triplet_size; j++) {
      result[k++] = triplet_metric(triplets[i], triplets[j], cloud);
    }
  }
  return result;
}

/*
@brief computation of the clustering.

This function computes the clustering. It uses the C++ standalone verion
of fastcluster.

@param  cloud is the pointcloud
@param  triplets is a list of triplets
@param  triplet_metric the metric to compute the distance between triplets
@param  cdist the distance for splitting the dendrogram into clusters
@param  opt_verbose verbosity
@return returns a cluster_group
*/
cluster_group compute_hc(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                         std::vector<triplet> const &triplets,
                         ScaleTripletMetric triplet_metric, float cdist,
                         int opt_verbose) {
  size_t const triplet_size = triplets.size();
  size_t k, cluster_size, i;
  cluster_group result;

  if (!triplet_size) {
    // if no triplets are generated
    return result;
  }

  double *distance_matrix, *height = new double[triplet_size - 1];
  int *merge = new int[2 * (triplet_size - 1)], *labels = new int[triplet_size];

  distance_matrix = calculate_distance_matrix(cloud, triplets, triplet_metric);

  hclust_fast(triplet_size, distance_matrix, HCLUST_METHOD_SINGLE, merge,
              height);

  // splitting the dendrogram into clusters
  for (k = 0; k < (triplet_size - 1); ++k) {
    if (height[k] >= cdist) {
      break;
    }
  }
  cluster_size = triplet_size - k;
  cutree_k(triplet_size, merge, cluster_size, labels);

  // generate clusters
  for (i = 0; i < cluster_size; ++i) {
    cluster new_cluster;
    result.clusters.push_back(new_cluster);
  }
  result.bestClusterDistance = height[k - 1];

  for (i = 0; i < triplet_size; ++i) {
    result.clusters[labels[i]].push_back(i);
  }

  if (opt_verbose > 1) {
    // write debug file
    const char *fname = "debug_cdists.csv";
    std::ofstream of(fname);
    if (of.is_open()) {
      for (size_t i = 0; i < (triplet_size - 1); ++i) {
        of << height[i] << std::endl;
      }
    } else {
      std::cerr << "Could Not write file '" << fname << "'\n";
    }
    of.close();
  }

  // cleanup
  delete[] distance_matrix;
  delete[] height;
  delete[] merge;
  delete[] labels;

  return result;
}

/*
cluster_history calculateHc(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                            std::vector<triplet> const &triplets,
                            ScaleTripletMetric tripletMetric,
                            ClusterMetric clusterMetric, int opt_verbose) {
  cluster_history result;
  std::vector<float> cdist;

  result.triplets = triplets;

  // calculate distance-Matrix
  Eigen::MatrixXf distanceMatrix =
      calculateDistanceMatrix(cloud, result.triplets, tripletMetric);

  cluster_group currentGeneration;

  // init first generation
  for (size_t i = 0; i < result.triplets.size(); ++i) {
    cluster newCluster;
    newCluster.push_back(i);
    currentGeneration.clusters.push_back(newCluster);
  }

  // merge until only one cluster left
  while (currentGeneration.clusters.size() > 1) {
    float bestClusterDistance = std::numeric_limits<float>::infinity();
    std::pair<size_t, size_t> bestClusterPair;

    // find best cluster-pair
    for (size_t i = 0; i < currentGeneration.clusters.size(); ++i) {
      for (size_t j = i + 1; j < currentGeneration.clusters.size(); ++j) {
        float const clusterDistance =
            clusterMetric(currentGeneration.clusters[i],
                          currentGeneration.clusters[j], distanceMatrix, cloud);

        if (clusterDistance < bestClusterDistance) {
          bestClusterDistance = clusterDistance;
          bestClusterPair = std::pair<size_t, size_t>(i, j);
        }
      }
    }

    if (opt_verbose > 1)
      cdist.push_back(
          bestClusterDistance);  // save best clusterdistance for debugging
    // merge cluster-pair
    cluster merged(currentGeneration.clusters[bestClusterPair.first]);
    merged.insert(merged.end(),
                  currentGeneration.clusters[bestClusterPair.second].begin(),
                  currentGeneration.clusters[bestClusterPair.second].end());

    // set best cluster distance for current generation and add to results
    currentGeneration.bestClusterDistance = bestClusterDistance;
    result.history.push_back(currentGeneration);

    // copy data to next generation
    cluster_group nextGeneration;
    nextGeneration.clusters.reserve(currentGeneration.clusters.size() - 1);

    for (size_t i = 0; i < currentGeneration.clusters.size(); ++i) {
      if (i != bestClusterPair.first && i != bestClusterPair.second)
        nextGeneration.clusters.push_back(currentGeneration.clusters[i]);
    }

    nextGeneration.clusters.push_back(merged);

    // overwrite current generation
    currentGeneration = nextGeneration;
  }

  // set last cluster distance and add to results
  currentGeneration.bestClusterDistance =
      std::numeric_limits<float>::infinity();
  result.history.push_back(currentGeneration);
  if (opt_verbose > 1) {
    // write debug file
    const char *fname = "debug_cdists.csv";
    std::ofstream of(fname);
    if (of.is_open()) {
      for (size_t i = 0; i < cdist.size(); ++i) {
        of << cdist[i] << std::endl;
      }
    } else {
      std::cerr << "Could Not write file '" << fname << "'\n";
    }
    of.close();
  }

  return result;
}


cluster_group getBestClusterGroup(cluster_history const &history,
                                  float bestClusterDistanceDelta) {
  float lastBestClusterDistance = history.history[0].bestClusterDistance;

  // for (cluster_group const &clusterGroup : history.history)
  for (std::vector<cluster_group>::const_iterator clusterGroup =
           history.history.begin();
       clusterGroup < history.history.end(); clusterGroup++) {
    float const bestClusterDistanceChange =
        clusterGroup->bestClusterDistance - lastBestClusterDistance;
    lastBestClusterDistance = clusterGroup->bestClusterDistance;

    if (bestClusterDistanceChange > bestClusterDistanceDelta)
      return *clusterGroup;
  }

  return history.history[history.history.size() - 1];
}
*/

cluster_group cleanupClusterGroup(cluster_group const &clusterGroup,
                                  size_t minTriplets) {
  cluster_group cleanedGroup;
  cleanedGroup.bestClusterDistance = clusterGroup.bestClusterDistance;
  for (size_t i = 0; i < clusterGroup.clusters.size(); i++) {
    if (clusterGroup.clusters[i].size() >= minTriplets)
      cleanedGroup.clusters.push_back(clusterGroup.clusters[i]);
  }

  // cleanedGroup.clusters.resize(clusterGroup.clusters.size());
  // auto newEnd = std::copy_if(clusterGroup.clusters.begin(),
  // clusterGroup.clusters.end(), cleanedGroup.clusters.begin(), [&](cluster
  // const &cluster) {
  //     return cluster.size() >= minTriplets;
  //   });
  // cleanedGroup.clusters.resize(std::distance(cleanedGroup.clusters.begin(),
  // newEnd));

  return cleanedGroup;
}

Cluster toCluster(std::vector<hc::triplet> const &triplets,
                  hc::cluster_group const &clusterGroup,
                  size_t pointIndexCount) {
  std::vector<pcl::PointIndicesPtr> result;


  // for (auto const &currentCluster : clusterGroup.clusters)
  for (std::vector<cluster>::const_iterator currentCluster =
           clusterGroup.clusters.begin();
       currentCluster < clusterGroup.clusters.end(); currentCluster++) {
    pcl::PointIndicesPtr pointIndices(new pcl::PointIndices());

    // add point indices
    // for (auto const &currentTripletIndex : currentCluster)
    for (std::vector<size_t>::const_iterator currentTripletIndex =
             currentCluster->begin();
         currentTripletIndex < currentCluster->end(); currentTripletIndex++) {
      hc::triplet const &currentTriplet = triplets[*currentTripletIndex];

      pointIndices->indices.push_back((int)currentTriplet.pointIndexA);
      pointIndices->indices.push_back((int)currentTriplet.pointIndexB);
      pointIndices->indices.push_back((int)currentTriplet.pointIndexC);
    }

    // sort point-indices and remove duplikates
    std::sort(pointIndices->indices.begin(), pointIndices->indices.end());
    std::vector<int>::iterator newEnd =
        std::unique(pointIndices->indices.begin(), pointIndices->indices.end());
    pointIndices->indices.resize(
        std::distance(pointIndices->indices.begin(), newEnd));

    result.push_back(pointIndices);
  }

  return Cluster(result, pointIndexCount);
}

/*
float TripletMetric::operator()(
    triplet const &lhs, triplet const &rhs,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
  float const perpendicularDistanceA =
      (rhs.center -
       (lhs.center +
        lhs.direction.dot(rhs.center - lhs.center) * lhs.direction))
          .squaredNorm();
  float const perpendicularDistanceB =
      (lhs.center -
       (rhs.center +
        rhs.direction.dot(lhs.center - rhs.center) * rhs.direction))
          .squaredNorm();

  float const angle = 1.0f - std::abs(lhs.direction.dot(rhs.direction));

  // std::cout << perpendicularDistanceA << ", " << perpendicularDistanceB << ",
  // " << std::max(perpendicularDistanceA, perpendicularDistanceB) << ',' <<
  // std::pow(2.0f, 1.0f + 12.0f * angle) << std::endl;

  // squared distances!
  return std::max(perpendicularDistanceA, perpendicularDistanceB) +
         std::pow(2.0f, 1.0f + 12.0f * angle);
}*/

ScaleTripletMetric::ScaleTripletMetric(float scale) { this->scale = scale; }

float ScaleTripletMetric::operator()(
    triplet const &lhs, triplet const &rhs,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
  float const perpendicularDistanceA =
      (rhs.center -
       (lhs.center +
        lhs.direction.dot(rhs.center - lhs.center) * lhs.direction))
          .squaredNorm();
  float const perpendicularDistanceB =
      (lhs.center -
       (rhs.center +
        rhs.direction.dot(lhs.center - rhs.center) * rhs.direction))
          .squaredNorm();

  float const angle =
      std::abs(std::tan(std::acos(lhs.direction.dot(rhs.direction))));
  return (std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) /
          this->scale) +
         angle;
}
}
