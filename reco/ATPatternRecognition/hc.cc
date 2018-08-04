#include "fastcluster.hh"
#include "hc.hh"

#pragma warning(push, 0)
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <fstream>
#pragma warning(pop)

#pragma warning(push, 0)
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <fstream>
#pragma warning(pop)

namespace hc {
std::vector<triplet> generateTriplets(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, size_t k, size_t n,
    float a) {
  std::vector<triplet> triplets;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(cloud);

  for (size_t pointIndexB = 0; pointIndexB < cloud->size(); ++pointIndexB) {
    pcl::PointXYZI pointB = (*cloud)[pointIndexB];
    Eigen::Vector3f pointBEigen(pointB.x, pointB.y, pointB.z);

    std::vector<triplet> tripletCandidates;

    std::vector<int> nnIndices;
    nnIndices.reserve(k);
    std::vector<float> nnbSquaredDistances;
    nnbSquaredDistances.reserve(k);
    int const nnFound = kdtree.nearestKSearch(*cloud, (int)pointIndexB, (int)k,
                                              nnIndices, nnbSquaredDistances);

    for (size_t pointIndexIndexA = 1; pointIndexIndexA < nnFound;
         ++pointIndexIndexA) {
      // When the distance is 0, we have the same point as pointB
      if (nnbSquaredDistances[pointIndexIndexA] == 0) continue;
      size_t const pointIndexA = nnIndices[pointIndexIndexA];
      pcl::PointXYZI pointA = (*cloud)[pointIndexA];
      Eigen::Vector3f pointAEigen(pointA.x, pointA.y, pointA.z);

      Eigen::Vector3f directionAB = pointBEigen - pointAEigen;
      directionAB /= directionAB.norm();

      for (size_t pointIndexIndexC = pointIndexIndexA + 1;
           pointIndexIndexC < nnFound; ++pointIndexIndexC) {
        // When the distance is 0, we have the same point as pointB
        if (nnbSquaredDistances[pointIndexIndexC] == 0) continue;
        size_t const pointIndexC = nnIndices[pointIndexIndexC];
        pcl::PointXYZI pointC = (*cloud)[pointIndexC];
        Eigen::Vector3f pointCEigen(pointC.x, pointC.y, pointC.z);

        Eigen::Vector3f directionBC = pointCEigen - pointBEigen;
        directionBC /= directionBC.norm();

        float const angle = directionAB.dot(directionBC);

        // calculate error
        float const error = 1.0f - angle;  // removed: -0.5f

        if (error <= a) {
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
    std::sort(tripletCandidates.begin(), tripletCandidates.end());

    // use the n best candidates
    for (size_t i = 0; i < std::min(n, tripletCandidates.size()); ++i) {
      triplets.push_back(tripletCandidates[i]);
    }
  }

  return triplets;
}

/*
@brief computation of condensed distance matrix.

This function computes the condensed distance matrix and returns it as an array.

@param  cloud is the pointcloud
@param  triplets is a list of triplets
@param  triplet_metric the metric to compute the distance between triplets
@return returns an array of double as condensed distance matrix
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

This function computes the clustering. It uses the C++ standalone version
of fastcluster.

@param  cloud is the pointcloud
@param  triplets is a list of triplets
@param  triplet_metric the metric to compute the distance between triplets
@param  t the distance for splitting the dendrogram into clusters
@param  opt_verbose verbosity
@return returns a cluster_group
*/
cluster_group compute_hc(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                         std::vector<triplet> const &triplets,
                         ScaleTripletMetric triplet_metric, float t,
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
    if (height[k] >= t) {
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
    const char *fname = "debug_ts.csv";
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

cluster_group cleanupClusterGroup(cluster_group const &clusterGroup, size_t m) {
  cluster_group cleanedGroup;
  cleanedGroup.bestClusterDistance = clusterGroup.bestClusterDistance;
  for (size_t i = 0; i < clusterGroup.clusters.size(); i++) {
    if (clusterGroup.clusters[i].size() >= m)
      cleanedGroup.clusters.push_back(clusterGroup.clusters[i]);
  }

  return cleanedGroup;
}

Cluster toCluster(std::vector<hc::triplet> const &triplets,
                  hc::cluster_group const &clusterGroup,
                  size_t pointIndexCount) {
  std::vector<pcl::PointIndicesPtr> result;

  for (std::vector<cluster>::const_iterator currentCluster =
           clusterGroup.clusters.begin();
       currentCluster < clusterGroup.clusters.end(); currentCluster++) {
    pcl::PointIndicesPtr pointIndices(new pcl::PointIndices());

    // add point indices
    for (std::vector<size_t>::const_iterator currentTripletIndex =
             currentCluster->begin();
         currentTripletIndex < currentCluster->end(); currentTripletIndex++) {
      hc::triplet const &currentTriplet = triplets[*currentTripletIndex];

      pointIndices->indices.push_back((int)currentTriplet.pointIndexA);
      pointIndices->indices.push_back((int)currentTriplet.pointIndexB);
      pointIndices->indices.push_back((int)currentTriplet.pointIndexC);
    }

    // sort point-indices and remove duplicates
    std::sort(pointIndices->indices.begin(), pointIndices->indices.end());
    std::vector<int>::iterator newEnd =
        std::unique(pointIndices->indices.begin(), pointIndices->indices.end());
    pointIndices->indices.resize(
        std::distance(pointIndices->indices.begin(), newEnd));

    result.push_back(pointIndices);
  }

  return Cluster(result, pointIndexCount);
}

ScaleTripletMetric::ScaleTripletMetric(float s) { this->scale = s; }

float ScaleTripletMetric::operator()(
    triplet const &lhs, triplet const &rhs,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
  float const perpendicularDistanceA =
      (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) *
                                      lhs.direction))
          .squaredNorm();
  float const perpendicularDistanceB =
      (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) *
                                      rhs.direction))
          .squaredNorm();

  float const angle =
      std::abs(std::tan(std::acos(lhs.direction.dot(rhs.direction))));
  return (std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) /
          this->scale) +
         angle;
}
}  // namespace hc
