#ifndef HC_H
#define HC_H

#include <pcl/io/io.h>
#include <limits>
#include <vector>

#include "cluster.hh"

namespace hc {
struct triplet {
  size_t pointIndexA;
  size_t pointIndexB;
  size_t pointIndexC;
  Eigen::Vector3f center;
  Eigen::Vector3f direction;
  float error;
  friend bool operator<(const triplet &t1, const triplet &t2) {
    return (t1.error < t2.error);
  };
};

typedef std::vector<size_t> cluster;

struct cluster_group {
  std::vector<cluster> clusters;
  float bestClusterDistance;
};

struct cluster_history {
  std::vector<triplet> triplets;
  std::vector<cluster_group> history;
};

class ScaleTripletMetric {
  float scale;

 public:
  ScaleTripletMetric(float s);
  float operator()(triplet const &lhs, triplet const &rhs,
                   pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);
};

typedef float (*ClusterMetric)(cluster const &lhs, cluster const &rhs,
                               Eigen::MatrixXf const &d,
                               pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud);

inline float singleLinkClusterMetric(
    cluster const &lhs, cluster const &rhs, Eigen::MatrixXf const &d,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
  float result = std::numeric_limits<float>::infinity();

  for (std::vector<size_t>::const_iterator a = lhs.begin(); a < lhs.end();
       a++) {
    for (std::vector<size_t>::const_iterator b = rhs.begin(); b < rhs.end();
         b++) {
      float distance = d(*a, *b);

      if (distance < result) result = distance;
    }
  }

  return result;
}

inline float completeLinkClusterMetric(
    cluster const &lhs, cluster const &rhs, Eigen::MatrixXf const &d,
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) {
  float result = 0.0f;

  for (std::vector<size_t>::const_iterator a = lhs.begin(); a < lhs.end();
       a++) {
    for (std::vector<size_t>::const_iterator b = rhs.begin(); b < rhs.end();
         b++) {
      float distance = d(*a, *b);

      if (distance > result) result = distance;
    }
  }

  return result;
}

std::vector<triplet> generateTriplets(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, size_t nnKandidates = 12,
    size_t nBest = 2, float maxError = 0.015f);
cluster_group cleanupClusterGroup(cluster_group const &clusterGroup,
                                  size_t minTriplets = 7);
Cluster toCluster(std::vector<hc::triplet> const &triplets,
                  hc::cluster_group const &clusterGroup,
                  size_t pointIndexCount);
cluster_group compute_hc(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                         std::vector<triplet> const &triplets,
                         ScaleTripletMetric triplet_metric, float cdist,
                         int opt_verbose);
}  // namespace hc

#endif
