#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <numeric>
#include <vector>

#include "msd.hh"
namespace msd {
/*
@brief Compute mean squared distance.

This function computes the mean squared distance from a point to its k
neighbours. This is done for all points in the given point cloud.

@param  k       number of neighbours
@param  cloud   the point cloud
@return vector of mean squared distances of every point in the cloud
*/
std::vector<float> compute_mean_square_distance(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k) {
  // compute mean square distances for every point to its k nearest neighbours
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<float> msd;

  k++;  // k must be one higher because the first point found by the kdtree is
        // the point itself
  kdtree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    // compute
    float sum;
    std::vector<int> nnIndices;
    std::vector<float> knnSquaredDistances;
    nnIndices.reserve(k);
    knnSquaredDistances.reserve(k);
    kdtree.nearestKSearch(cloud->at(i), k, nnIndices, knnSquaredDistances);

    knnSquaredDistances.erase(knnSquaredDistances.begin());  // The first value
                                                             // must be deleted
                                                             // because it is
                                                             // the distance
                                                             // with the point
                                                             // itself

    sum = std::accumulate(knnSquaredDistances.begin(),
                          knnSquaredDistances.end(), 0.0);
    msd.push_back(sum / knnSquaredDistances.size());
  }
  return msd;
}

/*
@brief Compute first quartile mean squared distance.

This function computes the first quartile of the mean squared distances from
every point to its nearest neighbour. This is done for all points in the given
oint cloud.

@param  cloud   the point cloud
@return first quartile
*/
float first_quartile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::vector<float> msd = compute_mean_square_distance(cloud, 1), lower_half;
  float const q1 = msd.size() /4;
  std::nth_element(msd.begin(), msd.begin() + q1, msd.end());
  return msd[q1];
}
}
