#include "smoothenCloud.hh"

#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr smoothenCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double r, bool useMedian) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  if (r == 0){
    pcl::copyPointCloud(*cloud, *result_cloud);
    return result_cloud;
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

  kdtree.setInputCloud(cloud);

  for (size_t i = 0; i < cloud->size(); ++i) {
    pcl::PointXYZI newPoint;

    pcl::CentroidPoint<pcl::PointXYZI> centroid;
    pcl::PointXYZI centroidPoint;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    int found = kdtree.radiusSearch(*cloud, (int)i, r, pointIdxNKNSearch,
                                    pointNKNSquaredDistance);

    for (int j = 0; j < found; ++j) {
      centroid.add((*cloud)[pointIdxNKNSearch[j]]);
    }

    centroid.get(centroidPoint);

    if (useMedian) {
      std::vector<float> xList;
      std::vector<float> yList;
      std::vector<float> zList;

      for (int j = 0; j < found; ++j) {
        pcl::PointXYZI const &point = (*cloud)[pointIdxNKNSearch[j]];

        xList.push_back(point.x);
        yList.push_back(point.y);
        zList.push_back(point.z);
      }

      std::vector<float>::iterator xMedianIt =
          xList.begin() + (xList.size() / 2);
      std::nth_element(xList.begin(), xMedianIt, xList.end());
      newPoint.x = *xMedianIt;

      std::vector<float>::iterator yMedianIt =
          yList.begin() + (yList.size() / 2);
      std::nth_element(yList.begin(), yMedianIt, yList.end());
      newPoint.y = *yMedianIt;

      std::vector<float>::iterator zMedianIt =
          zList.begin() + (zList.size() / 2);
      std::nth_element(zList.begin(), zMedianIt, zList.end());
      newPoint.z = *zMedianIt;
    } else
      newPoint = centroidPoint;

    result_cloud->push_back(newPoint);
  }

  return result_cloud;
}
