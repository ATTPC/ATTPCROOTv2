#ifndef SMOOTHENCLOUD_H
#define SMOOTHENCLOUD_H

#include <pcl/point_cloud.h> // for PointCloud, PointCloud<>::Ptr
#include <pcl/point_types.h> // for PointXYZI

pcl::PointCloud<pcl::PointXYZI>::Ptr
smoothenCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double radius, bool useMedian = false);

#endif
