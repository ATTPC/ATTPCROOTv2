#ifndef SMOOTHENCLOUD_H
#define SMOOTHENCLOUD_H

#include <pcl/io/io.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr smoothenCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double radius,
    bool useMedian = false);

#endif
