#ifndef SMOOTHENCLOUD_H
#define SMOOTHENCLOUD_H

#include <pcl/io/io.h>
#include <boost/shared_ptr.hpp>

namespace smoothenCloud
{

pcl::PointCloud<pcl::PointXYZ>::Ptr smoothenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k, bool useMedian);
pcl::PointCloud<pcl::PointXYZI>::Ptr smoothenCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double radius, bool useMedian);

}
#endif
